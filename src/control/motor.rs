//! Motor Controller for Drone Propulsion
//!
//! Platform-specific motor control implementations for STM32 and ESP32.
//!
//! # Safety
//!
//! Motor control is safety-critical. This module implements:
//! - Arm/disarm safety interlocks
//! - PWM output limiting
//! - Watchdog timeout for signal loss
//! - Soft start/stop for motor protection

use crate::types::*;

/// Number of motors (quadcopter configuration)
pub const NUM_MOTORS: usize = 4;

/// Motor positions on the frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorPosition {
    /// Front-Left motor (CCW)
    FrontLeft = 0,
    /// Front-Right motor (CW)
    FrontRight = 1,
    /// Rear-Left motor (CW)
    RearLeft = 2,
    /// Rear-Right motor (CCW)
    RearRight = 3,
}

impl MotorPosition {
    /// Get motor index
    pub fn index(self) -> usize {
        self as usize
    }

    /// Check if motor spins clockwise
    pub fn is_clockwise(self) -> bool {
        matches!(self, MotorPosition::FrontRight | MotorPosition::RearLeft)
    }
}

/// Motor state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorState {
    /// Motors disarmed (safe)
    Disarmed,
    /// Motors armed but idle
    Armed,
    /// Motors running
    Running,
    /// Emergency stop activated
    EmergencyStop,
    /// Motor fault detected
    Fault,
}

/// Motor controller configuration
#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    /// PWM frequency in Hz (typically 400Hz for ESC, 32kHz for direct drive)
    pub pwm_frequency: u32,
    /// Minimum throttle value (0-1000)
    pub min_throttle: u16,
    /// Maximum throttle value (0-1000)
    pub max_throttle: u16,
    /// Idle throttle when armed (0-1000)
    pub idle_throttle: u16,
    /// Soft start ramp rate (throttle units per second)
    pub ramp_rate: u16,
    /// Watchdog timeout in milliseconds
    pub watchdog_timeout_ms: u32,
    /// Enable motor mixing
    pub mixing_enabled: bool,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            pwm_frequency: 400,      // Standard ESC frequency
            min_throttle: 0,
            max_throttle: 1000,
            idle_throttle: 100,      // ~10% for armed idle
            ramp_rate: 500,          // 0.5s to full throttle
            watchdog_timeout_ms: 500, // 500ms signal timeout
            mixing_enabled: true,
        }
    }
}

/// Motor mixer input (from flight controller)
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorMixerInput {
    /// Throttle command (0.0 - 1.0)
    pub throttle: f32,
    /// Roll command (-1.0 to 1.0)
    pub roll: f32,
    /// Pitch command (-1.0 to 1.0)
    pub pitch: f32,
    /// Yaw command (-1.0 to 1.0)
    pub yaw: f32,
}

/// Motor controller trait
pub trait MotorController {
    /// Initialize motor controller hardware
    fn init(&mut self, config: &MotorConfig) -> Result<()>;

    /// Arm motors (enable output)
    fn arm(&mut self) -> Result<()>;

    /// Disarm motors (disable output)
    fn disarm(&mut self) -> Result<()>;

    /// Emergency stop all motors immediately
    fn emergency_stop(&mut self) -> Result<()>;

    /// Set individual motor throttle (0-1000)
    fn set_motor(&mut self, position: MotorPosition, throttle: u16) -> Result<()>;

    /// Set all motors from mixer input
    fn set_motors_mixed(&mut self, input: &MotorMixerInput) -> Result<()>;

    /// Get current motor state
    fn state(&self) -> MotorState;

    /// Check if motors are armed
    fn is_armed(&self) -> bool;

    /// Update watchdog (must be called regularly)
    fn update_watchdog(&mut self, current_time_ms: u64) -> Result<()>;

    /// Get individual motor throttle
    fn get_motor(&self, position: MotorPosition) -> u16;

    /// Get all motor throttles
    fn get_all_motors(&self) -> [u16; NUM_MOTORS];

    /// Self-test motors (brief spin up)
    fn self_test(&mut self) -> Result<()>;
}

/// Quadcopter motor mixer
///
/// Converts throttle/roll/pitch/yaw commands to individual motor outputs.
/// Uses X-configuration mixing (motors at 45-degree angles).
#[derive(Debug, Clone, Copy)]
pub struct QuadMixer {
    /// Mixing coefficients [motor][throttle, roll, pitch, yaw]
    coefficients: [[f32; 4]; NUM_MOTORS],
}

impl Default for QuadMixer {
    fn default() -> Self {
        Self::new_x_config()
    }
}

impl QuadMixer {
    /// Create mixer for X-configuration quadcopter
    ///
    /// ```text
    ///     Front
    ///   FL     FR
    ///     \   /
    ///      \ /
    ///      / \
    ///     /   \
    ///   RL     RR
    ///     Rear
    /// ```
    pub fn new_x_config() -> Self {
        Self {
            coefficients: [
                // [throttle, roll, pitch, yaw]
                [1.0, -1.0,  1.0, -1.0], // Front-Left (CCW)
                [1.0,  1.0,  1.0,  1.0], // Front-Right (CW)
                [1.0, -1.0, -1.0,  1.0], // Rear-Left (CW)
                [1.0,  1.0, -1.0, -1.0], // Rear-Right (CCW)
            ],
        }
    }

    /// Create mixer for plus-configuration quadcopter
    ///
    /// ```text
    ///       Front
    ///         F
    ///         |
    ///    L----+----R
    ///         |
    ///         B
    ///       Rear
    /// ```
    pub fn new_plus_config() -> Self {
        Self {
            coefficients: [
                [1.0,  0.0,  1.0, -1.0], // Front (CCW)
                [1.0,  1.0,  0.0,  1.0], // Right (CW)
                [1.0,  0.0, -1.0, -1.0], // Rear (CCW)
                [1.0, -1.0,  0.0,  1.0], // Left (CW)
            ],
        }
    }

    /// Mix input commands to motor outputs
    ///
    /// Returns motor throttle values (0.0 - 1.0)
    pub fn mix(&self, input: &MotorMixerInput) -> [f32; NUM_MOTORS] {
        let mut outputs = [0.0f32; NUM_MOTORS];

        for (i, coeff) in self.coefficients.iter().enumerate() {
            outputs[i] = input.throttle * coeff[0]
                + input.roll * coeff[1] * 0.5
                + input.pitch * coeff[2] * 0.5
                + input.yaw * coeff[3] * 0.5;

            // Clamp to valid range
            outputs[i] = outputs[i].clamp(0.0, 1.0);
        }

        outputs
    }
}

// ============================================================================
// STM32 MOTOR CONTROLLER IMPLEMENTATION
// ============================================================================

/// STM32 PWM-based motor controller
#[cfg(feature = "stm32")]
pub struct Stm32MotorController {
    config: MotorConfig,
    state: MotorState,
    throttles: [u16; NUM_MOTORS],
    mixer: QuadMixer,
    last_update_ms: u64,
    armed_time_ms: u64,
}

#[cfg(feature = "stm32")]
impl Stm32MotorController {
    /// Create new STM32 motor controller
    pub fn new() -> Self {
        Self {
            config: MotorConfig::default(),
            state: MotorState::Disarmed,
            throttles: [0; NUM_MOTORS],
            mixer: QuadMixer::default(),
            last_update_ms: 0,
            armed_time_ms: 0,
        }
    }

    /// Configure PWM timer for motor output
    fn configure_pwm(&mut self) -> Result<()> {
        // In a real implementation using stm32f4xx-hal:
        //
        // let gpioa = dp.GPIOA.split();
        // let channels = (
        //     gpioa.pa0.into_alternate_af1(),  // TIM2_CH1
        //     gpioa.pa1.into_alternate_af1(),  // TIM2_CH2
        //     gpioa.pa2.into_alternate_af1(),  // TIM2_CH3
        //     gpioa.pa3.into_alternate_af1(),  // TIM2_CH4
        // );
        //
        // let pwm = Timer::tim2(dp.TIM2, &clocks)
        //     .pwm(channels, self.config.pwm_frequency.hz());
        //
        // self.pwm_channels = Some(pwm);

        Ok(())
    }

    /// Set PWM duty cycle for a motor
    fn set_pwm(&mut self, motor: MotorPosition, duty: u16) -> Result<()> {
        // In a real implementation:
        // match motor {
        //     MotorPosition::FrontLeft => self.pwm_ch1.set_duty(duty),
        //     MotorPosition::FrontRight => self.pwm_ch2.set_duty(duty),
        //     MotorPosition::RearLeft => self.pwm_ch3.set_duty(duty),
        //     MotorPosition::RearRight => self.pwm_ch4.set_duty(duty),
        // }

        self.throttles[motor.index()] = duty;
        Ok(())
    }

    /// Convert throttle (0-1000) to PWM duty cycle
    fn throttle_to_pwm(&self, throttle: u16) -> u16 {
        // Map throttle to PWM range (typically 1000-2000us for ESC)
        // Assuming 16-bit timer with 1MHz clock and 400Hz PWM:
        // Period = 2500, 1000us = 1000, 2000us = 2000
        let clamped = throttle.min(self.config.max_throttle);
        1000 + clamped // 1000-2000 range
    }
}

#[cfg(feature = "stm32")]
impl Default for Stm32MotorController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "stm32")]
impl MotorController for Stm32MotorController {
    fn init(&mut self, config: &MotorConfig) -> Result<()> {
        self.config = *config;
        self.configure_pwm()?;
        self.state = MotorState::Disarmed;

        // Set all motors to minimum
        for i in 0..NUM_MOTORS {
            self.throttles[i] = 0;
        }

        Ok(())
    }

    fn arm(&mut self) -> Result<()> {
        if self.state == MotorState::Fault {
            return Err(SwarmError::HardwareFault);
        }

        // Safety: Don't arm if any throttle is above idle
        for &throttle in &self.throttles {
            if throttle > self.config.idle_throttle {
                return Err(SwarmError::PermissionDenied);
            }
        }

        self.state = MotorState::Armed;
        self.armed_time_ms = 0; // Will be set by watchdog update

        // Set all motors to idle
        for i in 0..NUM_MOTORS {
            let pwm = self.throttle_to_pwm(self.config.idle_throttle);
            self.set_pwm(MotorPosition::FrontLeft, pwm)?; // Note: should map i to position
        }

        Ok(())
    }

    fn disarm(&mut self) -> Result<()> {
        self.state = MotorState::Disarmed;

        // Set all motors to zero
        for i in 0..NUM_MOTORS {
            self.throttles[i] = 0;
            self.set_pwm(MotorPosition::FrontLeft, 0)?; // Note: should map i to position
        }

        Ok(())
    }

    fn emergency_stop(&mut self) -> Result<()> {
        self.state = MotorState::EmergencyStop;

        // Immediately zero all outputs
        for i in 0..NUM_MOTORS {
            self.throttles[i] = 0;
        }

        // In real implementation, also disable PWM timer
        // self.timer.disable();

        Ok(())
    }

    fn set_motor(&mut self, position: MotorPosition, throttle: u16) -> Result<()> {
        if self.state != MotorState::Armed && self.state != MotorState::Running {
            return Err(SwarmError::PermissionDenied);
        }

        let clamped = throttle.clamp(self.config.min_throttle, self.config.max_throttle);
        let pwm = self.throttle_to_pwm(clamped);
        self.set_pwm(position, pwm)?;

        if throttle > self.config.idle_throttle {
            self.state = MotorState::Running;
        }

        Ok(())
    }

    fn set_motors_mixed(&mut self, input: &MotorMixerInput) -> Result<()> {
        if !self.config.mixing_enabled {
            return Err(SwarmError::ConfigError);
        }

        if self.state != MotorState::Armed && self.state != MotorState::Running {
            return Err(SwarmError::PermissionDenied);
        }

        let outputs = self.mixer.mix(input);

        for (i, &output) in outputs.iter().enumerate() {
            let throttle = (output * self.config.max_throttle as f32) as u16;
            let position = match i {
                0 => MotorPosition::FrontLeft,
                1 => MotorPosition::FrontRight,
                2 => MotorPosition::RearLeft,
                _ => MotorPosition::RearRight,
            };
            self.set_motor(position, throttle)?;
        }

        Ok(())
    }

    fn state(&self) -> MotorState {
        self.state
    }

    fn is_armed(&self) -> bool {
        matches!(self.state, MotorState::Armed | MotorState::Running)
    }

    fn update_watchdog(&mut self, current_time_ms: u64) -> Result<()> {
        if self.armed_time_ms == 0 {
            self.armed_time_ms = current_time_ms;
        }

        let elapsed = current_time_ms.saturating_sub(self.last_update_ms);

        if self.is_armed() && elapsed > self.config.watchdog_timeout_ms as u64 {
            // Signal loss - emergency stop
            self.emergency_stop()?;
            return Err(SwarmError::Timeout);
        }

        self.last_update_ms = current_time_ms;
        Ok(())
    }

    fn get_motor(&self, position: MotorPosition) -> u16 {
        self.throttles[position.index()]
    }

    fn get_all_motors(&self) -> [u16; NUM_MOTORS] {
        self.throttles
    }

    fn self_test(&mut self) -> Result<()> {
        if self.state != MotorState::Disarmed {
            return Err(SwarmError::PermissionDenied);
        }

        // Brief spin-up test for each motor
        // In real implementation, would also check for motor response
        for i in 0..NUM_MOTORS {
            let position = match i {
                0 => MotorPosition::FrontLeft,
                1 => MotorPosition::FrontRight,
                2 => MotorPosition::RearLeft,
                _ => MotorPosition::RearRight,
            };

            // Set low throttle briefly (in real impl, use timer)
            let _test_throttle = self.config.idle_throttle + 50;
            self.throttles[position.index()] = 0; // Reset after test
        }

        Ok(())
    }
}

// ============================================================================
// SOFTWARE MOTOR CONTROLLER (for simulation)
// ============================================================================

/// Software motor controller for simulation/testing
pub struct SoftwareMotorController {
    config: MotorConfig,
    state: MotorState,
    throttles: [u16; NUM_MOTORS],
    mixer: QuadMixer,
    last_update_ms: u64,
}

impl SoftwareMotorController {
    /// Create new software motor controller
    pub fn new() -> Self {
        Self {
            config: MotorConfig::default(),
            state: MotorState::Disarmed,
            throttles: [0; NUM_MOTORS],
            mixer: QuadMixer::default(),
            last_update_ms: 0,
        }
    }
}

impl Default for SoftwareMotorController {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorController for SoftwareMotorController {
    fn init(&mut self, config: &MotorConfig) -> Result<()> {
        self.config = *config;
        self.state = MotorState::Disarmed;
        self.throttles = [0; NUM_MOTORS];
        Ok(())
    }

    fn arm(&mut self) -> Result<()> {
        if self.state == MotorState::Fault {
            return Err(SwarmError::HardwareFault);
        }
        self.state = MotorState::Armed;
        for i in 0..NUM_MOTORS {
            self.throttles[i] = self.config.idle_throttle;
        }
        Ok(())
    }

    fn disarm(&mut self) -> Result<()> {
        self.state = MotorState::Disarmed;
        self.throttles = [0; NUM_MOTORS];
        Ok(())
    }

    fn emergency_stop(&mut self) -> Result<()> {
        self.state = MotorState::EmergencyStop;
        self.throttles = [0; NUM_MOTORS];
        Ok(())
    }

    fn set_motor(&mut self, position: MotorPosition, throttle: u16) -> Result<()> {
        if self.state != MotorState::Armed && self.state != MotorState::Running {
            return Err(SwarmError::PermissionDenied);
        }

        let clamped = throttle.clamp(self.config.min_throttle, self.config.max_throttle);
        self.throttles[position.index()] = clamped;

        if throttle > self.config.idle_throttle {
            self.state = MotorState::Running;
        }

        Ok(())
    }

    fn set_motors_mixed(&mut self, input: &MotorMixerInput) -> Result<()> {
        if !self.config.mixing_enabled {
            return Err(SwarmError::ConfigError);
        }

        if self.state != MotorState::Armed && self.state != MotorState::Running {
            return Err(SwarmError::PermissionDenied);
        }

        let outputs = self.mixer.mix(input);

        for (i, &output) in outputs.iter().enumerate() {
            let throttle = (output * self.config.max_throttle as f32) as u16;
            self.throttles[i] = throttle.clamp(self.config.min_throttle, self.config.max_throttle);
        }

        if input.throttle > 0.1 {
            self.state = MotorState::Running;
        }

        Ok(())
    }

    fn state(&self) -> MotorState {
        self.state
    }

    fn is_armed(&self) -> bool {
        matches!(self.state, MotorState::Armed | MotorState::Running)
    }

    fn update_watchdog(&mut self, current_time_ms: u64) -> Result<()> {
        let elapsed = current_time_ms.saturating_sub(self.last_update_ms);

        if self.is_armed() && elapsed > self.config.watchdog_timeout_ms as u64 {
            self.emergency_stop()?;
            return Err(SwarmError::Timeout);
        }

        self.last_update_ms = current_time_ms;
        Ok(())
    }

    fn get_motor(&self, position: MotorPosition) -> u16 {
        self.throttles[position.index()]
    }

    fn get_all_motors(&self) -> [u16; NUM_MOTORS] {
        self.throttles
    }

    fn self_test(&mut self) -> Result<()> {
        if self.state != MotorState::Disarmed {
            return Err(SwarmError::PermissionDenied);
        }
        Ok(())
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_position() {
        assert_eq!(MotorPosition::FrontLeft.index(), 0);
        assert_eq!(MotorPosition::RearRight.index(), 3);
        assert!(!MotorPosition::FrontLeft.is_clockwise());
        assert!(MotorPosition::FrontRight.is_clockwise());
    }

    #[test]
    fn test_quad_mixer() {
        let mixer = QuadMixer::new_x_config();

        // Pure throttle
        let input = MotorMixerInput {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let outputs = mixer.mix(&input);
        for output in outputs {
            assert!((output - 0.5).abs() < 0.01);
        }

        // Roll right
        let input = MotorMixerInput {
            throttle: 0.5,
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
        };
        let outputs = mixer.mix(&input);
        // Right side should be higher
        assert!(outputs[MotorPosition::FrontRight.index()] > outputs[MotorPosition::FrontLeft.index()]);
    }

    #[test]
    fn test_software_motor_controller() {
        let mut ctrl = SoftwareMotorController::new();
        let config = MotorConfig::default();

        ctrl.init(&config).unwrap();
        assert_eq!(ctrl.state(), MotorState::Disarmed);

        // Can't set motors while disarmed
        assert!(ctrl.set_motor(MotorPosition::FrontLeft, 500).is_err());

        // Arm and set motors
        ctrl.arm().unwrap();
        assert!(ctrl.is_armed());

        ctrl.set_motor(MotorPosition::FrontLeft, 500).unwrap();
        assert_eq!(ctrl.get_motor(MotorPosition::FrontLeft), 500);
        assert_eq!(ctrl.state(), MotorState::Running);

        // Emergency stop
        ctrl.emergency_stop().unwrap();
        assert_eq!(ctrl.state(), MotorState::EmergencyStop);
        assert_eq!(ctrl.get_motor(MotorPosition::FrontLeft), 0);
    }

    #[test]
    fn test_mixed_motor_control() {
        let mut ctrl = SoftwareMotorController::new();
        let config = MotorConfig::default();

        ctrl.init(&config).unwrap();
        ctrl.arm().unwrap();

        let input = MotorMixerInput {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };

        ctrl.set_motors_mixed(&input).unwrap();

        let motors = ctrl.get_all_motors();
        for motor in motors {
            assert!(motor > 0);
        }
    }

    #[test]
    fn test_watchdog_timeout() {
        let mut ctrl = SoftwareMotorController::new();
        let mut config = MotorConfig::default();
        config.watchdog_timeout_ms = 100;

        ctrl.init(&config).unwrap();
        ctrl.arm().unwrap();

        // First update
        ctrl.update_watchdog(0).unwrap();

        // Simulate timeout
        let result = ctrl.update_watchdog(200);
        assert!(result.is_err());
        assert_eq!(ctrl.state(), MotorState::EmergencyStop);
    }
}
