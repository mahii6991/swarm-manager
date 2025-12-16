//! Failsafe Behaviors for Drone Safety
//!
//! Implements automatic safety responses for critical situations:
//! - Low battery → Return to launch / Land
//! - Communication loss → Hold / RTL / Land
//! - GPS failure → Altitude hold / Land
//! - Geofence breach → Return to boundary
//! - Motor failure → Emergency land
//! - Collision imminent → Emergency stop
//!
//! # Safety Philosophy
//! Failsafe actions are designed with these priorities:
//! 1. Protect human life
//! 2. Protect property
//! 3. Protect the drone
//! 4. Complete the mission
//!
//! # Example
//! ```ignore
//! use drone_swarm_system::failsafe::{FailsafeManager, FailsafeConfig};
//!
//! let mut fsm = FailsafeManager::new(FailsafeConfig::default());
//! fsm.update_state(current_position, battery_percent, gps_fix, ...);
//! let action = fsm.evaluate();
//! ```

/// Failsafe actions ordered by severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FailsafeAction {
    /// Continue normal operation
    None = 0,
    /// Issue warning but continue
    Warn = 1,
    /// Hold position
    Hold = 2,
    /// Reduce altitude
    Descend = 3,
    /// Return to launch
    ReturnToLaunch = 4,
    /// Land at current position
    Land = 5,
    /// Emergency motor stop (only when on ground or imminent crash)
    EmergencyStop = 6,
}

/// Failsafe trigger types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeTrigger {
    /// No failsafe active
    None,
    /// Low battery warning
    BatteryWarning,
    /// Critical battery
    BatteryCritical,
    /// RC link lost
    RcLost,
    /// Telemetry/GCS link lost
    DataLinkLost,
    /// GPS fix lost
    GpsLost,
    /// Geofence boundary breached
    GeofenceBreach,
    /// Motor failure detected
    MotorFailure,
    /// IMU/sensor failure
    SensorFailure,
    /// Collision imminent
    CollisionImminent,
    /// Maximum altitude exceeded
    AltitudeLimit,
    /// Maximum flight time exceeded
    FlightTimeLimit,
    /// Manual failsafe triggered
    ManualTrigger,
}

/// Failsafe configuration
#[derive(Debug, Clone)]
pub struct FailsafeConfig {
    // Battery thresholds
    /// Battery warning level (%)
    pub battery_warning_percent: u8,
    /// Battery critical level (%)
    pub battery_critical_percent: u8,
    /// Battery failsafe action
    pub battery_action: FailsafeAction,

    // Communication thresholds
    /// RC lost timeout (ms)
    pub rc_lost_timeout_ms: u32,
    /// RC lost action
    pub rc_lost_action: FailsafeAction,
    /// Data link lost timeout (ms)
    pub data_link_timeout_ms: u32,
    /// Data link lost action
    pub data_link_action: FailsafeAction,

    // GPS
    /// Minimum GPS satellites required
    pub min_gps_satellites: u8,
    /// GPS lost action
    pub gps_lost_action: FailsafeAction,

    // Geofence
    /// Enable geofence failsafe
    pub geofence_enabled: bool,
    /// Geofence breach action
    pub geofence_action: FailsafeAction,

    // Limits
    /// Maximum altitude (m AGL)
    pub max_altitude: f32,
    /// Altitude limit action
    pub altitude_action: FailsafeAction,
    /// Maximum flight time (seconds)
    pub max_flight_time_secs: u32,
    /// Flight time limit action
    pub flight_time_action: FailsafeAction,

    // Motor/Sensor
    /// Motor failure action
    pub motor_failure_action: FailsafeAction,
    /// Sensor failure action
    pub sensor_failure_action: FailsafeAction,

    // Collision
    /// Enable collision avoidance failsafe
    pub collision_avoidance_enabled: bool,
    /// Collision avoidance action
    pub collision_action: FailsafeAction,

    // Landing
    /// Descend rate during failsafe land (m/s)
    pub failsafe_descend_rate: f32,
    /// Minimum altitude to allow RTL (below this, land instead)
    pub min_rtl_altitude: f32,
}

impl Default for FailsafeConfig {
    fn default() -> Self {
        Self {
            battery_warning_percent: 30,
            battery_critical_percent: 15,
            battery_action: FailsafeAction::ReturnToLaunch,

            rc_lost_timeout_ms: 2000,
            rc_lost_action: FailsafeAction::Hold,
            data_link_timeout_ms: 5000,
            data_link_action: FailsafeAction::ReturnToLaunch,

            min_gps_satellites: 6,
            gps_lost_action: FailsafeAction::Descend,

            geofence_enabled: true,
            geofence_action: FailsafeAction::ReturnToLaunch,

            max_altitude: 120.0,
            altitude_action: FailsafeAction::Descend,
            max_flight_time_secs: 1800, // 30 minutes
            flight_time_action: FailsafeAction::ReturnToLaunch,

            motor_failure_action: FailsafeAction::Land,
            sensor_failure_action: FailsafeAction::Land,

            collision_avoidance_enabled: true,
            collision_action: FailsafeAction::Hold,

            failsafe_descend_rate: 1.0,
            min_rtl_altitude: 10.0,
        }
    }
}

impl FailsafeConfig {
    /// Create a conservative (safer) configuration
    pub fn conservative() -> Self {
        Self {
            battery_warning_percent: 40,
            battery_critical_percent: 25,
            battery_action: FailsafeAction::Land,

            rc_lost_timeout_ms: 1000,
            rc_lost_action: FailsafeAction::ReturnToLaunch,
            data_link_timeout_ms: 3000,
            data_link_action: FailsafeAction::Land,

            min_gps_satellites: 8,
            gps_lost_action: FailsafeAction::Land,

            geofence_enabled: true,
            geofence_action: FailsafeAction::Land,

            max_altitude: 100.0,
            altitude_action: FailsafeAction::Land,
            max_flight_time_secs: 1200, // 20 minutes
            flight_time_action: FailsafeAction::Land,

            motor_failure_action: FailsafeAction::EmergencyStop,
            sensor_failure_action: FailsafeAction::Land,

            collision_avoidance_enabled: true,
            collision_action: FailsafeAction::EmergencyStop,

            failsafe_descend_rate: 0.5,
            min_rtl_altitude: 15.0,
        }
    }

    /// Create an aggressive (mission-focused) configuration
    pub fn aggressive() -> Self {
        Self {
            battery_warning_percent: 20,
            battery_critical_percent: 10,
            battery_action: FailsafeAction::ReturnToLaunch,

            rc_lost_timeout_ms: 5000,
            rc_lost_action: FailsafeAction::Hold,
            data_link_timeout_ms: 10000,
            data_link_action: FailsafeAction::Hold,

            min_gps_satellites: 4,
            gps_lost_action: FailsafeAction::Hold,

            geofence_enabled: true,
            geofence_action: FailsafeAction::Hold,

            max_altitude: 150.0,
            altitude_action: FailsafeAction::Descend,
            max_flight_time_secs: 2400, // 40 minutes
            flight_time_action: FailsafeAction::Warn,

            motor_failure_action: FailsafeAction::Land,
            sensor_failure_action: FailsafeAction::Hold,

            collision_avoidance_enabled: true,
            collision_action: FailsafeAction::Hold,

            failsafe_descend_rate: 2.0,
            min_rtl_altitude: 5.0,
        }
    }
}

/// Current failsafe state
#[derive(Debug, Clone, Copy)]
pub struct FailsafeState {
    /// Battery percentage
    pub battery_percent: u8,
    /// GPS satellite count
    pub gps_satellites: u8,
    /// GPS fix type (0=none, 2=2D, 3=3D)
    pub gps_fix: u8,
    /// Current altitude (m AGL)
    pub altitude: f32,
    /// Time since RC last received (ms)
    pub rc_lost_duration_ms: u32,
    /// Time since data link last received (ms)
    pub data_link_lost_duration_ms: u32,
    /// Flight time (seconds)
    pub flight_time_secs: u32,
    /// Is inside geofence
    pub inside_geofence: bool,
    /// Motor health flags (bit per motor)
    pub motor_health: u8,
    /// Sensor health flags
    pub sensor_health: u8,
    /// Collision warning active
    pub collision_warning: bool,
    /// Is on ground
    pub on_ground: bool,
    /// Distance to home (m)
    pub distance_to_home: f32,
}

impl Default for FailsafeState {
    fn default() -> Self {
        Self {
            battery_percent: 100,
            gps_satellites: 12,
            gps_fix: 3,
            altitude: 0.0,
            rc_lost_duration_ms: 0,
            data_link_lost_duration_ms: 0,
            flight_time_secs: 0,
            inside_geofence: true,
            motor_health: 0xFF, // All healthy
            sensor_health: 0xFF,
            collision_warning: false,
            on_ground: true,
            distance_to_home: 0.0,
        }
    }
}

/// Failsafe evaluation result
#[derive(Debug, Clone)]
pub struct FailsafeResult {
    /// Recommended action
    pub action: FailsafeAction,
    /// Trigger that caused failsafe
    pub trigger: FailsafeTrigger,
    /// Priority (higher = more urgent)
    pub priority: u8,
    /// Additional info
    pub message: &'static str,
}

impl FailsafeResult {
    fn new(action: FailsafeAction, trigger: FailsafeTrigger, priority: u8, message: &'static str) -> Self {
        Self {
            action,
            trigger,
            priority,
            message,
        }
    }

    fn none() -> Self {
        Self {
            action: FailsafeAction::None,
            trigger: FailsafeTrigger::None,
            priority: 0,
            message: "Normal operation",
        }
    }
}

/// Failsafe manager
pub struct FailsafeManager {
    /// Configuration
    config: FailsafeConfig,
    /// Current state
    state: FailsafeState,
    /// Active failsafe result
    active_failsafe: FailsafeResult,
    /// Is failsafe locked (requires manual clear)
    locked: bool,
    /// Failsafe triggered count
    trigger_count: u32,
}

impl FailsafeManager {
    /// Create new failsafe manager
    pub fn new(config: FailsafeConfig) -> Self {
        Self {
            config,
            state: FailsafeState::default(),
            active_failsafe: FailsafeResult::none(),
            locked: false,
            trigger_count: 0,
        }
    }

    /// Update configuration
    pub fn set_config(&mut self, config: FailsafeConfig) {
        self.config = config;
    }

    /// Get current configuration
    pub fn config(&self) -> &FailsafeConfig {
        &self.config
    }

    /// Update failsafe state
    pub fn update_state(&mut self, state: FailsafeState) {
        self.state = state;
    }

    /// Manually trigger failsafe
    pub fn manual_trigger(&mut self, action: FailsafeAction) {
        self.active_failsafe = FailsafeResult::new(
            action,
            FailsafeTrigger::ManualTrigger,
            255, // Highest priority
            "Manual failsafe triggered",
        );
        self.locked = true;
        self.trigger_count += 1;
    }

    /// Clear failsafe (if conditions allow)
    pub fn clear(&mut self) -> bool {
        if self.locked {
            return false; // Require explicit unlock
        }

        let result = self.evaluate_internal();
        if result.action == FailsafeAction::None {
            self.active_failsafe = FailsafeResult::none();
            true
        } else {
            false
        }
    }

    /// Unlock failsafe (allows clearing)
    pub fn unlock(&mut self) {
        self.locked = false;
    }

    /// Check if failsafe is active
    pub fn is_active(&self) -> bool {
        self.active_failsafe.action != FailsafeAction::None
    }

    /// Get active failsafe result
    pub fn active(&self) -> &FailsafeResult {
        &self.active_failsafe
    }

    /// Get trigger count
    pub fn trigger_count(&self) -> u32 {
        self.trigger_count
    }

    /// Evaluate failsafe conditions and return recommended action
    pub fn evaluate(&mut self) -> FailsafeResult {
        // If locked, return current failsafe
        if self.locked {
            return self.active_failsafe.clone();
        }

        let result = self.evaluate_internal();

        // Update active failsafe if higher priority
        if result.priority > self.active_failsafe.priority {
            if result.action != FailsafeAction::None {
                self.trigger_count += 1;
            }
            self.active_failsafe = result.clone();
        }

        result
    }

    /// Internal evaluation logic
    fn evaluate_internal(&self) -> FailsafeResult {
        let mut highest = FailsafeResult::none();

        // Check each condition in priority order

        // 1. Motor failure (highest priority if in air)
        if self.state.motor_health != 0xFF && !self.state.on_ground {
            let result = FailsafeResult::new(
                self.config.motor_failure_action,
                FailsafeTrigger::MotorFailure,
                100,
                "Motor failure detected",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 2. Collision imminent
        if self.config.collision_avoidance_enabled && self.state.collision_warning {
            let result = FailsafeResult::new(
                self.config.collision_action,
                FailsafeTrigger::CollisionImminent,
                95,
                "Collision warning",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 3. Critical battery
        if self.state.battery_percent <= self.config.battery_critical_percent {
            let result = FailsafeResult::new(
                FailsafeAction::Land, // Always land on critical battery
                FailsafeTrigger::BatteryCritical,
                90,
                "Critical battery level",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 4. Sensor failure
        if self.state.sensor_health != 0xFF {
            let result = FailsafeResult::new(
                self.config.sensor_failure_action,
                FailsafeTrigger::SensorFailure,
                85,
                "Sensor failure detected",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 5. GPS lost
        if self.state.gps_fix < 3 || self.state.gps_satellites < self.config.min_gps_satellites {
            let result = FailsafeResult::new(
                self.config.gps_lost_action,
                FailsafeTrigger::GpsLost,
                80,
                "GPS fix lost",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 6. Geofence breach
        if self.config.geofence_enabled && !self.state.inside_geofence {
            let result = FailsafeResult::new(
                self.config.geofence_action,
                FailsafeTrigger::GeofenceBreach,
                75,
                "Geofence boundary breached",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 7. Altitude limit
        if self.state.altitude > self.config.max_altitude {
            let result = FailsafeResult::new(
                self.config.altitude_action,
                FailsafeTrigger::AltitudeLimit,
                70,
                "Maximum altitude exceeded",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 8. Data link lost
        if self.state.data_link_lost_duration_ms > self.config.data_link_timeout_ms {
            let result = FailsafeResult::new(
                self.config.data_link_action,
                FailsafeTrigger::DataLinkLost,
                65,
                "Data link lost",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 9. RC lost
        if self.state.rc_lost_duration_ms > self.config.rc_lost_timeout_ms {
            let result = FailsafeResult::new(
                self.config.rc_lost_action,
                FailsafeTrigger::RcLost,
                60,
                "RC signal lost",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 10. Low battery warning
        if self.state.battery_percent <= self.config.battery_warning_percent
            && self.state.battery_percent > self.config.battery_critical_percent
        {
            let action = if self.state.distance_to_home > 100.0 {
                // Far from home - start returning
                self.config.battery_action
            } else {
                // Close to home - just warn
                FailsafeAction::Warn
            };
            let result = FailsafeResult::new(
                action,
                FailsafeTrigger::BatteryWarning,
                50,
                "Low battery warning",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        // 11. Flight time limit
        if self.state.flight_time_secs > self.config.max_flight_time_secs {
            let result = FailsafeResult::new(
                self.config.flight_time_action,
                FailsafeTrigger::FlightTimeLimit,
                40,
                "Maximum flight time exceeded",
            );
            if result.priority > highest.priority {
                highest = result;
            }
        }

        highest
    }

    /// Get recommended descent rate based on situation
    pub fn get_descent_rate(&self) -> f32 {
        match self.active_failsafe.trigger {
            FailsafeTrigger::BatteryCritical => self.config.failsafe_descend_rate * 2.0,
            FailsafeTrigger::MotorFailure => self.config.failsafe_descend_rate * 3.0,
            _ => self.config.failsafe_descend_rate,
        }
    }

    /// Check if RTL is safe (enough altitude and battery)
    pub fn is_rtl_safe(&self) -> bool {
        self.state.altitude >= self.config.min_rtl_altitude
            && self.state.battery_percent > self.config.battery_critical_percent
            && self.state.gps_fix >= 3
    }

    /// Get safe landing position (current position if no better option)
    pub fn get_safe_landing_position(&self) -> Option<[f32; 3]> {
        // In a real implementation, this would consider terrain, obstacles, etc.
        // For now, return None to indicate "land at current position"
        None
    }
}

impl Default for FailsafeManager {
    fn default() -> Self {
        Self::new(FailsafeConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = FailsafeConfig::default();
        assert_eq!(config.battery_warning_percent, 30);
        assert_eq!(config.battery_critical_percent, 15);
    }

    #[test]
    fn test_no_failsafe_normal() {
        let mut fsm = FailsafeManager::default();
        let result = fsm.evaluate();
        assert_eq!(result.action, FailsafeAction::None);
        assert!(!fsm.is_active());
    }

    #[test]
    fn test_low_battery_warning() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();
        state.battery_percent = 25; // Below warning, above critical
        state.distance_to_home = 200.0; // Far from home
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryWarning);
        assert_eq!(result.action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_critical_battery() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();
        state.battery_percent = 10; // Critical
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
        assert_eq!(result.action, FailsafeAction::Land);
    }

    #[test]
    fn test_gps_lost() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();
        state.gps_satellites = 3; // Below minimum
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GpsLost);
    }

    #[test]
    fn test_geofence_breach() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();
        state.inside_geofence = false;
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GeofenceBreach);
    }

    #[test]
    fn test_manual_trigger() {
        let mut fsm = FailsafeManager::default();
        fsm.manual_trigger(FailsafeAction::Land);

        assert!(fsm.is_active());
        assert_eq!(fsm.active().action, FailsafeAction::Land);
        assert_eq!(fsm.active().trigger, FailsafeTrigger::ManualTrigger);

        // Cannot clear while locked
        assert!(!fsm.clear());

        // Unlock and clear
        fsm.unlock();
        assert!(fsm.clear());
    }

    #[test]
    fn test_priority() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();

        // Multiple conditions
        state.battery_percent = 10; // Critical battery (priority 90)
        state.gps_satellites = 3; // GPS lost (priority 80)
        fsm.update_state(state);

        let result = fsm.evaluate();
        // Critical battery should win
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
    }

    #[test]
    fn test_rtl_safety_check() {
        let mut fsm = FailsafeManager::default();
        let mut state = FailsafeState::default();
        state.altitude = 50.0;
        state.battery_percent = 50;
        state.gps_fix = 3;
        fsm.update_state(state);

        assert!(fsm.is_rtl_safe());

        // Low altitude
        state.altitude = 5.0;
        fsm.update_state(state);
        assert!(!fsm.is_rtl_safe());
    }
}
