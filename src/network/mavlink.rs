//! MAVLink Flight Controller Interface
//!
//! This module provides a high-level interface for controlling drones via MAVLink.
//! It supports PX4 and ArduPilot flight controllers through the MAVLink protocol.
//!
//! # Features
//! - Arm/Disarm control
//! - Takeoff and landing
//! - Position and velocity control
//! - Return to Launch (RTL)
//! - Flight mode switching
//! - Telemetry monitoring
//!
//! # Example
//! ```rust,ignore
//! use drone_swarm_system::mavlink_controller::*;
//!
//! let mut controller = FlightController::connect("udpin:0.0.0.0:14540")?;
//! controller.arm()?;
//! controller.takeoff(10.0)?;
//! controller.goto_position(10.0, 20.0, -10.0)?;
//! controller.land()?;
//! ```

#[cfg(feature = "simulation")]
use mavlink::common::{
    MavCmd, MavMessage, MavModeFlag, MavState, PositionTargetTypemask, COMMAND_LONG_DATA,
    SET_POSITION_TARGET_LOCAL_NED_DATA,
};
#[cfg(feature = "simulation")]
use mavlink::MavHeader;

#[cfg(feature = "simulation")]
use std::sync::Arc;
#[cfg(feature = "simulation")]
use std::time::Instant;

/// Flight modes supported by PX4/ArduPilot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    /// Manual control
    Manual,
    /// Altitude hold
    Altitude,
    /// Position hold
    Position,
    /// Offboard control (external commands)
    Offboard,
    /// Auto mission
    Mission,
    /// Return to launch
    ReturnToLaunch,
    /// Land at current position
    Land,
    /// Takeoff
    Takeoff,
    /// Loiter at current position
    Loiter,
    /// Stabilize mode
    Stabilize,
}

impl FlightMode {
    /// Convert to PX4 custom mode value
    #[cfg(feature = "simulation")]
    pub fn to_px4_mode(&self) -> u32 {
        match self {
            FlightMode::Manual => 1,         // MANUAL
            FlightMode::Altitude => 2,       // ALTITUDE
            FlightMode::Position => 3,       // POSITION
            FlightMode::Offboard => 6,       // OFFBOARD
            FlightMode::Mission => 4,        // MISSION
            FlightMode::ReturnToLaunch => 5, // RTL
            FlightMode::Land => 8,           // LAND
            FlightMode::Takeoff => 10,       // TAKEOFF
            FlightMode::Loiter => 3,         // Same as POSITION for PX4
            FlightMode::Stabilize => 7,      // STABILIZED
        }
    }
}

/// Drone telemetry state
#[derive(Debug, Clone, Default)]
pub struct TelemetryState {
    /// Position in NED frame [x, y, z] meters
    pub position: [f32; 3],
    /// Velocity in NED frame [vx, vy, vz] m/s
    pub velocity: [f32; 3],
    /// Attitude [roll, pitch, yaw] radians
    pub attitude: [f32; 3],
    /// Angular rates [rollspeed, pitchspeed, yawspeed] rad/s
    pub angular_rates: [f32; 3],
    /// GPS coordinates [lat, lon, alt] (degrees, degrees, meters)
    pub gps: [f64; 3],
    /// GPS fix type (0=no fix, 3=3D fix)
    pub gps_fix: u8,
    /// Number of satellites
    pub satellites: u8,
    /// Battery voltage (V)
    pub battery_voltage: f32,
    /// Battery percentage (0-100)
    pub battery_percent: i8,
    /// Armed state
    pub armed: bool,
    /// Current flight mode
    pub mode: u8,
    /// System status
    pub system_status: u8,
    /// Last heartbeat time
    pub last_heartbeat_ms: u64,
    /// Last position update time
    pub last_position_ms: u64,
}

/// Command result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandResult {
    /// Command accepted
    Accepted,
    /// Command temporarily rejected
    Denied,
    /// Command not supported
    Unsupported,
    /// Command failed
    Failed,
    /// Command in progress
    InProgress,
    /// Command timed out
    Timeout,
}

/// Flight controller error
#[derive(Debug, Clone)]
pub enum ControllerError {
    /// Connection failed
    ConnectionFailed(String),
    /// Command failed
    CommandFailed(String),
    /// Timeout
    Timeout,
    /// Not armed
    NotArmed,
    /// Already armed
    AlreadyArmed,
    /// Invalid parameter
    InvalidParameter,
    /// Communication error
    CommunicationError(String),
}

#[cfg(feature = "simulation")]
impl std::fmt::Display for ControllerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ControllerError::ConnectionFailed(s) => write!(f, "Connection failed: {}", s),
            ControllerError::CommandFailed(s) => write!(f, "Command failed: {}", s),
            ControllerError::Timeout => write!(f, "Operation timed out"),
            ControllerError::NotArmed => write!(f, "Drone is not armed"),
            ControllerError::AlreadyArmed => write!(f, "Drone is already armed"),
            ControllerError::InvalidParameter => write!(f, "Invalid parameter"),
            ControllerError::CommunicationError(s) => write!(f, "Communication error: {}", s),
        }
    }
}

#[cfg(feature = "simulation")]
impl std::error::Error for ControllerError {}

/// Result type for controller operations
#[cfg(feature = "simulation")]
pub type ControllerResult<T> = std::result::Result<T, ControllerError>;

/// MAVLink Flight Controller
#[cfg(feature = "simulation")]
pub struct FlightController {
    /// MAVLink connection
    connection: Arc<dyn mavlink::MavConnection<MavMessage> + Send + Sync>,
    /// Target system ID
    target_system: u8,
    /// Target component ID
    target_component: u8,
    /// Our system ID
    system_id: u8,
    /// Our component ID
    component_id: u8,
    /// Current telemetry state
    state: TelemetryState,
    /// Message sequence number
    sequence: u8,
    /// Command timeout (ms) - reserved for future async command acknowledgment
    #[allow(dead_code)]
    command_timeout_ms: u64,
    /// Start time for timing
    start_time: Instant,
}

#[cfg(feature = "simulation")]
impl FlightController {
    /// Connect to a MAVLink endpoint
    pub fn connect(address: &str) -> ControllerResult<Self> {
        let connection = mavlink::connect::<MavMessage>(address)
            .map_err(|e| ControllerError::ConnectionFailed(e.to_string()))?;

        Ok(Self {
            connection: Arc::from(connection),
            target_system: 1,
            target_component: 1,
            system_id: 255,
            component_id: 0,
            state: TelemetryState::default(),
            sequence: 0,
            command_timeout_ms: 5000,
            start_time: Instant::now(),
        })
    }

    /// Get current telemetry state
    pub fn state(&self) -> &TelemetryState {
        &self.state
    }

    /// Check if drone is armed
    pub fn is_armed(&self) -> bool {
        self.state.armed
    }

    /// Get current position
    pub fn position(&self) -> [f32; 3] {
        self.state.position
    }

    /// Update telemetry by processing received messages
    pub fn update(&mut self) -> ControllerResult<()> {
        let current_time = self.start_time.elapsed().as_millis() as u64;

        // Process available messages (non-blocking would be better)
        // For now, try to receive with a short timeout
        match self.connection.recv() {
            Ok((_header, msg)) => {
                self.process_message(msg, current_time);
            }
            Err(_) => {
                // No message available or error, continue
            }
        }

        Ok(())
    }

    /// Process a received MAVLink message
    fn process_message(&mut self, msg: MavMessage, current_time: u64) {
        match msg {
            MavMessage::HEARTBEAT(hb) => {
                self.state.armed =
                    (hb.base_mode.bits() & MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED.bits()) != 0;
                self.state.mode = hb.custom_mode as u8;
                self.state.system_status = hb.system_status as u8;
                self.state.last_heartbeat_ms = current_time;
            }
            MavMessage::LOCAL_POSITION_NED(pos) => {
                self.state.position = [pos.x, pos.y, pos.z];
                self.state.velocity = [pos.vx, pos.vy, pos.vz];
                self.state.last_position_ms = current_time;
            }
            MavMessage::ATTITUDE(att) => {
                self.state.attitude = [att.roll, att.pitch, att.yaw];
                self.state.angular_rates = [att.rollspeed, att.pitchspeed, att.yawspeed];
            }
            MavMessage::GPS_RAW_INT(gps) => {
                self.state.gps = [
                    gps.lat as f64 / 1e7,
                    gps.lon as f64 / 1e7,
                    gps.alt as f64 / 1000.0,
                ];
                self.state.gps_fix = gps.fix_type as u8;
                self.state.satellites = gps.satellites_visible;
            }
            MavMessage::BATTERY_STATUS(bat) => {
                self.state.battery_percent = bat.battery_remaining;
                if !bat.voltages.is_empty() && bat.voltages[0] != u16::MAX {
                    self.state.battery_voltage = bat.voltages[0] as f32 / 1000.0;
                }
            }
            MavMessage::SYS_STATUS(status) => {
                self.state.battery_voltage = status.voltage_battery as f32 / 1000.0;
                self.state.battery_percent = status.battery_remaining;
            }
            _ => {}
        }
    }

    /// Send a MAVLink command
    fn send_command(&mut self, cmd: MavCmd, params: [f32; 7]) -> ControllerResult<()> {
        let header = self.make_header();

        let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: self.target_system,
            target_component: self.target_component,
            command: cmd,
            confirmation: 0,
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
        });

        self.connection
            .send(&header, &msg)
            .map_err(|e| ControllerError::CommunicationError(e.to_string()))?;

        Ok(())
    }

    /// Make a MAVLink header
    fn make_header(&mut self) -> MavHeader {
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);

        MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: seq,
        }
    }

    /// Arm the drone
    pub fn arm(&mut self) -> ControllerResult<()> {
        if self.state.armed {
            return Err(ControllerError::AlreadyArmed);
        }

        self.send_command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// Disarm the drone
    pub fn disarm(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// Force disarm (even in flight - DANGEROUS)
    pub fn force_disarm(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0], // Magic number for force disarm
        )
    }

    /// Takeoff to specified altitude (meters)
    pub fn takeoff(&mut self, altitude: f32) -> ControllerResult<()> {
        if !self.state.armed {
            return Err(ControllerError::NotArmed);
        }

        // Send takeoff command
        self.send_command(
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                0.0,      // Minimum pitch
                0.0,      // Empty
                0.0,      // Empty
                f32::NAN, // Yaw (NAN = current)
                f32::NAN, // Latitude (NAN = current)
                f32::NAN, // Longitude (NAN = current)
                altitude, // Altitude
            ],
        )
    }

    /// Land at current position
    pub fn land(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_NAV_LAND,
            [
                0.0,      // Abort altitude
                0.0,      // Land mode
                0.0,      // Empty
                f32::NAN, // Yaw (NAN = current)
                f32::NAN, // Latitude (NAN = current)
                f32::NAN, // Longitude (NAN = current)
                0.0,      // Altitude (ignored for land)
            ],
        )
    }

    /// Return to launch position
    pub fn return_to_launch(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// Set flight mode
    pub fn set_mode(&mut self, mode: FlightMode) -> ControllerResult<()> {
        let custom_mode = mode.to_px4_mode();

        // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        self.send_command(
            MavCmd::MAV_CMD_DO_SET_MODE,
            [
                1.0, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                custom_mode as f32,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
    }

    /// Go to position (NED frame: x=North, y=East, z=Down)
    pub fn goto_position(&mut self, x: f32, y: f32, z: f32) -> ControllerResult<()> {
        self.goto_position_with_yaw(x, y, z, f32::NAN)
    }

    /// Go to position with specific yaw
    pub fn goto_position_with_yaw(
        &mut self,
        x: f32,
        y: f32,
        z: f32,
        yaw: f32,
    ) -> ControllerResult<()> {
        let header = self.make_header();

        let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask: PositionTargetTypemask::empty(),
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw,
            yaw_rate: 0.0,
        });

        self.connection
            .send(&header, &msg)
            .map_err(|e| ControllerError::CommunicationError(e.to_string()))?;

        Ok(())
    }

    /// Set velocity (NED frame: vx=North, vy=East, vz=Down)
    pub fn set_velocity(&mut self, vx: f32, vy: f32, vz: f32) -> ControllerResult<()> {
        self.set_velocity_with_yaw_rate(vx, vy, vz, 0.0)
    }

    /// Set velocity with yaw rate
    pub fn set_velocity_with_yaw_rate(
        &mut self,
        vx: f32,
        vy: f32,
        vz: f32,
        yaw_rate: f32,
    ) -> ControllerResult<()> {
        let header = self.make_header();

        // Use velocity only (ignore position)
        let type_mask = PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE;

        let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx,
            vy,
            vz,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate,
        });

        self.connection
            .send(&header, &msg)
            .map_err(|e| ControllerError::CommunicationError(e.to_string()))?;

        Ok(())
    }

    /// Hover at current position
    pub fn hover(&mut self) -> ControllerResult<()> {
        let pos = self.state.position;
        self.goto_position(pos[0], pos[1], pos[2])
    }

    /// Emergency stop (kills motors immediately)
    pub fn emergency_stop(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// Reboot the flight controller
    pub fn reboot(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// Set home position to current location
    pub fn set_home_current(&mut self) -> ControllerResult<()> {
        self.send_command(
            MavCmd::MAV_CMD_DO_SET_HOME,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], // 1.0 = use current position
        )
    }

    /// Set a specific parameter
    pub fn set_param(&mut self, param_id: &str, value: f32) -> ControllerResult<()> {
        let header = self.make_header();

        // Convert param_id to fixed array
        let mut id_bytes = [0u8; 16];
        for (i, byte) in param_id.bytes().take(16).enumerate() {
            id_bytes[i] = byte;
        }

        let msg = MavMessage::PARAM_SET(mavlink::common::PARAM_SET_DATA {
            target_system: self.target_system,
            target_component: self.target_component,
            param_id: id_bytes,
            param_value: value,
            param_type: mavlink::common::MavParamType::MAV_PARAM_TYPE_REAL32,
        });

        self.connection
            .send(&header, &msg)
            .map_err(|e| ControllerError::CommunicationError(e.to_string()))?;

        Ok(())
    }

    /// Calculate distance to a point
    pub fn distance_to(&self, x: f32, y: f32, z: f32) -> f32 {
        let dx = self.state.position[0] - x;
        let dy = self.state.position[1] - y;
        let dz = self.state.position[2] - z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Calculate 2D distance to a point (ignoring altitude)
    pub fn distance_2d_to(&self, x: f32, y: f32) -> f32 {
        let dx = self.state.position[0] - x;
        let dy = self.state.position[1] - y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Get altitude (negative of z in NED)
    pub fn altitude(&self) -> f32 {
        -self.state.position[2]
    }

    /// Send heartbeat (for maintaining connection)
    pub fn send_heartbeat(&mut self) -> ControllerResult<()> {
        let header = self.make_header();

        let msg = MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: mavlink::common::MavType::MAV_TYPE_GCS,
            autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });

        self.connection
            .send(&header, &msg)
            .map_err(|e| ControllerError::CommunicationError(e.to_string()))?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "simulation")]
    fn test_flight_mode_to_px4() {
        assert_eq!(FlightMode::Manual.to_px4_mode(), 1);
        assert_eq!(FlightMode::Offboard.to_px4_mode(), 6);
        assert_eq!(FlightMode::ReturnToLaunch.to_px4_mode(), 5);
    }

    #[test]
    fn test_telemetry_state_default() {
        let state = TelemetryState::default();
        assert!(!state.armed);
        assert_eq!(state.position, [0.0, 0.0, 0.0]);
        assert_eq!(state.battery_percent, 0);
    }
}
