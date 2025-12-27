//! Telemetry Monitoring System
//!
//! Real-time telemetry aggregation, health monitoring, and alerting
//! for drone swarm operations.
//!
//! # Features
//! - Multi-drone telemetry aggregation
//! - Health status monitoring
//! - Alert/warning generation
//! - Statistics and metrics
//! - Trend analysis
//! - `no_std` compatible
//!
//! # Example
//! ```ignore
//! use drone_swarm_system::telemetry::{TelemetryMonitor, DroneStatus};
//!
//! let mut monitor = TelemetryMonitor::new();
//! monitor.update_drone(0, DroneStatus::default());
//! let alerts = monitor.check_alerts();
//! ```

use heapless::Vec;

/// Maximum drones to monitor
pub const MAX_MONITORED_DRONES: usize = 32;

/// Maximum alerts in queue
pub const MAX_ALERTS: usize = 64;

/// Maximum telemetry history samples per drone
pub const HISTORY_SIZE: usize = 100;

/// Alert severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlertSeverity {
    /// Informational
    Info = 0,
    /// Warning - attention needed
    Warning = 1,
    /// Critical - immediate action required
    Critical = 2,
    /// Emergency - safety risk
    Emergency = 3,
}

/// Alert types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertType {
    /// Low battery
    LowBattery,
    /// Critical battery
    CriticalBattery,
    /// Communication lost
    CommLost,
    /// GPS fix lost
    GpsLost,
    /// Geofence breach
    GeofenceBreach,
    /// Motor failure
    MotorFailure,
    /// Sensor failure
    SensorFailure,
    /// Temperature warning
    OverTemperature,
    /// Altitude limit
    AltitudeLimit,
    /// Speed limit exceeded
    SpeedLimit,
    /// Formation deviation
    FormationDeviation,
    /// Collision warning
    CollisionWarning,
    /// System error
    SystemError,
    /// Custom alert
    Custom,
}

/// Single alert
#[derive(Debug, Clone)]
pub struct Alert {
    /// Drone ID (255 = system-wide)
    pub drone_id: u8,
    /// Alert type
    pub alert_type: AlertType,
    /// Severity
    pub severity: AlertSeverity,
    /// Timestamp (ms since boot)
    pub timestamp_ms: u64,
    /// Alert message/details
    pub message: [u8; 32],
    /// Is alert acknowledged
    pub acknowledged: bool,
}

impl Alert {
    /// Create new alert
    pub fn new(drone_id: u8, alert_type: AlertType, severity: AlertSeverity, timestamp_ms: u64) -> Self {
        Self {
            drone_id,
            alert_type,
            severity,
            timestamp_ms,
            message: [0; 32],
            acknowledged: false,
        }
    }

    /// Create alert with message
    pub fn with_message(mut self, msg: &str) -> Self {
        let bytes = msg.as_bytes();
        let len = bytes.len().min(31);
        self.message[..len].copy_from_slice(&bytes[..len]);
        self
    }
}

/// Drone health status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    /// All systems nominal
    Healthy,
    /// Some warnings but operational
    Degraded,
    /// Critical issues
    Critical,
    /// Drone unresponsive
    Offline,
    /// Unknown status
    Unknown,
}

/// Individual drone telemetry
#[derive(Debug, Clone, Copy)]
pub struct DroneStatus {
    /// Drone ID
    pub drone_id: u8,
    /// Position [x, y, z] NED
    pub position: [f32; 3],
    /// Velocity [vx, vy, vz]
    pub velocity: [f32; 3],
    /// Attitude [roll, pitch, yaw] radians
    pub attitude: [f32; 3],
    /// Battery percentage (0-100)
    pub battery_percent: u8,
    /// Battery voltage
    pub battery_voltage: f32,
    /// GPS fix type (0=none, 2=2D, 3=3D)
    pub gps_fix: u8,
    /// Number of GPS satellites
    pub gps_satellites: u8,
    /// HDOP (horizontal dilution of precision)
    pub gps_hdop: f32,
    /// RC link quality (0-100)
    pub rc_signal: u8,
    /// Telemetry link RSSI (dBm)
    pub rssi: i8,
    /// Is armed
    pub armed: bool,
    /// Flight mode string
    pub mode: u8,
    /// Last update timestamp (ms)
    pub last_update_ms: u64,
    /// Health status
    pub health: HealthStatus,
    /// CPU temperature (Celsius)
    pub cpu_temp: f32,
    /// Motor temperatures [4]
    pub motor_temps: [f32; 4],
    /// Vibration levels [x, y, z]
    pub vibration: [f32; 3],
}

impl Default for DroneStatus {
    fn default() -> Self {
        Self {
            drone_id: 0,
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            attitude: [0.0, 0.0, 0.0],
            battery_percent: 100,
            battery_voltage: 16.8,
            gps_fix: 0,
            gps_satellites: 0,
            gps_hdop: 99.9,
            rc_signal: 0,
            rssi: -100,
            armed: false,
            mode: 0,
            last_update_ms: 0,
            health: HealthStatus::Unknown,
            cpu_temp: 25.0,
            motor_temps: [25.0; 4],
            vibration: [0.0; 3],
        }
    }
}

impl DroneStatus {
    /// Calculate ground speed
    pub fn ground_speed(&self) -> f32 {
        libm::sqrtf(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
    }

    /// Get altitude (positive, meters AGL)
    pub fn altitude(&self) -> f32 {
        -self.position[2]
    }

    /// Calculate 3D speed
    pub fn speed_3d(&self) -> f32 {
        libm::sqrtf(
            self.velocity[0] * self.velocity[0]
                + self.velocity[1] * self.velocity[1]
                + self.velocity[2] * self.velocity[2],
        )
    }
}

/// Alert thresholds configuration
#[derive(Debug, Clone)]
pub struct AlertThresholds {
    /// Low battery warning (percent)
    pub battery_warning: u8,
    /// Critical battery (percent)
    pub battery_critical: u8,
    /// Maximum speed (m/s)
    pub max_speed: f32,
    /// Maximum altitude (m)
    pub max_altitude: f32,
    /// Minimum GPS satellites for warning
    pub min_gps_sats: u8,
    /// Maximum time without update (ms)
    pub comm_timeout_ms: u64,
    /// Maximum temperature (Celsius)
    pub max_temperature: f32,
    /// Maximum vibration level
    pub max_vibration: f32,
    /// Formation deviation threshold (m)
    pub formation_deviation: f32,
}

impl Default for AlertThresholds {
    fn default() -> Self {
        Self {
            battery_warning: 30,
            battery_critical: 15,
            max_speed: 20.0,
            max_altitude: 120.0,
            min_gps_sats: 6,
            comm_timeout_ms: 5000,
            max_temperature: 80.0,
            max_vibration: 30.0,
            formation_deviation: 10.0,
        }
    }
}

/// Swarm statistics
#[derive(Debug, Clone, Copy, Default)]
pub struct SwarmStats {
    /// Number of active drones
    pub active_drones: usize,
    /// Number of healthy drones
    pub healthy_drones: usize,
    /// Number of drones with warnings
    pub warning_drones: usize,
    /// Number of critical/offline drones
    pub critical_drones: usize,
    /// Average battery level
    pub avg_battery: f32,
    /// Minimum battery level
    pub min_battery: u8,
    /// Average ground speed
    pub avg_speed: f32,
    /// Maximum ground speed
    pub max_speed: f32,
    /// Swarm center position
    pub center: [f32; 3],
    /// Swarm spread (max distance from center)
    pub spread: f32,
    /// Active alerts count
    pub active_alerts: usize,
}

/// Telemetry monitor
#[derive(Clone)]
pub struct TelemetryMonitor {
    /// Drone statuses
    drones: Vec<DroneStatus, MAX_MONITORED_DRONES>,
    /// Active alerts
    alerts: Vec<Alert, MAX_ALERTS>,
    /// Alert thresholds
    thresholds: AlertThresholds,
    /// Current time (ms)
    current_time_ms: u64,
    /// Alert callback enabled
    alerts_enabled: bool,
}

impl TelemetryMonitor {
    /// Create new telemetry monitor
    pub fn new() -> Self {
        Self {
            drones: Vec::new(),
            alerts: Vec::new(),
            thresholds: AlertThresholds::default(),
            current_time_ms: 0,
            alerts_enabled: true,
        }
    }

    /// Set alert thresholds
    pub fn set_thresholds(&mut self, thresholds: AlertThresholds) {
        self.thresholds = thresholds;
    }

    /// Enable/disable alert generation
    pub fn set_alerts_enabled(&mut self, enabled: bool) {
        self.alerts_enabled = enabled;
    }

    /// Update current time
    pub fn set_time(&mut self, time_ms: u64) {
        self.current_time_ms = time_ms;
    }

    /// Update or add drone status
    pub fn update_drone(&mut self, drone_id: u8, mut status: DroneStatus) {
        status.drone_id = drone_id;
        status.last_update_ms = self.current_time_ms;

        // Update health status based on telemetry
        status.health = self.evaluate_health(&status);

        // Find existing or add new
        if let Some(drone) = self.drones.iter_mut().find(|d| d.drone_id == drone_id) {
            *drone = status;
        } else {
            let _ = self.drones.push(status);
        }
    }

    /// Remove drone from monitoring
    pub fn remove_drone(&mut self, drone_id: u8) {
        if let Some(idx) = self.drones.iter().position(|d| d.drone_id == drone_id) {
            self.drones.swap_remove(idx);
        }
    }

    /// Get drone status
    pub fn get_drone(&self, drone_id: u8) -> Option<&DroneStatus> {
        self.drones.iter().find(|d| d.drone_id == drone_id)
    }

    /// Get all drone statuses
    pub fn get_all_drones(&self) -> &[DroneStatus] {
        &self.drones
    }

    /// Get number of monitored drones
    pub fn drone_count(&self) -> usize {
        self.drones.len()
    }

    /// Evaluate drone health status
    fn evaluate_health(&self, status: &DroneStatus) -> HealthStatus {
        // Check for critical conditions
        if status.battery_percent <= self.thresholds.battery_critical {
            return HealthStatus::Critical;
        }

        if self.current_time_ms > status.last_update_ms
            && self.current_time_ms - status.last_update_ms > self.thresholds.comm_timeout_ms
        {
            return HealthStatus::Offline;
        }

        // Check for warnings
        let mut has_warning = false;

        if status.battery_percent <= self.thresholds.battery_warning {
            has_warning = true;
        }

        if status.gps_satellites < self.thresholds.min_gps_sats {
            has_warning = true;
        }

        if status.cpu_temp > self.thresholds.max_temperature {
            has_warning = true;
        }

        let vib_mag = libm::sqrtf(
            status.vibration[0] * status.vibration[0]
                + status.vibration[1] * status.vibration[1]
                + status.vibration[2] * status.vibration[2],
        );
        if vib_mag > self.thresholds.max_vibration {
            has_warning = true;
        }

        if has_warning {
            HealthStatus::Degraded
        } else {
            HealthStatus::Healthy
        }
    }

    /// Check for alerts and generate new ones
    pub fn check_alerts(&mut self) -> Vec<Alert, MAX_ALERTS> {
        let mut new_alerts: Vec<Alert, MAX_ALERTS> = Vec::new();

        if !self.alerts_enabled {
            return new_alerts;
        }

        for drone in &self.drones {
            // Battery checks
            if drone.battery_percent <= self.thresholds.battery_critical {
                let _ = new_alerts.push(
                    Alert::new(
                        drone.drone_id,
                        AlertType::CriticalBattery,
                        AlertSeverity::Emergency,
                        self.current_time_ms,
                    )
                    .with_message("Critical battery"),
                );
            } else if drone.battery_percent <= self.thresholds.battery_warning {
                let _ = new_alerts.push(
                    Alert::new(
                        drone.drone_id,
                        AlertType::LowBattery,
                        AlertSeverity::Warning,
                        self.current_time_ms,
                    )
                    .with_message("Low battery"),
                );
            }

            // GPS check
            if drone.gps_fix < 3 || drone.gps_satellites < self.thresholds.min_gps_sats {
                let _ = new_alerts.push(Alert::new(
                    drone.drone_id,
                    AlertType::GpsLost,
                    AlertSeverity::Warning,
                    self.current_time_ms,
                ));
            }

            // Speed check
            if drone.ground_speed() > self.thresholds.max_speed {
                let _ = new_alerts.push(Alert::new(
                    drone.drone_id,
                    AlertType::SpeedLimit,
                    AlertSeverity::Warning,
                    self.current_time_ms,
                ));
            }

            // Altitude check
            if drone.altitude() > self.thresholds.max_altitude {
                let _ = new_alerts.push(Alert::new(
                    drone.drone_id,
                    AlertType::AltitudeLimit,
                    AlertSeverity::Warning,
                    self.current_time_ms,
                ));
            }

            // Temperature check
            if drone.cpu_temp > self.thresholds.max_temperature {
                let _ = new_alerts.push(Alert::new(
                    drone.drone_id,
                    AlertType::OverTemperature,
                    AlertSeverity::Critical,
                    self.current_time_ms,
                ));
            }

            // Communication timeout
            if self.current_time_ms > drone.last_update_ms
                && self.current_time_ms - drone.last_update_ms > self.thresholds.comm_timeout_ms
            {
                let _ = new_alerts.push(Alert::new(
                    drone.drone_id,
                    AlertType::CommLost,
                    AlertSeverity::Critical,
                    self.current_time_ms,
                ));
            }
        }

        // Store alerts
        for alert in &new_alerts {
            if !self.has_similar_alert(alert) {
                let _ = self.alerts.push(alert.clone());
            }
        }

        new_alerts
    }

    /// Check if similar alert already exists (same drone, type, and recent)
    fn has_similar_alert(&self, alert: &Alert) -> bool {
        const ALERT_COOLDOWN_MS: u64 = 10000; // 10 seconds

        self.alerts.iter().any(|a| {
            a.drone_id == alert.drone_id
                && a.alert_type == alert.alert_type
                && !a.acknowledged
                && alert.timestamp_ms - a.timestamp_ms < ALERT_COOLDOWN_MS
        })
    }

    /// Get active (unacknowledged) alerts
    pub fn get_active_alerts(&self) -> impl Iterator<Item = &Alert> {
        self.alerts.iter().filter(|a| !a.acknowledged)
    }

    /// Get alerts by severity
    pub fn get_alerts_by_severity(&self, severity: AlertSeverity) -> impl Iterator<Item = &Alert> {
        self.alerts
            .iter()
            .filter(move |a| a.severity == severity && !a.acknowledged)
    }

    /// Acknowledge alert
    pub fn acknowledge_alert(&mut self, index: usize) {
        if let Some(alert) = self.alerts.get_mut(index) {
            alert.acknowledged = true;
        }
    }

    /// Acknowledge all alerts for drone
    pub fn acknowledge_drone_alerts(&mut self, drone_id: u8) {
        for alert in &mut self.alerts {
            if alert.drone_id == drone_id {
                alert.acknowledged = true;
            }
        }
    }

    /// Clear old acknowledged alerts
    pub fn clear_old_alerts(&mut self, max_age_ms: u64) {
        let current = self.current_time_ms;
        self.alerts.retain(|a| {
            !a.acknowledged || current - a.timestamp_ms < max_age_ms
        });
    }

    /// Get swarm statistics
    pub fn get_swarm_stats(&self) -> SwarmStats {
        let mut stats = SwarmStats::default();

        if self.drones.is_empty() {
            return stats;
        }

        let mut total_battery = 0u32;
        let mut total_speed = 0.0f32;
        let mut center = [0.0f32; 3];

        stats.min_battery = 100;

        for drone in &self.drones {
            stats.active_drones += 1;

            match drone.health {
                HealthStatus::Healthy => stats.healthy_drones += 1,
                HealthStatus::Degraded => stats.warning_drones += 1,
                HealthStatus::Critical | HealthStatus::Offline => stats.critical_drones += 1,
                HealthStatus::Unknown => {}
            }

            total_battery += drone.battery_percent as u32;
            stats.min_battery = stats.min_battery.min(drone.battery_percent);

            let speed = drone.ground_speed();
            total_speed += speed;
            stats.max_speed = stats.max_speed.max(speed);

            center[0] += drone.position[0];
            center[1] += drone.position[1];
            center[2] += drone.position[2];
        }

        let n = stats.active_drones as f32;
        stats.avg_battery = total_battery as f32 / n;
        stats.avg_speed = total_speed / n;
        stats.center = [center[0] / n, center[1] / n, center[2] / n];

        // Calculate spread
        for drone in &self.drones {
            let dist = libm::sqrtf(
                (drone.position[0] - stats.center[0]).powi(2)
                    + (drone.position[1] - stats.center[1]).powi(2)
                    + (drone.position[2] - stats.center[2]).powi(2),
            );
            stats.spread = stats.spread.max(dist);
        }

        stats.active_alerts = self.alerts.iter().filter(|a| !a.acknowledged).count();

        stats
    }

    /// Check overall swarm health
    pub fn swarm_health(&self) -> HealthStatus {
        let stats = self.get_swarm_stats();

        if stats.critical_drones > 0 {
            HealthStatus::Critical
        } else if stats.warning_drones > stats.active_drones / 2 {
            HealthStatus::Degraded
        } else if stats.healthy_drones == stats.active_drones {
            HealthStatus::Healthy
        } else if stats.active_drones == 0 {
            HealthStatus::Unknown
        } else {
            HealthStatus::Degraded
        }
    }
}

impl Default for TelemetryMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drone_status_default() {
        let status = DroneStatus::default();
        assert_eq!(status.battery_percent, 100);
        assert_eq!(status.health, HealthStatus::Unknown);
    }

    #[test]
    fn test_speed_calculations() {
        let status = DroneStatus {
            velocity: [3.0, 4.0, 0.0],
            ..Default::default()
        };

        assert!((status.ground_speed() - 5.0).abs() < 0.001);
        assert!((status.speed_3d() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_telemetry_monitor() {
        let mut monitor = TelemetryMonitor::new();
        monitor.set_time(1000);

        let status = DroneStatus::default();
        monitor.update_drone(1, status);

        assert_eq!(monitor.drone_count(), 1);
        assert!(monitor.get_drone(1).is_some());
        assert!(monitor.get_drone(99).is_none());
    }

    #[test]
    fn test_low_battery_alert() {
        let mut monitor = TelemetryMonitor::new();
        monitor.set_time(1000);

        let status = DroneStatus {
            battery_percent: 10, // Critical
            ..Default::default()
        };
        monitor.update_drone(1, status);

        let alerts = monitor.check_alerts();
        assert!(!alerts.is_empty());
        assert!(alerts.iter().any(|a| a.alert_type == AlertType::CriticalBattery));
    }

    #[test]
    fn test_swarm_stats() {
        let mut monitor = TelemetryMonitor::new();
        monitor.set_time(1000);

        for i in 0..3 {
            let status = DroneStatus {
                position: [i as f32 * 10.0, 0.0, -10.0],
                battery_percent: 80,
                ..Default::default()
            };
            monitor.update_drone(i, status);
        }

        let stats = monitor.get_swarm_stats();
        assert_eq!(stats.active_drones, 3);
        assert!((stats.avg_battery - 80.0).abs() < 0.1);
    }

    #[test]
    fn test_health_evaluation() {
        let mut monitor = TelemetryMonitor::new();
        monitor.set_time(1000);

        // Healthy drone (with proper GPS)
        let status = DroneStatus {
            gps_satellites: 10,
            gps_fix: 3,
            ..Default::default()
        };
        monitor.update_drone(1, status);
        assert_eq!(monitor.get_drone(1).unwrap().health, HealthStatus::Healthy);

        // Low battery drone
        let status2 = DroneStatus {
            battery_percent: 20,
            gps_satellites: 10,
            gps_fix: 3,
            ..Default::default()
        };
        monitor.update_drone(2, status2);
        assert_eq!(monitor.get_drone(2).unwrap().health, HealthStatus::Degraded);
    }
}
