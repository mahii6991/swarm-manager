//! Telemetry Monitoring System Demo
//!
//! This example demonstrates the telemetry monitoring system for drone swarms:
//! - Real-time telemetry aggregation
//! - Health status monitoring
//! - Alert generation and handling
//! - Swarm statistics computation
//!
//! # Running
//! ```bash
//! cargo run --example telemetry_monitoring --features std
//! ```

use std::time::Instant;

use drone_swarm_system::telemetry::{
    AlertSeverity, AlertThresholds, AlertType, DroneStatus, HealthStatus, SwarmStats,
    TelemetryMonitor,
};

/// Number of simulated drones
const NUM_DRONES: usize = 16;

/// Simulation steps
const SIMULATION_STEPS: usize = 100;

/// Generate simulated drone status
fn generate_drone_status(id: u8, step: usize) -> DroneStatus {
    let angle = (step as f32 * 0.1) + (id as f32 * 0.5);
    let radius = 20.0 + (id as f32);

    let mut status = DroneStatus::default();
    status.drone_id = id;

    // Position in circular pattern
    status.position = [
        radius * angle.cos(),
        radius * angle.sin(),
        -(15.0 + (id as f32 * 0.5)), // Altitude (NED)
    ];

    // Velocity tangent to circle
    status.velocity = [
        -2.0 * angle.sin(),
        2.0 * angle.cos(),
        0.0,
    ];

    // Attitude
    status.attitude = [0.0, 0.05, angle];

    // Battery drains over time
    let battery_drain = (step as f32 * 0.1) + (id as f32 * 0.5);
    status.battery_percent = (100.0 - battery_drain).max(5.0) as u8;
    status.battery_voltage = 12.6 + (status.battery_percent as f32 - 50.0) * 0.04;

    // GPS
    status.gps_fix = 3;
    status.gps_satellites = 12 - (id % 4);
    status.gps_hdop = 1.2 + (id as f32 * 0.1);

    // Signal quality
    status.rc_signal = 90 - (id % 20);
    status.rssi = -50 - (id as i8 % 20);

    // Health degrades for some drones
    status.health = if status.battery_percent < 20 {
        HealthStatus::Critical
    } else if status.battery_percent < 40 {
        HealthStatus::Degraded
    } else {
        HealthStatus::Healthy
    };

    status.armed = true;
    status.mode = 4; // Auto mode

    // Temperature
    status.cpu_temp = 45.0 + (id as f32 * 2.0);
    status.motor_temps = [40.0, 42.0, 41.0, 43.0];
    status.vibration = [0.5, 0.6, 0.4];

    status
}

fn print_swarm_stats(stats: &SwarmStats) {
    println!("  Active drones: {}", stats.active_drones);
    println!("  Healthy: {} | Warning: {} | Critical: {}",
        stats.healthy_drones, stats.warning_drones, stats.critical_drones);
    println!("  Battery - Avg: {:.1}% | Min: {}%",
        stats.avg_battery, stats.min_battery);
    println!("  Speed - Avg: {:.1}m/s | Max: {:.1}m/s",
        stats.avg_speed, stats.max_speed);
    println!("  Swarm center: ({:.1}, {:.1}, {:.1})",
        stats.center[0], stats.center[1], stats.center[2]);
    println!("  Swarm spread: {:.1}m", stats.spread);
    println!("  Active alerts: {}", stats.active_alerts);
}

fn severity_to_str(severity: AlertSeverity) -> &'static str {
    match severity {
        AlertSeverity::Info => "INFO",
        AlertSeverity::Warning => "WARN",
        AlertSeverity::Critical => "CRIT",
        AlertSeverity::Emergency => "EMER",
    }
}

fn alert_type_to_str(alert_type: AlertType) -> &'static str {
    match alert_type {
        AlertType::LowBattery => "Low Battery",
        AlertType::CriticalBattery => "Critical Battery",
        AlertType::CommLost => "Comm Lost",
        AlertType::GpsLost => "GPS Lost",
        AlertType::GeofenceBreach => "Geofence Breach",
        AlertType::MotorFailure => "Motor Failure",
        AlertType::SensorFailure => "Sensor Failure",
        AlertType::OverTemperature => "Over Temperature",
        AlertType::AltitudeLimit => "Altitude Limit",
        AlertType::SpeedLimit => "Speed Limit",
        AlertType::FormationDeviation => "Formation Deviation",
        AlertType::CollisionWarning => "Collision Warning",
        AlertType::SystemError => "System Error",
        AlertType::Custom => "Custom",
    }
}

fn main() {
    println!("=== Telemetry Monitoring System Demo ===\n");
    println!("Simulating {} drones with real-time telemetry monitoring.\n", NUM_DRONES);

    let start_time = Instant::now();

    // Create telemetry monitor with custom thresholds
    let mut monitor = TelemetryMonitor::new();

    // Configure alert thresholds
    let thresholds = AlertThresholds {
        battery_warning: 40,
        battery_critical: 20,
        max_speed: 15.0,
        max_altitude: 50.0,
        min_gps_sats: 6,
        comm_timeout_ms: 3000,
        max_temperature: 70.0,
        max_vibration: 20.0,
        formation_deviation: 15.0,
    };
    monitor.set_thresholds(thresholds);

    println!("--- Alert Thresholds ---");
    println!("  Battery warning: 40% | Critical: 20%");
    println!("  Max speed: 15 m/s | Max altitude: 50m");
    println!("  Min GPS satellites: 6");
    println!("  Communication timeout: 3000ms");
    println!();

    // Main simulation loop
    let mut total_alerts_generated = 0;
    let mut alerts_by_severity = [0usize; 4];

    for step in 0..SIMULATION_STEPS {
        let sim_time_ms = (step as u64) * 100;
        monitor.set_time(sim_time_ms);

        // Update all drone telemetry
        for id in 0..NUM_DRONES as u8 {
            let status = generate_drone_status(id, step);
            monitor.update_drone(id, status);
        }

        // Check for alerts
        let new_alerts = monitor.check_alerts();

        // Print status every 20 steps
        if step % 20 == 0 || !new_alerts.is_empty() {
            println!("\n--- Step {} (t={}ms) ---", step, sim_time_ms);

            let stats = monitor.get_swarm_stats();
            print_swarm_stats(&stats);

            // Print new alerts
            if !new_alerts.is_empty() {
                println!("\n  New Alerts:");
                for alert in &new_alerts {
                    println!("    [{}] Drone {}: {}",
                        severity_to_str(alert.severity),
                        alert.drone_id,
                        alert_type_to_str(alert.alert_type));

                    total_alerts_generated += 1;
                    alerts_by_severity[alert.severity as usize] += 1;
                }
            }
        }
    }

    // Print individual drone status at end
    println!("\n\n=== Final Drone Status ===");
    for id in 0..NUM_DRONES as u8 {
        if let Some(status) = monitor.get_drone(id) {
            let health_str = match status.health {
                HealthStatus::Healthy => "OK",
                HealthStatus::Degraded => "WARN",
                HealthStatus::Critical => "CRIT",
                HealthStatus::Offline => "OFF",
                HealthStatus::Unknown => "???",
            };
            println!(
                "Drone {:2}: Pos({:6.1},{:6.1},{:5.1}) | Batt: {:3}% | Health: {} | Sats: {}",
                id,
                status.position[0],
                status.position[1],
                status.position[2],
                status.battery_percent,
                health_str,
                status.gps_satellites
            );
        }
    }

    // Final statistics
    let elapsed = start_time.elapsed();
    let final_stats = monitor.get_swarm_stats();

    println!("\n=== Simulation Summary ===");
    println!("Total time: {:.2}ms", elapsed.as_secs_f32() * 1000.0);
    println!("Simulation steps: {}", SIMULATION_STEPS);
    println!("Simulated time: {}ms", SIMULATION_STEPS * 100);
    println!();
    println!("Final swarm state:");
    println!("  Active drones: {}", final_stats.active_drones);
    println!("  Healthy: {} | Degraded: {} | Critical: {}",
        final_stats.healthy_drones, final_stats.warning_drones, final_stats.critical_drones);
    println!();
    println!("Alert Statistics:");
    println!("  Total alerts: {}", total_alerts_generated);
    println!("  Info: {} | Warning: {} | Critical: {} | Emergency: {}",
        alerts_by_severity[0], alerts_by_severity[1],
        alerts_by_severity[2], alerts_by_severity[3]);

    println!("\n[OK] Telemetry monitoring demo completed!");
    println!("\nThis module integrates with:");
    println!("  - MAVLink telemetry streams");
    println!("  - ESP32 mesh network status");
    println!("  - Ground control station displays");
}
