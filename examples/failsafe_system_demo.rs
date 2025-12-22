//! Failsafe System Demo
//!
//! This example demonstrates the comprehensive failsafe system:
//! - Battery failsafe (RTL, Land)
//! - GPS failsafe (Position hold, Land)
//! - RC link failsafe (RTL, Land)
//! - Geofence violation handling
//! - Motor failure detection
//! - Communication timeout handling
//!
//! # Running
//! ```bash
//! cargo run --example failsafe_system_demo --features std
//! ```

use std::time::Instant;

use drone_swarm_system::failsafe::{
    FailsafeManager, FailsafeConfig, FailsafeState, FailsafeAction, FailsafeTrigger,
};

fn action_to_string(action: FailsafeAction) -> &'static str {
    match action {
        FailsafeAction::None => "None",
        FailsafeAction::Warn => "Warn",
        FailsafeAction::Hold => "Position Hold",
        FailsafeAction::Descend => "Descend",
        FailsafeAction::ReturnToLaunch => "Return to Launch",
        FailsafeAction::Land => "Land",
        FailsafeAction::EmergencyStop => "Emergency Stop",
    }
}

fn trigger_to_string(trigger: FailsafeTrigger) -> &'static str {
    match trigger {
        FailsafeTrigger::None => "None",
        FailsafeTrigger::BatteryWarning => "Battery Warning",
        FailsafeTrigger::BatteryCritical => "Battery Critical",
        FailsafeTrigger::RcLost => "RC Lost",
        FailsafeTrigger::DataLinkLost => "Data Link Lost",
        FailsafeTrigger::GpsLost => "GPS Lost",
        FailsafeTrigger::GeofenceBreach => "Geofence Breach",
        FailsafeTrigger::MotorFailure => "Motor Failure",
        FailsafeTrigger::SensorFailure => "Sensor Failure",
        FailsafeTrigger::CollisionImminent => "Collision Imminent",
        FailsafeTrigger::AltitudeLimit => "Altitude Limit",
        FailsafeTrigger::FlightTimeLimit => "Flight Time Limit",
        FailsafeTrigger::ManualTrigger => "Manual Trigger",
    }
}

fn run_scenario(name: &str, description: &str, scenario_fn: impl FnOnce(&mut FailsafeManager)) {
    println!("\n{}", "=".repeat(60));
    println!("Scenario: {}", name);
    println!("{}", "=".repeat(60));
    println!("{}\n", description);

    let config = FailsafeConfig::default();
    let mut manager = FailsafeManager::new(config);

    println!("Configuration:");
    println!("  Battery warning: {}% | Critical: {}%",
        manager.config().battery_warning_percent,
        manager.config().battery_critical_percent);
    println!("  RC timeout: {}ms | Data link timeout: {}ms",
        manager.config().rc_lost_timeout_ms,
        manager.config().data_link_timeout_ms);
    println!("  Min GPS satellites: {}", manager.config().min_gps_satellites);
    println!("  Max altitude: {}m", manager.config().max_altitude);
    println!();

    scenario_fn(&mut manager);

    println!("\nFinal state:");
    let result = manager.active();
    println!("  Active failsafe: {}", action_to_string(result.action));
    println!("  Trigger: {}", trigger_to_string(result.trigger));
}

fn main() {
    println!("=== Failsafe System Demo ===\n");
    println!("This demo shows how the failsafe system responds to various");
    println!("emergency conditions that can occur during drone operations.\n");

    let start_time = Instant::now();

    // Scenario 1: Battery depletion
    run_scenario(
        "Battery Depletion",
        "Battery drains from normal levels to critical during flight.",
        |manager| {
            let levels = [
                (80, "Normal"),
                (50, "Good"),
                (35, "Moderate"),
                (28, "Warning threshold"),
                (18, "Low"),
                (12, "Critical"),
                (8, "Emergency"),
            ];

            for (level, desc) in levels {
                let state = FailsafeState {
                    battery_percent: level,
                    gps_fix: 3,
                    gps_satellites: 12,
                    altitude: 30.0,
                    rc_lost_duration_ms: 0,
                    data_link_lost_duration_ms: 0,
                    ..FailsafeState::default()
                };

                manager.update_state(state);
                let result = manager.evaluate();

                println!("Battery: {:3}% ({:<20}) -> Action: {}",
                    level, desc, action_to_string(result.action));
            }
        }
    );

    // Scenario 2: GPS signal loss
    run_scenario(
        "GPS Signal Degradation and Loss",
        "GPS signal degrades and is eventually lost completely.",
        |manager| {
            let states = [
                (3, 14, "Excellent GPS"),
                (3, 10, "Good GPS"),
                (3, 6, "Marginal GPS"),
                (3, 4, "Degraded GPS"),
                (2, 3, "2D fix only"),
                (0, 0, "GPS Lost"),
            ];

            for (fix, sats, desc) in states {
                let state = FailsafeState {
                    battery_percent: 80,
                    gps_fix: fix,
                    gps_satellites: sats,
                    altitude: 30.0,
                    rc_lost_duration_ms: 0,
                    data_link_lost_duration_ms: 0,
                    ..FailsafeState::default()
                };

                manager.update_state(state);
                let result = manager.evaluate();

                println!("GPS: fix={} sats={:2} ({:<15}) -> Action: {}",
                    fix, sats, desc, action_to_string(result.action));
            }
        }
    );

    // Scenario 3: RC Link Loss
    run_scenario(
        "Communication Link Loss",
        "RC and telemetry links are lost progressively.",
        |manager| {
            let states = [
                (true, true, "Both links OK"),
                (false, true, "RC lost, telemetry OK"),
                (true, false, "RC OK, telemetry lost"),
                (false, false, "Both links lost"),
            ];

            for (rc, data, desc) in states {
                let state = FailsafeState {
                    battery_percent: 80,
                    gps_fix: 3,
                    gps_satellites: 12,
                    altitude: 30.0,
                    rc_lost_duration_ms: if rc { 0 } else { 5000 },
                    data_link_lost_duration_ms: if data { 0 } else { 10000 },
                    ..FailsafeState::default()
                };

                manager.update_state(state);
                let result = manager.evaluate();

                println!("RC: {:5} | Telem: {:5} ({:<25}) -> Action: {}",
                    rc, data, desc, action_to_string(result.action));
            }
        }
    );

    // Scenario 4: Altitude limit
    run_scenario(
        "Altitude Limit Violation",
        "Drone approaches and exceeds maximum altitude.",
        |manager| {
            let altitudes = [30.0, 80.0, 100.0, 115.0, 125.0, 150.0];

            for alt in altitudes {
                let state = FailsafeState {
                    battery_percent: 80,
                    gps_fix: 3,
                    gps_satellites: 12,
                    altitude: alt,
                    rc_lost_duration_ms: 0,
                    data_link_lost_duration_ms: 0,
                    ..FailsafeState::default()
                };

                manager.update_state(state);
                let result = manager.evaluate();

                let status = if alt <= manager.config().max_altitude {
                    "Within limits"
                } else {
                    "EXCEEDED"
                };

                println!("Altitude: {:5.1}m ({:<15}) -> Action: {}",
                    alt, status, action_to_string(result.action));
            }
        }
    );

    // Scenario 5: Motor/Sensor failure
    run_scenario(
        "System Failures",
        "Various system failures detected during flight.",
        |manager| {
            let state = FailsafeState {
                battery_percent: 80,
                gps_fix: 3,
                gps_satellites: 12,
                altitude: 30.0,
                rc_lost_duration_ms: 0,
                data_link_lost_duration_ms: 0,
                motor_health: 0xFF,
                sensor_health: 0xFF,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("Normal operation      -> Action: {}", action_to_string(result.action));

            // Sensor failure
            let state = FailsafeState {
                battery_percent: 80,
                gps_fix: 3,
                gps_satellites: 12,
                altitude: 30.0,
                rc_lost_duration_ms: 0,
                data_link_lost_duration_ms: 0,
                motor_health: 0xFF,
                sensor_health: 0x00,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("Sensor failure        -> Action: {}", action_to_string(result.action));

            // Motor failure
            let state = FailsafeState {
                battery_percent: 80,
                gps_fix: 3,
                gps_satellites: 12,
                altitude: 30.0,
                rc_lost_duration_ms: 0,
                data_link_lost_duration_ms: 0,
                motor_health: 0x00,
                sensor_health: 0xFF,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("Motor failure         -> Action: {}", action_to_string(result.action));
        }
    );

    // Scenario 6: Multiple simultaneous failures
    run_scenario(
        "Multiple Simultaneous Failures",
        "Combined failures require priority-based decision making.",
        |manager| {
            println!("Step 1: Low battery + GPS degradation");
            let state = FailsafeState {
                battery_percent: 25,
                gps_fix: 2,
                gps_satellites: 4,
                altitude: 30.0,
                rc_lost_duration_ms: 0,
                data_link_lost_duration_ms: 0,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("  -> Trigger: {} | Action: {}",
                trigger_to_string(result.trigger), action_to_string(result.action));

            println!("\nStep 2: Add RC link loss");
            let state = FailsafeState {
                battery_percent: 25,
                gps_fix: 2,
                gps_satellites: 4,
                altitude: 30.0,
                rc_lost_duration_ms: 5000,
                data_link_lost_duration_ms: 0,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("  -> Trigger: {} | Action: {}",
                trigger_to_string(result.trigger), action_to_string(result.action));

            println!("\nStep 3: Add motor failure");
            let state = FailsafeState {
                battery_percent: 25,
                gps_fix: 2,
                gps_satellites: 4,
                altitude: 30.0,
                rc_lost_duration_ms: 5000,
                data_link_lost_duration_ms: 0,
                motor_health: 0x00,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("  -> Trigger: {} | Action: {}",
                trigger_to_string(result.trigger), action_to_string(result.action));

            println!("\n  Failsafe system prioritizes most critical action");
            println!("  to ensure safe recovery in degraded conditions.");
        }
    );

    // Scenario 7: Manual trigger
    run_scenario(
        "Manual Failsafe Trigger",
        "Operator manually triggers failsafe action.",
        |manager| {
            println!("Normal operation:");
            let state = FailsafeState {
                battery_percent: 80,
                gps_fix: 3,
                gps_satellites: 12,
                altitude: 30.0,
                rc_lost_duration_ms: 0,
                data_link_lost_duration_ms: 0,
                ..FailsafeState::default()
            };
            manager.update_state(state);
            let result = manager.evaluate();
            println!("  -> Action: {}", action_to_string(result.action));

            println!("\nOperator triggers manual RTL:");
            manager.manual_trigger(FailsafeAction::ReturnToLaunch);
            let result = manager.active();
            println!("  -> Action: {} (locked)", action_to_string(result.action));

            println!("\nAttempt to clear without unlock:");
            let cleared = manager.clear();
            println!("  -> Clear result: {} (should be false)", cleared);

            println!("\nUnlock and clear:");
            manager.unlock();
            let cleared = manager.clear();
            println!("  -> Clear result: {} (should be true)", cleared);
        }
    );

    // Compare configurations
    println!("\n{}", "=".repeat(60));
    println!("Configuration Comparison");
    println!("{}", "=".repeat(60));
    println!("\nComparing default vs conservative vs aggressive configs:\n");

    let configs = [
        ("Default", FailsafeConfig::default()),
        ("Conservative", FailsafeConfig::conservative()),
        ("Aggressive", FailsafeConfig::aggressive()),
    ];

    println!("{:<15} {:>12} {:>12} {:>10} {:>15}",
        "Config", "Batt Warn", "Batt Crit", "RC Timeout", "Battery Action");
    println!("{}", "-".repeat(65));

    for (name, config) in &configs {
        println!("{:<15} {:>11}% {:>11}% {:>9}ms {:>15}",
            name,
            config.battery_warning_percent,
            config.battery_critical_percent,
            config.rc_lost_timeout_ms,
            action_to_string(config.battery_action));
    }

    let elapsed = start_time.elapsed();
    println!("\n{}", "=".repeat(60));
    println!("=== Demo Complete ===");
    println!("Total runtime: {:.2}ms", elapsed.as_secs_f32() * 1000.0);

    println!("\n[OK] Failsafe system demo completed!");
    println!("\nKey failsafe features:");
    println!("  - Hierarchical priority system");
    println!("  - Graceful degradation");
    println!("  - Manual trigger and lock capability");
    println!("  - Multiple simultaneous failure handling");
    println!("  - Configurable thresholds and actions");
}