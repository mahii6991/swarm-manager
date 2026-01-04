//! Comprehensive tests for the failsafe system
//!
//! Tests cover:
//! - All failsafe trigger types
//! - Priority ordering and cascade scenarios
//! - Configuration variants (default, conservative, aggressive)
//! - State transitions and locking
//! - Edge cases and boundary conditions

use drone_swarm_system::failsafe::{
    FailsafeAction, FailsafeConfig, FailsafeManager, FailsafeResult, FailsafeState,
    FailsafeTrigger,
};

// ============================================================================
// Basic Functionality Tests
// ============================================================================

mod basic_tests {
    use super::*;

    #[test]
    fn test_default_manager_creation() {
        let fsm = FailsafeManager::default();
        assert!(!fsm.is_active());
        assert_eq!(fsm.trigger_count(), 0);
    }

    #[test]
    fn test_normal_operation() {
        let mut fsm = FailsafeManager::default();
        let result = fsm.evaluate();

        assert_eq!(result.action, FailsafeAction::None);
        assert_eq!(result.trigger, FailsafeTrigger::None);
        assert!(!fsm.is_active());
    }

    #[test]
    fn test_state_update() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 50,
            gps_satellites: 10,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.action, FailsafeAction::None);
    }
}

// ============================================================================
// Battery Failsafe Tests
// ============================================================================

mod battery_tests {
    use super::*;

    #[test]
    fn test_battery_warning_far_from_home() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 25, // Below 30% warning
            distance_to_home: 200.0, // Far from home
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryWarning);
        assert_eq!(result.action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_battery_warning_close_to_home() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 25, // Below 30% warning
            distance_to_home: 50.0, // Close to home (< 100m)
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryWarning);
        assert_eq!(result.action, FailsafeAction::Warn); // Just warn when close
    }

    #[test]
    fn test_battery_critical() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 10, // Below 15% critical
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
        assert_eq!(result.action, FailsafeAction::Land); // Always land on critical
    }

    #[test]
    fn test_battery_at_exact_warning_threshold() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 30, // Exactly at warning threshold
            distance_to_home: 200.0,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // At exactly 30%, should trigger warning
        assert_eq!(result.trigger, FailsafeTrigger::BatteryWarning);
    }

    #[test]
    fn test_battery_at_exact_critical_threshold() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 15, // Exactly at critical threshold
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
    }

    #[test]
    fn test_battery_zero() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 0,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
        assert_eq!(result.action, FailsafeAction::Land);
    }
}

// ============================================================================
// GPS Failsafe Tests
// ============================================================================

mod gps_tests {
    use super::*;

    #[test]
    fn test_gps_lost_low_satellites() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            gps_satellites: 3, // Below minimum 6
            gps_fix: 3,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GpsLost);
        assert_eq!(result.action, FailsafeAction::Descend);
    }

    #[test]
    fn test_gps_lost_no_fix() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            gps_satellites: 10,
            gps_fix: 2, // Only 2D fix, need 3D
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GpsLost);
    }

    #[test]
    fn test_gps_ok() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            gps_satellites: 12,
            gps_fix: 3, // Good 3D fix
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.action, FailsafeAction::None);
    }

    #[test]
    fn test_gps_at_minimum_satellites() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            gps_satellites: 6, // Exactly at minimum
            gps_fix: 3,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // At exactly 6, should be OK (not < 6)
        assert_eq!(result.action, FailsafeAction::None);
    }
}

// ============================================================================
// Communication Failsafe Tests
// ============================================================================

mod comm_tests {
    use super::*;

    #[test]
    fn test_rc_lost() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            rc_lost_duration_ms: 3000, // Above 2000ms timeout
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::RcLost);
        assert_eq!(result.action, FailsafeAction::Hold);
    }

    #[test]
    fn test_data_link_lost() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            data_link_lost_duration_ms: 6000, // Above 5000ms timeout
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::DataLinkLost);
        assert_eq!(result.action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_rc_at_exact_timeout() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            rc_lost_duration_ms: 2000, // Exactly at timeout
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // At exactly 2000, should NOT trigger (need > 2000)
        assert_ne!(result.trigger, FailsafeTrigger::RcLost);
    }

    #[test]
    fn test_both_links_lost() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            rc_lost_duration_ms: 3000,
            data_link_lost_duration_ms: 6000,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // Data link has higher priority (65) than RC (60)
        assert_eq!(result.trigger, FailsafeTrigger::DataLinkLost);
    }
}

// ============================================================================
// Geofence and Altitude Tests
// ============================================================================

mod geofence_tests {
    use super::*;

    #[test]
    fn test_geofence_breach() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            inside_geofence: false,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GeofenceBreach);
        assert_eq!(result.action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_geofence_disabled() {
        let config = FailsafeConfig {
            geofence_enabled: false,
            ..Default::default()
        };
        let mut fsm = FailsafeManager::new(config);
        let state = FailsafeState {
            inside_geofence: false,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_ne!(result.trigger, FailsafeTrigger::GeofenceBreach);
    }

    #[test]
    fn test_altitude_limit_exceeded() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 150.0, // Above 120m limit
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::AltitudeLimit);
        assert_eq!(result.action, FailsafeAction::Descend);
    }

    #[test]
    fn test_altitude_at_limit() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 120.0, // Exactly at limit
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // At exactly 120, should NOT trigger (need > 120)
        assert_ne!(result.trigger, FailsafeTrigger::AltitudeLimit);
    }
}

// ============================================================================
// Motor and Sensor Failure Tests
// ============================================================================

mod hardware_tests {
    use super::*;

    #[test]
    fn test_motor_failure_in_air() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            motor_health: 0xFE, // One motor failed
            on_ground: false,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::MotorFailure);
        assert_eq!(result.action, FailsafeAction::Land);
    }

    #[test]
    fn test_motor_failure_on_ground() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            motor_health: 0xFE,
            on_ground: true, // On ground - less critical
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // Motor failure on ground shouldn't trigger failsafe
        assert_ne!(result.trigger, FailsafeTrigger::MotorFailure);
    }

    #[test]
    fn test_sensor_failure() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            sensor_health: 0xFE, // Sensor failed
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::SensorFailure);
        assert_eq!(result.action, FailsafeAction::Land);
    }

    #[test]
    fn test_collision_warning() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            collision_warning: true,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::CollisionImminent);
        assert_eq!(result.action, FailsafeAction::Hold);
    }

    #[test]
    fn test_collision_avoidance_disabled() {
        let config = FailsafeConfig {
            collision_avoidance_enabled: false,
            ..Default::default()
        };
        let mut fsm = FailsafeManager::new(config);
        let state = FailsafeState {
            collision_warning: true,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_ne!(result.trigger, FailsafeTrigger::CollisionImminent);
    }
}

// ============================================================================
// Priority and Cascade Tests
// ============================================================================

mod priority_tests {
    use super::*;

    #[test]
    fn test_motor_failure_highest_priority() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            motor_health: 0xFE, // Motor failed (priority 100)
            battery_percent: 10, // Critical battery (priority 90)
            gps_satellites: 3,  // GPS lost (priority 80)
            on_ground: false,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::MotorFailure);
    }

    #[test]
    fn test_critical_battery_over_gps() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 10, // Critical battery (priority 90)
            gps_satellites: 3,  // GPS lost (priority 80)
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryCritical);
    }

    #[test]
    fn test_collision_over_battery_warning() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            collision_warning: true, // Priority 95
            battery_percent: 25, // Warning level (priority 50)
            distance_to_home: 200.0,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::CollisionImminent);
    }

    #[test]
    fn test_cascade_gps_plus_geofence() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            gps_satellites: 3, // GPS lost (priority 80)
            inside_geofence: false, // Geofence breach (priority 75)
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::GpsLost);
    }

    #[test]
    fn test_multiple_low_priority_triggers() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            rc_lost_duration_ms: 3000, // Priority 60
            flight_time_secs: 2000, // Priority 40 (above 1800s limit)
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::RcLost); // Higher priority
    }
}

// ============================================================================
// Configuration Variant Tests
// ============================================================================

mod config_tests {
    use super::*;

    #[test]
    fn test_conservative_config() {
        let config = FailsafeConfig::conservative();
        assert_eq!(config.battery_warning_percent, 40);
        assert_eq!(config.battery_critical_percent, 25);
        assert_eq!(config.min_gps_satellites, 8);
        assert_eq!(config.max_altitude, 100.0);
    }

    #[test]
    fn test_aggressive_config() {
        let config = FailsafeConfig::aggressive();
        assert_eq!(config.battery_warning_percent, 20);
        assert_eq!(config.battery_critical_percent, 10);
        assert_eq!(config.min_gps_satellites, 4);
        assert_eq!(config.max_altitude, 150.0);
    }

    #[test]
    fn test_conservative_triggers_earlier() {
        let config = FailsafeConfig::conservative();
        let mut fsm = FailsafeManager::new(config);
        let state = FailsafeState {
            battery_percent: 35, // Would be OK with default, but triggers conservative
            distance_to_home: 200.0,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::BatteryWarning);
    }

    #[test]
    fn test_aggressive_allows_more() {
        let config = FailsafeConfig::aggressive();
        let mut fsm = FailsafeManager::new(config);
        let state = FailsafeState {
            battery_percent: 25, // Would trigger warning with default
            distance_to_home: 200.0,
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // With aggressive config, 25% is above 20% warning threshold
        assert_eq!(result.action, FailsafeAction::None);
    }

    #[test]
    fn test_config_update() {
        let mut fsm = FailsafeManager::default();
        let new_config = FailsafeConfig::conservative();
        fsm.set_config(new_config);

        assert_eq!(fsm.config().battery_warning_percent, 40);
    }
}

// ============================================================================
// State Management Tests
// ============================================================================

mod state_tests {
    use super::*;

    #[test]
    fn test_manual_trigger() {
        let mut fsm = FailsafeManager::default();
        fsm.manual_trigger(FailsafeAction::Land);

        assert!(fsm.is_active());
        assert_eq!(fsm.active().action, FailsafeAction::Land);
        assert_eq!(fsm.active().trigger, FailsafeTrigger::ManualTrigger);
        assert_eq!(fsm.trigger_count(), 1);
    }

    #[test]
    fn test_locked_cannot_clear() {
        let mut fsm = FailsafeManager::default();
        fsm.manual_trigger(FailsafeAction::Land);

        // Cannot clear while locked
        assert!(!fsm.clear());
        assert!(fsm.is_active());
    }

    #[test]
    fn test_unlock_and_clear() {
        let mut fsm = FailsafeManager::default();
        fsm.manual_trigger(FailsafeAction::Land);

        fsm.unlock();
        assert!(fsm.clear());
        assert!(!fsm.is_active());
    }

    #[test]
    fn test_cannot_clear_with_active_condition() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 10, // Critical battery
            ..Default::default()
        };
        fsm.update_state(state);
        fsm.evaluate();

        fsm.unlock();
        // Cannot clear because condition is still active
        assert!(!fsm.clear());
    }

    #[test]
    fn test_trigger_count_increments() {
        let mut fsm = FailsafeManager::default();

        // First trigger
        let state = FailsafeState {
            battery_percent: 10,
            ..Default::default()
        };
        fsm.update_state(state);
        fsm.evaluate();
        assert_eq!(fsm.trigger_count(), 1);

        // Same condition doesn't increment again (already higher priority)
        fsm.evaluate();
        assert_eq!(fsm.trigger_count(), 1);
    }

    #[test]
    fn test_locked_returns_current_failsafe() {
        let mut fsm = FailsafeManager::default();
        fsm.manual_trigger(FailsafeAction::Hold);

        // Update with different condition
        let state = FailsafeState {
            battery_percent: 10, // Would normally be higher priority
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // Should still return the locked manual trigger
        assert_eq!(result.trigger, FailsafeTrigger::ManualTrigger);
    }
}

// ============================================================================
// RTL Safety Tests
// ============================================================================

mod rtl_tests {
    use super::*;

    #[test]
    fn test_rtl_safe() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 50.0,
            battery_percent: 50,
            gps_fix: 3,
            ..Default::default()
        };
        fsm.update_state(state);

        assert!(fsm.is_rtl_safe());
    }

    #[test]
    fn test_rtl_unsafe_low_altitude() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 5.0, // Below min_rtl_altitude (10m)
            battery_percent: 50,
            gps_fix: 3,
            ..Default::default()
        };
        fsm.update_state(state);

        assert!(!fsm.is_rtl_safe());
    }

    #[test]
    fn test_rtl_unsafe_critical_battery() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 50.0,
            battery_percent: 10, // Critical
            gps_fix: 3,
            ..Default::default()
        };
        fsm.update_state(state);

        assert!(!fsm.is_rtl_safe());
    }

    #[test]
    fn test_rtl_unsafe_no_gps() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            altitude: 50.0,
            battery_percent: 50,
            gps_fix: 2, // Only 2D fix
            ..Default::default()
        };
        fsm.update_state(state);

        assert!(!fsm.is_rtl_safe());
    }

    #[test]
    fn test_descent_rate_normal() {
        let fsm = FailsafeManager::default();
        // No active failsafe - should return config rate
        assert!((fsm.get_descent_rate() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_descent_rate_critical_battery() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 10,
            ..Default::default()
        };
        fsm.update_state(state);
        fsm.evaluate();

        // Critical battery should double descent rate
        assert!((fsm.get_descent_rate() - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_descent_rate_motor_failure() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            motor_health: 0xFE,
            on_ground: false,
            ..Default::default()
        };
        fsm.update_state(state);
        fsm.evaluate();

        // Motor failure should triple descent rate
        assert!((fsm.get_descent_rate() - 3.0).abs() < 0.01);
    }
}

// ============================================================================
// Flight Time Tests
// ============================================================================

mod flight_time_tests {
    use super::*;

    #[test]
    fn test_flight_time_limit() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            flight_time_secs: 2000, // Above 1800s (30 min) limit
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        assert_eq!(result.trigger, FailsafeTrigger::FlightTimeLimit);
        assert_eq!(result.action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_flight_time_at_limit() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            flight_time_secs: 1800, // Exactly at limit
            ..Default::default()
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // At exactly limit, should NOT trigger (need > limit)
        assert_ne!(result.trigger, FailsafeTrigger::FlightTimeLimit);
    }
}

// ============================================================================
// Action Ordering Tests
// ============================================================================

mod action_ordering_tests {
    use super::*;

    #[test]
    fn test_action_ordering() {
        // Verify PartialOrd implementation
        assert!(FailsafeAction::EmergencyStop > FailsafeAction::Land);
        assert!(FailsafeAction::Land > FailsafeAction::ReturnToLaunch);
        assert!(FailsafeAction::ReturnToLaunch > FailsafeAction::Descend);
        assert!(FailsafeAction::Descend > FailsafeAction::Hold);
        assert!(FailsafeAction::Hold > FailsafeAction::Warn);
        assert!(FailsafeAction::Warn > FailsafeAction::None);
    }

    #[test]
    fn test_action_equality() {
        assert_eq!(FailsafeAction::Land, FailsafeAction::Land);
        assert_ne!(FailsafeAction::Land, FailsafeAction::Hold);
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

mod edge_case_tests {
    use super::*;

    #[test]
    fn test_all_failures_at_once() {
        let mut fsm = FailsafeManager::default();
        let state = FailsafeState {
            battery_percent: 0,
            gps_satellites: 0,
            gps_fix: 0,
            altitude: 200.0,
            rc_lost_duration_ms: 10000,
            data_link_lost_duration_ms: 10000,
            flight_time_secs: 5000,
            inside_geofence: false,
            motor_health: 0x00,
            sensor_health: 0x00,
            collision_warning: true,
            on_ground: false,
            distance_to_home: 1000.0,
        };
        fsm.update_state(state);

        let result = fsm.evaluate();
        // Motor failure should win (highest priority)
        assert_eq!(result.trigger, FailsafeTrigger::MotorFailure);
    }

    #[test]
    fn test_default_state_values() {
        let state = FailsafeState::default();
        assert_eq!(state.battery_percent, 100);
        assert_eq!(state.gps_satellites, 12);
        assert_eq!(state.gps_fix, 3);
        assert!(state.inside_geofence);
        assert_eq!(state.motor_health, 0xFF);
        assert_eq!(state.sensor_health, 0xFF);
        assert!(state.on_ground);
    }

    #[test]
    fn test_result_none() {
        let result = FailsafeResult {
            action: FailsafeAction::None,
            trigger: FailsafeTrigger::None,
            priority: 0,
            message: "Normal",
        };

        assert_eq!(result.action, FailsafeAction::None);
        assert_eq!(result.priority, 0);
    }
}
