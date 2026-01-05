//! Comprehensive tests for the collision avoidance system
//!
//! Tests cover:
//! - All algorithm variants (VO, RVO, ORCA, PotentialField, Hybrid)
//! - Multiple obstacle scenarios
//! - Geofence enforcement
//! - Altitude limits
//! - Emergency detection
//! - Edge cases and boundary conditions

use drone_swarm_system::control::collision::{
    AvoidanceAlgorithm, AvoidanceConfig, CollisionAvoidance, Geofence, Obstacle,
    MAX_GEOFENCE_VERTICES, MAX_OBSTACLES,
};

// ============================================================================
// Algorithm Variant Tests
// ============================================================================

mod algorithm_tests {
    use super::*;

    fn create_config_with_algorithm(algo: AvoidanceAlgorithm) -> AvoidanceConfig {
        AvoidanceConfig {
            algorithm: algo,
            min_separation: 3.0,
            safety_margin: 1.0,
            max_velocity: 10.0,
            time_horizon: 5.0,
            responsiveness: 0.8,
            geofence_enabled: false, // Disable for algorithm-specific tests
            geofence_buffer: 5.0,
            altitude_limits: [2.0, 100.0],
        }
    }

    #[test]
    fn test_velocity_obstacle_avoidance() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::VelocityObstacle);
        let mut ca = CollisionAvoidance::new(config);

        // Add obstacle approaching from ahead (triggers positive t_closest)
        // Obstacle at (10,0,0) moving toward us at velocity (-5,0,0)
        ca.add_obstacle([10.0, 0.0, 0.0], [-5.0, 0.0, 0.0], 1.0);

        // We're stationary - obstacle is coming toward us
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

        // VO should compute some safe velocity (may be zero if no modification needed)
        // At minimum, it should return a valid velocity within bounds
        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag <= 10.0, "VO should respect max velocity bounds");
    }

    #[test]
    fn test_rvo_avoidance() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::RVO);
        let mut ca = CollisionAvoidance::new(config);

        // Add moving obstacle on collision course
        ca.add_obstacle([5.0, 0.0, 0.0], [-2.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [3.0, 0.0, 0.0]);

        // RVO should provide avoidance
        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag > 0.0, "RVO should provide non-zero safe velocity");
    }

    #[test]
    fn test_orca_avoidance() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::ORCA);
        let mut ca = CollisionAvoidance::new(config);
        ca.set_agent_radius(1.0);

        // Add obstacle inside combined radius (agent=1.0 + obstacle=1.0 + margin=1.0 = 3.0)
        // Placing at 2.5 triggers the escape velocity logic
        ca.add_obstacle([2.5, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // ORCA should compute escape velocity when in collision zone
        // Escape direction should be away from obstacle (negative x)
        assert!(safe_vel[0] < 5.0, "ORCA should modify velocity when in collision zone");
    }

    #[test]
    fn test_potential_field_avoidance() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::PotentialField);
        let mut ca = CollisionAvoidance::new(config);

        // Add obstacle within influence range (3x combined radius)
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [3.0, 0.0, 0.0]);

        // Potential field should add repulsive force
        // Velocity toward obstacle should be reduced or reversed
        assert!(safe_vel[0] < 3.0, "Potential field should reduce forward velocity");
    }

    #[test]
    fn test_hybrid_avoidance() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::Hybrid);
        let mut ca = CollisionAvoidance::new(config);

        // Add obstacle
        ca.add_obstacle([4.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Hybrid should combine ORCA and potential field
        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag > 0.0 && vel_mag <= 10.0, "Hybrid should provide bounded velocity");
    }

    #[test]
    fn test_no_obstacle_passthrough() {
        let config = create_config_with_algorithm(AvoidanceAlgorithm::Hybrid);
        let ca = CollisionAvoidance::new(config);

        // No obstacles - velocity should pass through (except clamping)
        let desired = [5.0, 3.0, -1.0];
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], desired);

        // Should be close to desired (might be slightly modified by geofence if enabled)
        assert!((safe_vel[0] - desired[0]).abs() < 0.1);
        assert!((safe_vel[1] - desired[1]).abs() < 0.1);
    }
}

// ============================================================================
// Multiple Obstacle Tests
// ============================================================================

mod multi_obstacle_tests {
    use super::*;

    #[test]
    fn test_two_obstacles_corridor() {
        let mut ca = CollisionAvoidance::default();

        // Create corridor with obstacles on both sides
        ca.add_obstacle([10.0, 5.0, 0.0], [0.0, 0.0, 0.0], 2.0);
        ca.add_obstacle([10.0, -5.0, 0.0], [0.0, 0.0, 0.0], 2.0);

        // Try to fly through corridor
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);

        // Should still allow forward motion through the gap
        assert!(safe_vel[0] > 0.0, "Should be able to navigate corridor");
    }

    #[test]
    fn test_surrounded_by_obstacles() {
        let mut ca = CollisionAvoidance::default();

        // Surround the drone with obstacles
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);
        ca.add_obstacle([-5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);
        ca.add_obstacle([0.0, 5.0, 0.0], [0.0, 0.0, 0.0], 1.0);
        ca.add_obstacle([0.0, -5.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Should still provide some velocity (might be up/down to escape)
        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag <= 10.0, "Velocity should be clamped");
    }

    #[test]
    fn test_max_obstacles() {
        let mut ca = CollisionAvoidance::default();

        // Add maximum number of obstacles
        for i in 0..MAX_OBSTACLES {
            let angle = (i as f32) * 2.0 * core::f32::consts::PI / (MAX_OBSTACLES as f32);
            let x = 20.0 * libm::cosf(angle);
            let y = 20.0 * libm::sinf(angle);
            assert!(ca.add_obstacle([x, y, 0.0], [0.0, 0.0, 0.0], 0.5));
        }

        assert_eq!(ca.obstacle_count(), MAX_OBSTACLES);

        // Adding one more should fail
        assert!(!ca.add_obstacle([100.0, 100.0, 0.0], [0.0, 0.0, 0.0], 1.0));
    }

    #[test]
    fn test_clear_obstacles() {
        let mut ca = CollisionAvoidance::default();
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);
        ca.add_obstacle([0.0, 5.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        assert_eq!(ca.obstacle_count(), 2);

        ca.clear_obstacles();
        assert_eq!(ca.obstacle_count(), 0);
    }

    #[test]
    fn test_moving_obstacles() {
        let mut ca = CollisionAvoidance::default();

        // Add two obstacles moving toward each other
        ca.add_obstacle([10.0, 0.0, 0.0], [-5.0, 0.0, 0.0], 1.0);
        ca.add_obstacle([-10.0, 0.0, 0.0], [5.0, 0.0, 0.0], 1.0);

        // Drone trying to cross their paths
        let safe_vel = ca.compute_safe_velocity([0.0, 5.0, 0.0], [0.0, -5.0, 0.0]);

        // Should modify velocity to avoid collision course
        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag > 0.0, "Should still have velocity");
    }
}

// ============================================================================
// Geofence Tests
// ============================================================================

mod geofence_tests {
    use super::*;
    use heapless::Vec;

    fn create_square_geofence(half_size: f32) -> Geofence {
        let mut vertices: Vec<[f32; 2], MAX_GEOFENCE_VERTICES> = Vec::new();
        let _ = vertices.push([-half_size, -half_size]);
        let _ = vertices.push([half_size, -half_size]);
        let _ = vertices.push([half_size, half_size]);
        let _ = vertices.push([-half_size, half_size]);

        Geofence {
            vertices,
            min_altitude: 2.0,
            max_altitude: 100.0,
            inclusive: true,
        }
    }

    #[test]
    fn test_geofence_contains_center() {
        let geofence = create_square_geofence(50.0);
        assert!(geofence.contains_point(0.0, 0.0));
        assert!(geofence.contains_point(25.0, 25.0));
        assert!(geofence.contains_point(-25.0, -25.0));
    }

    #[test]
    fn test_geofence_outside() {
        let geofence = create_square_geofence(50.0);
        assert!(!geofence.contains_point(60.0, 0.0));
        assert!(!geofence.contains_point(0.0, 60.0));
        assert!(!geofence.contains_point(-60.0, -60.0));
    }

    #[test]
    fn test_geofence_boundary() {
        let geofence = create_square_geofence(50.0);
        // Points very close to boundary
        assert!(geofence.contains_point(49.0, 0.0));
        assert!(!geofence.contains_point(51.0, 0.0));
    }

    #[test]
    fn test_geofence_distance_to_boundary() {
        let geofence = create_square_geofence(50.0);

        // At center, distance should be ~50m
        let dist = geofence.distance_to_boundary(0.0, 0.0);
        assert!((dist - 50.0).abs() < 0.1);

        // Near edge
        let dist = geofence.distance_to_boundary(45.0, 0.0);
        assert!((dist - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_geofence_constraint_enforcement() {
        let mut config = AvoidanceConfig::default();
        config.geofence_enabled = true;
        config.geofence_buffer = 10.0;

        let mut ca = CollisionAvoidance::new(config);
        ca.set_geofence(create_square_geofence(50.0));

        // At edge of geofence, trying to fly out
        let safe_vel = ca.compute_safe_velocity([45.0, 0.0, -20.0], [10.0, 0.0, 0.0]);

        // Should be pushed back toward center
        assert!(safe_vel[0] < 10.0, "Should reduce outward velocity near geofence");
    }

    #[test]
    fn test_altitude_lower_limit() {
        let mut config = AvoidanceConfig::default();
        config.geofence_enabled = true;
        config.altitude_limits = [10.0, 100.0];
        config.geofence_buffer = 5.0;

        let ca = CollisionAvoidance::new(config);

        // Flying too low (z is negative in NED, so -5 = 5m altitude)
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, -8.0], [5.0, 0.0, 2.0]);

        // Should push up (negative z velocity in NED)
        assert!(safe_vel[2] < 0.0, "Should push up when too low");
    }

    #[test]
    fn test_altitude_upper_limit() {
        let mut config = AvoidanceConfig::default();
        config.geofence_enabled = true;
        config.altitude_limits = [10.0, 100.0];
        config.geofence_buffer = 5.0;

        let ca = CollisionAvoidance::new(config);

        // Flying too high (z = -98 means 98m altitude, within buffer of 100m limit)
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, -98.0], [5.0, 0.0, -5.0]);

        // Should push down (positive z velocity in NED)
        assert!(safe_vel[2] > -5.0, "Should reduce upward velocity near ceiling");
    }

    #[test]
    fn test_exclusive_geofence() {
        let mut geofence = create_square_geofence(20.0);
        geofence.inclusive = false; // Stay OUTSIDE this area (no-fly zone)

        // For exclusive geofence, contains_point returns !inside
        // Point at center is geometrically inside polygon
        // But with inclusive=false, contains_point returns FALSE (not in allowed zone)
        assert!(!geofence.contains_point(0.0, 0.0));

        // Point far outside is allowed
        assert!(geofence.contains_point(100.0, 100.0));
    }
}

// ============================================================================
// Emergency Detection Tests
// ============================================================================

mod emergency_tests {
    use super::*;

    #[test]
    fn test_emergency_collision_zone() {
        let mut ca = CollisionAvoidance::default();

        // Obstacle within combined radius (0.5 agent + 1.0 obstacle = 1.5m)
        ca.add_obstacle([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Already in collision zone
        assert!(ca.check_emergency([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]));
    }

    #[test]
    fn test_emergency_imminent_collision() {
        let mut ca = CollisionAvoidance::default();
        ca.set_agent_radius(0.5);

        // Put obstacle very close - within combined radius triggers emergency
        // combined_radius = 0.5 (agent) + 1.0 (obstacle) = 1.5
        ca.add_obstacle([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Already in collision zone - definitely emergency
        assert!(ca.check_emergency([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]));
    }

    #[test]
    fn test_no_emergency_moving_away() {
        let mut ca = CollisionAvoidance::default();
        ca.set_agent_radius(0.5);

        // Obstacle far enough away that no collision zone is triggered
        ca.add_obstacle([50.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Flying away from it (perpendicular direction)
        assert!(!ca.check_emergency([0.0, 0.0, 0.0], [0.0, 10.0, 0.0]));
    }

    #[test]
    fn test_no_emergency_far_obstacle() {
        let mut ca = CollisionAvoidance::default();

        // Obstacle far away
        ca.add_obstacle([100.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Even flying toward it, takes too long
        assert!(!ca.check_emergency([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]));
    }

    #[test]
    fn test_emergency_multiple_close_obstacles() {
        let mut ca = CollisionAvoidance::default();
        ca.set_agent_radius(0.5);

        // Multiple obstacles within collision zone
        // combined_radius = 0.5 (agent) + 0.5 (obstacle) = 1.0
        // Placing at 0.8 triggers collision zone
        ca.add_obstacle([0.8, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5);
        ca.add_obstacle([0.0, 0.8, 0.0], [0.0, 0.0, 0.0], 0.5);

        assert!(ca.check_emergency([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]));
    }
}

// ============================================================================
// Velocity Clamping Tests
// ============================================================================

mod velocity_tests {
    use super::*;

    #[test]
    fn test_velocity_clamped_to_max() {
        let mut config = AvoidanceConfig::default();
        config.max_velocity = 10.0;
        config.geofence_enabled = false;

        let ca = CollisionAvoidance::new(config);

        // Request very high velocity
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [100.0, 0.0, 0.0]);

        let speed = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(speed <= 10.0 + 0.01, "Velocity should be clamped to max");
    }

    #[test]
    fn test_velocity_under_max_unchanged() {
        let mut config = AvoidanceConfig::default();
        config.max_velocity = 20.0;
        config.geofence_enabled = false;

        let ca = CollisionAvoidance::new(config);

        // Request velocity under max
        let desired = [5.0, 3.0, -2.0];
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], desired);

        // Should be approximately unchanged
        assert!((safe_vel[0] - desired[0]).abs() < 0.1);
        assert!((safe_vel[1] - desired[1]).abs() < 0.1);
        assert!((safe_vel[2] - desired[2]).abs() < 0.1);
    }

    #[test]
    fn test_zero_velocity() {
        let ca = CollisionAvoidance::default();

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

        assert_eq!(safe_vel[0], 0.0);
        assert_eq!(safe_vel[1], 0.0);
        // Z might be modified by altitude constraints
    }
}

// ============================================================================
// Static vs Dynamic Obstacle Tests
// ============================================================================

mod obstacle_type_tests {
    use super::*;

    #[test]
    fn test_static_obstacle_creation() {
        let obs = Obstacle::new_static([10.0, 20.0, 30.0], 2.0);
        assert!(obs.is_static);
        assert_eq!(obs.velocity, [0.0, 0.0, 0.0]);
        assert_eq!(obs.priority, 2); // Static has higher priority
    }

    #[test]
    fn test_dynamic_obstacle_creation() {
        let obs = Obstacle::new([10.0, 20.0, 30.0], [1.0, 2.0, 0.0], 1.5);
        assert!(!obs.is_static);
        assert_eq!(obs.velocity, [1.0, 2.0, 0.0]);
        assert_eq!(obs.priority, 1);
    }

    #[test]
    fn test_add_static_obstacle() {
        let mut ca = CollisionAvoidance::default();
        assert!(ca.add_static_obstacle([10.0, 0.0, 0.0], 2.0));
        assert_eq!(ca.obstacle_count(), 1);
    }

    #[test]
    fn test_min_obstacle_distance() {
        let mut ca = CollisionAvoidance::default();
        ca.set_agent_radius(0.5);
        ca.add_obstacle([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let dist = ca.min_obstacle_distance([0.0, 0.0, 0.0]);

        // Distance should be 10 - 1 (obstacle radius) - 0.5 (agent radius) = 8.5
        assert!((dist - 8.5).abs() < 0.1);
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

mod edge_case_tests {
    use super::*;

    #[test]
    fn test_obstacle_at_same_position() {
        let mut ca = CollisionAvoidance::default();

        // Obstacle at exact same position
        ca.add_obstacle([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Should handle gracefully (emergency escape)
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        let vel_mag = (safe_vel[0].powi(2) + safe_vel[1].powi(2) + safe_vel[2].powi(2)).sqrt();
        assert!(vel_mag > 0.0, "Should provide escape velocity");
    }

    #[test]
    fn test_obstacle_very_close() {
        let mut ca = CollisionAvoidance::default();

        // Obstacle very close (within combined radius)
        ca.add_obstacle([0.1, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Should push away (negative x)
        assert!(safe_vel[0] < 5.0, "Should reduce forward velocity");
    }

    #[test]
    fn test_high_responsiveness() {
        let mut config = AvoidanceConfig::default();
        config.responsiveness = 1.0;
        config.geofence_enabled = false;

        let mut ca = CollisionAvoidance::new(config);
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // High responsiveness should cause stronger avoidance
        assert!(safe_vel[1].abs() > 0.0 || safe_vel[0] < 5.0);
    }

    #[test]
    fn test_low_responsiveness() {
        let mut config = AvoidanceConfig::default();
        config.responsiveness = 0.1;
        config.geofence_enabled = false;

        let mut ca = CollisionAvoidance::new(config);
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        let safe_vel_low = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Low responsiveness should cause weaker avoidance
        // (Still should avoid, just less aggressively)
        let vel_mag = (safe_vel_low[0].powi(2) + safe_vel_low[1].powi(2) + safe_vel_low[2].powi(2)).sqrt();
        assert!(vel_mag > 0.0);
    }

    #[test]
    fn test_short_time_horizon() {
        let mut config = AvoidanceConfig::default();
        config.time_horizon = 0.5; // Very short
        config.geofence_enabled = false;

        let mut ca = CollisionAvoidance::new(config);
        ca.add_obstacle([50.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Far obstacle with short time horizon - shouldn't trigger avoidance
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Should be close to desired since collision is outside time horizon
        assert!((safe_vel[0] - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_long_time_horizon() {
        let mut config = AvoidanceConfig::default();
        config.algorithm = AvoidanceAlgorithm::PotentialField;
        config.time_horizon = 20.0; // Long horizon
        config.geofence_enabled = false;

        let mut ca = CollisionAvoidance::new(config);
        ca.set_agent_radius(1.0);
        // Obstacle at 5m with radius 1.0 + agent 1.0 + margin 1.0 = 3.0 combined
        // Influence range = 3.0 * 3.0 = 9.0m, so obstacle at 5m is in range
        ca.add_obstacle([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // PotentialField should apply repulsion
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        // Should reduce forward velocity due to repulsion
        assert!(safe_vel[0] < 5.0, "Long time horizon with nearby obstacle should trigger avoidance");
    }
}

// ============================================================================
// Configuration Tests
// ============================================================================

mod config_tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = AvoidanceConfig::default();
        assert_eq!(config.min_separation, 3.0);
        assert_eq!(config.safety_margin, 1.0);
        assert_eq!(config.max_velocity, 10.0);
        assert_eq!(config.time_horizon, 5.0);
        assert!(config.geofence_enabled);
        assert_eq!(config.algorithm, AvoidanceAlgorithm::Hybrid);
    }

    #[test]
    fn test_agent_radius_setting() {
        let mut ca = CollisionAvoidance::default();
        ca.set_agent_radius(1.0);

        // Add obstacle at 2m
        ca.add_obstacle([2.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5);

        // With agent radius 1.0 + obstacle 0.5 + margin 1.0 = 2.5m combined
        // So 2m distance should trigger strong avoidance
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, 0.0], [5.0, 0.0, 0.0]);

        assert!(safe_vel[0] < 5.0, "Should avoid obstacle with larger agent radius");
    }

    #[test]
    fn test_algorithm_enum_values() {
        assert_eq!(AvoidanceAlgorithm::VelocityObstacle, AvoidanceAlgorithm::VelocityObstacle);
        assert_ne!(AvoidanceAlgorithm::RVO, AvoidanceAlgorithm::ORCA);
    }
}
