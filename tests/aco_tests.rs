//! Comprehensive tests for Ant Colony Optimization (ACO)
//!
//! Tests path planning algorithms, obstacle avoidance, and ACO variants

use drone_swarm_system::aco::*;

#[cfg(test)]
mod position3d_tests {
    use super::*;

    #[test]
    fn test_position3d_new() {
        let pos = Position3D::new(1.0, 2.0, 3.0);

        assert_eq!(pos.x, 1.0);
        assert_eq!(pos.y, 2.0);
        assert_eq!(pos.z, 3.0);
    }

    #[test]
    fn test_position3d_distance_to_self() {
        let pos = Position3D::new(10.0, 20.0, 30.0);
        let distance = pos.distance_to(&pos);

        assert!(
            (distance - 0.0).abs() < 1e-6,
            "Distance to self should be 0"
        );
    }

    #[test]
    fn test_position3d_distance_simple() {
        let pos1 = Position3D::new(0.0, 0.0, 0.0);
        let pos2 = Position3D::new(3.0, 4.0, 0.0);

        let distance = pos1.distance_to(&pos2);

        assert!(
            (distance - 5.0).abs() < 1e-6,
            "3-4-5 triangle should have distance 5"
        );
    }

    #[test]
    fn test_position3d_distance_3d() {
        let pos1 = Position3D::new(0.0, 0.0, 0.0);
        let pos2 = Position3D::new(1.0, 1.0, 1.0);

        let distance = pos1.distance_to(&pos2);
        let expected = (3.0_f32).sqrt();

        assert!((distance - expected).abs() < 1e-6);
    }

    #[test]
    fn test_position3d_distance_symmetry() {
        let pos1 = Position3D::new(10.0, 20.0, 30.0);
        let pos2 = Position3D::new(40.0, 50.0, 60.0);

        let dist12 = pos1.distance_to(&pos2);
        let dist21 = pos2.distance_to(&pos1);

        assert!(
            (dist12 - dist21).abs() < 1e-6,
            "Distance should be symmetric"
        );
    }

    #[test]
    fn test_position3d_is_within_bounds_inside() {
        let pos = Position3D::new(5.0, 5.0, 5.0);
        let min = Position3D::new(0.0, 0.0, 0.0);
        let max = Position3D::new(10.0, 10.0, 10.0);

        assert!(pos.is_within_bounds(&min, &max));
    }

    #[test]
    fn test_position3d_is_within_bounds_outside_x() {
        let pos = Position3D::new(15.0, 5.0, 5.0);
        let min = Position3D::new(0.0, 0.0, 0.0);
        let max = Position3D::new(10.0, 10.0, 10.0);

        assert!(!pos.is_within_bounds(&min, &max));
    }

    #[test]
    fn test_position3d_is_within_bounds_outside_y() {
        let pos = Position3D::new(5.0, -1.0, 5.0);
        let min = Position3D::new(0.0, 0.0, 0.0);
        let max = Position3D::new(10.0, 10.0, 10.0);

        assert!(!pos.is_within_bounds(&min, &max));
    }

    #[test]
    fn test_position3d_is_within_bounds_outside_z() {
        let pos = Position3D::new(5.0, 5.0, 20.0);
        let min = Position3D::new(0.0, 0.0, 0.0);
        let max = Position3D::new(10.0, 10.0, 10.0);

        assert!(!pos.is_within_bounds(&min, &max));
    }

    #[test]
    fn test_position3d_is_within_bounds_on_boundary() {
        let pos = Position3D::new(10.0, 10.0, 10.0);
        let min = Position3D::new(0.0, 0.0, 0.0);
        let max = Position3D::new(10.0, 10.0, 10.0);

        assert!(pos.is_within_bounds(&min, &max));
    }

    #[test]
    fn test_position3d_clone() {
        let pos1 = Position3D::new(1.0, 2.0, 3.0);
        let pos2 = pos1.clone();

        assert_eq!(pos1.x, pos2.x);
        assert_eq!(pos1.y, pos2.y);
        assert_eq!(pos1.z, pos2.z);
    }

    #[test]
    fn test_position3d_copy() {
        let pos1 = Position3D::new(7.0, 8.0, 9.0);
        let pos2 = pos1; // Copy semantics

        assert_eq!(pos1.x, pos2.x);
        assert_eq!(pos1.y, pos2.y);
        assert_eq!(pos1.z, pos2.z);
    }
}

#[cfg(test)]
mod obstacle_tests {
    use super::*;

    #[test]
    fn test_obstacle_new() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);

        assert_eq!(obstacle.center.x, 10.0);
        assert_eq!(obstacle.center.y, 10.0);
        assert_eq!(obstacle.center.z, 10.0);
        assert_eq!(obstacle.radius, 5.0);
    }

    #[test]
    fn test_obstacle_collides_with_inside() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);
        let pos = Position3D::new(10.0, 10.0, 10.0); // At center

        assert!(obstacle.collides_with(&pos));
    }

    #[test]
    fn test_obstacle_collides_with_outside() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);
        let pos = Position3D::new(20.0, 20.0, 20.0); // Far away

        assert!(!obstacle.collides_with(&pos));
    }

    #[test]
    fn test_obstacle_collides_with_on_boundary() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);
        let pos = Position3D::new(15.0, 10.0, 10.0); // Exactly at radius

        // This depends on whether we use < or <= in collides_with
        // Currently uses <, so boundary should not collide
        assert!(!obstacle.collides_with(&pos));
    }

    #[test]
    fn test_obstacle_intersects_segment_passing_through() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);

        let p1 = Position3D::new(0.0, 10.0, 10.0);
        let p2 = Position3D::new(20.0, 10.0, 10.0);

        // Segment passes directly through center
        assert!(obstacle.intersects_segment(&p1, &p2));
    }

    #[test]
    fn test_obstacle_intersects_segment_not_intersecting() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);

        let p1 = Position3D::new(0.0, 0.0, 0.0);
        let p2 = Position3D::new(1.0, 0.0, 0.0);

        // Segment is far from obstacle
        assert!(!obstacle.intersects_segment(&p1, &p2));
    }

    #[test]
    fn test_obstacle_intersects_segment_endpoint_inside() {
        let center = Position3D::new(10.0, 10.0, 10.0);
        let obstacle = Obstacle::new(center, 5.0);

        let p1 = Position3D::new(10.0, 10.0, 10.0); // Inside
        let p2 = Position3D::new(20.0, 20.0, 20.0); // Outside

        assert!(obstacle.intersects_segment(&p1, &p2));
    }

    #[test]
    fn test_obstacle_clone() {
        let center = Position3D::new(5.0, 6.0, 7.0);
        let obstacle1 = Obstacle::new(center, 3.0);
        let obstacle2 = obstacle1.clone();

        assert_eq!(obstacle1.center.x, obstacle2.center.x);
        assert_eq!(obstacle1.radius, obstacle2.radius);
    }
}

#[cfg(test)]
mod path_tests {
    use super::*;

    #[test]
    fn test_path_new() {
        let path = Path::new();

        assert_eq!(path.waypoints.len(), 0);
        assert!(path.cost.is_infinite());
        assert!(!path.is_valid);
    }

    #[test]
    fn test_path_default() {
        let path = Path::default();

        assert_eq!(path.waypoints.len(), 0);
        assert!(path.cost.is_infinite());
        assert!(!path.is_valid);
    }

    #[test]
    fn test_path_calculate_cost_empty() {
        let mut path = Path::new();
        path.calculate_cost();

        assert!(
            path.cost.is_infinite(),
            "Empty path should have infinite cost"
        );
    }

    #[test]
    fn test_path_calculate_cost_single_waypoint() {
        let mut path = Path::new();
        path.waypoints.push(Position3D::new(0.0, 0.0, 0.0)).unwrap();
        path.calculate_cost();

        assert!(
            path.cost.is_infinite(),
            "Single waypoint should have infinite cost"
        );
    }

    #[test]
    fn test_path_calculate_cost_two_waypoints() {
        let mut path = Path::new();
        path.waypoints.push(Position3D::new(0.0, 0.0, 0.0)).unwrap();
        path.waypoints.push(Position3D::new(3.0, 4.0, 0.0)).unwrap();
        path.calculate_cost();

        assert!(
            (path.cost - 5.0).abs() < 1e-6,
            "Cost should be 5.0 for 3-4-5 triangle"
        );
    }

    #[test]
    fn test_path_calculate_cost_multiple_waypoints() {
        let mut path = Path::new();
        path.waypoints.push(Position3D::new(0.0, 0.0, 0.0)).unwrap();
        path.waypoints.push(Position3D::new(1.0, 0.0, 0.0)).unwrap();
        path.waypoints.push(Position3D::new(1.0, 1.0, 0.0)).unwrap();
        path.calculate_cost();

        assert!((path.cost - 2.0).abs() < 1e-6, "Cost should be 2.0 (1 + 1)");
    }

    #[test]
    fn test_path_validate_against_obstacles_no_obstacles() {
        let mut path = Path::new();
        path.waypoints.push(Position3D::new(0.0, 0.0, 0.0)).unwrap();
        path.waypoints
            .push(Position3D::new(10.0, 10.0, 10.0))
            .unwrap();

        let obstacles: heapless::Vec<Obstacle, MAX_OBSTACLES> = heapless::Vec::new();
        path.validate_against_obstacles(&obstacles);

        assert!(path.is_valid, "Path with no obstacles should be valid");
    }

    #[test]
    fn test_path_validate_against_obstacles_collision() {
        let mut path = Path::new();
        path.waypoints
            .push(Position3D::new(0.0, 10.0, 10.0))
            .unwrap();
        path.waypoints
            .push(Position3D::new(20.0, 10.0, 10.0))
            .unwrap();

        let mut obstacles: heapless::Vec<Obstacle, MAX_OBSTACLES> = heapless::Vec::new();
        let center = Position3D::new(10.0, 10.0, 10.0);
        obstacles.push(Obstacle::new(center, 5.0)).unwrap();

        path.validate_against_obstacles(&obstacles);

        assert!(!path.is_valid, "Path through obstacle should be invalid");
        assert!(
            path.cost.is_infinite(),
            "Invalid path should have infinite cost"
        );
    }

    #[test]
    fn test_path_validate_against_obstacles_no_collision() {
        let mut path = Path::new();
        path.waypoints.push(Position3D::new(0.0, 0.0, 0.0)).unwrap();
        path.waypoints.push(Position3D::new(1.0, 0.0, 0.0)).unwrap();

        let mut obstacles: heapless::Vec<Obstacle, MAX_OBSTACLES> = heapless::Vec::new();
        let center = Position3D::new(10.0, 10.0, 10.0);
        obstacles.push(Obstacle::new(center, 2.0)).unwrap();

        path.validate_against_obstacles(&obstacles);

        assert!(path.is_valid, "Path away from obstacle should be valid");
    }

    #[test]
    fn test_path_clone() {
        let mut path1 = Path::new();
        path1
            .waypoints
            .push(Position3D::new(1.0, 2.0, 3.0))
            .unwrap();
        path1.cost = 42.0;
        path1.is_valid = true;

        let path2 = path1.clone();

        assert_eq!(path1.waypoints.len(), path2.waypoints.len());
        assert_eq!(path1.cost, path2.cost);
        assert_eq!(path1.is_valid, path2.is_valid);
    }
}

#[cfg(test)]
mod ant_tests {
    use super::*;

    #[test]
    fn test_ant_new() {
        let start = Position3D::new(0.0, 0.0, 0.0);
        let ant = Ant::new(1, start);

        assert_eq!(ant.id, 1);
        assert_eq!(ant.path.waypoints.len(), 1);
        assert_eq!(ant.current_pos.x, 0.0);
        assert_eq!(ant.current_pos.y, 0.0);
        assert_eq!(ant.current_pos.z, 0.0);
    }

    #[test]
    fn test_ant_reset() {
        let start = Position3D::new(0.0, 0.0, 0.0);
        let mut ant = Ant::new(1, start);

        // Add some waypoints
        ant.path
            .waypoints
            .push(Position3D::new(10.0, 10.0, 10.0))
            .unwrap();
        ant.current_pos = Position3D::new(10.0, 10.0, 10.0);

        // Reset to new start
        let new_start = Position3D::new(5.0, 5.0, 5.0);
        ant.reset(new_start);

        assert_eq!(ant.path.waypoints.len(), 1);
        assert_eq!(ant.current_pos.x, 5.0);
        assert_eq!(ant.current_pos.y, 5.0);
        assert_eq!(ant.current_pos.z, 5.0);
    }

    #[test]
    fn test_ant_select_next_waypoint_empty_candidates() {
        let start = Position3D::new(0.0, 0.0, 0.0);
        let ant = Ant::new(1, start);

        let candidates: heapless::Vec<Position3D, MAX_WAYPOINTS> = heapless::Vec::new();
        let pheromones: heapless::Vec<f32, MAX_WAYPOINTS> = heapless::Vec::new();

        let result =
            ant.select_next_waypoint(&candidates, &pheromones, ACOAlgorithm::AntSystem, 42);

        assert!(result.is_none(), "Should return None for empty candidates");
    }
}

#[cfg(test)]
mod aco_algorithm_tests {
    use super::*;

    #[test]
    fn test_aco_algorithm_equality() {
        assert_eq!(ACOAlgorithm::AntSystem, ACOAlgorithm::AntSystem);
        assert_eq!(ACOAlgorithm::MMAS, ACOAlgorithm::MMAS);
        assert_eq!(ACOAlgorithm::ACS, ACOAlgorithm::ACS);

        assert_ne!(ACOAlgorithm::AntSystem, ACOAlgorithm::MMAS);
        assert_ne!(ACOAlgorithm::MMAS, ACOAlgorithm::ACS);
        assert_ne!(ACOAlgorithm::ACS, ACOAlgorithm::AntSystem);
    }

    #[test]
    fn test_aco_algorithm_clone() {
        let algo1 = ACOAlgorithm::MMAS;
        let algo2 = algo1.clone();

        assert_eq!(algo1, algo2);
    }

    #[test]
    fn test_aco_algorithm_copy() {
        let algo1 = ACOAlgorithm::ACS;
        let algo2 = algo1; // Copy semantics

        assert_eq!(algo1, algo2);
    }
}

#[cfg(test)]
mod aco_config_tests {
    use super::*;

    #[test]
    fn test_aco_config_default() {
        let config = ACOConfig::default();

        assert_eq!(config.algorithm, ACOAlgorithm::MMAS);
        assert_eq!(config.num_ants, 20);
        assert_eq!(config.max_iterations, 100);
        assert_eq!(config.pheromone_init, 1.0);
        assert!(config.evaporation_rate > 0.0);
        assert!(config.alpha > 0.0);
        assert!(config.beta > 0.0);
    }

    #[test]
    fn test_aco_config_custom() {
        let config = ACOConfig {
            algorithm: ACOAlgorithm::ACS,
            num_ants: 50,
            max_iterations: 200,
            pheromone_init: 2.0,
            evaporation_rate: 0.2,
            alpha: 1.5,
            beta: 3.0,
        };

        assert_eq!(config.algorithm, ACOAlgorithm::ACS);
        assert_eq!(config.num_ants, 50);
        assert_eq!(config.max_iterations, 200);
        assert_eq!(config.pheromone_init, 2.0);
        assert_eq!(config.evaporation_rate, 0.2);
        assert_eq!(config.alpha, 1.5);
        assert_eq!(config.beta, 3.0);
    }

    #[test]
    fn test_aco_config_clone() {
        let config1 = ACOConfig::default();
        let config2 = config1.clone();

        assert_eq!(config1.algorithm, config2.algorithm);
        assert_eq!(config1.num_ants, config2.num_ants);
        assert_eq!(config1.max_iterations, config2.max_iterations);
    }
}

#[cfg(test)]
mod aco_optimizer_tests {
    use super::*;

    #[test]
    fn test_aco_optimizer_new() {
        let config = ACOConfig::default();
        let start = Position3D::new(0.0, 0.0, 0.0);
        let goal = Position3D::new(100.0, 100.0, 100.0);
        let bounds_min = Position3D::new(-10.0, -10.0, -10.0);
        let bounds_max = Position3D::new(110.0, 110.0, 110.0);

        let optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max);

        assert!(optimizer.is_ok());
    }

    #[test]
    fn test_aco_optimizer_new_too_many_ants() {
        let mut config = ACOConfig::default();
        config.num_ants = MAX_ANTS + 1; // Exceed maximum

        let start = Position3D::new(0.0, 0.0, 0.0);
        let goal = Position3D::new(100.0, 100.0, 100.0);
        let bounds_min = Position3D::new(-10.0, -10.0, -10.0);
        let bounds_max = Position3D::new(110.0, 110.0, 110.0);

        let optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max);

        assert!(optimizer.is_err());
    }

    #[test]
    fn test_aco_optimizer_add_obstacle() {
        let config = ACOConfig::default();
        let start = Position3D::new(0.0, 0.0, 0.0);
        let goal = Position3D::new(100.0, 100.0, 100.0);
        let bounds_min = Position3D::new(-10.0, -10.0, -10.0);
        let bounds_max = Position3D::new(110.0, 110.0, 110.0);

        let mut optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max).unwrap();

        let center = Position3D::new(50.0, 50.0, 50.0);
        let obstacle = Obstacle::new(center, 10.0);

        let result = optimizer.add_obstacle(obstacle);

        assert!(result.is_ok());
    }

    #[test]
    fn test_aco_optimizer_add_multiple_obstacles() {
        let config = ACOConfig::default();
        let start = Position3D::new(0.0, 0.0, 0.0);
        let goal = Position3D::new(100.0, 100.0, 100.0);
        let bounds_min = Position3D::new(-10.0, -10.0, -10.0);
        let bounds_max = Position3D::new(110.0, 110.0, 110.0);

        let mut optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max).unwrap();

        // Add 10 obstacles
        for i in 0..10 {
            let center = Position3D::new(i as f32 * 10.0, i as f32 * 10.0, 50.0);
            let obstacle = Obstacle::new(center, 5.0);
            assert!(optimizer.add_obstacle(obstacle).is_ok());
        }
    }
}

#[cfg(test)]
mod constants_tests {
    use super::*;

    #[test]
    fn test_max_constants() {
        assert_eq!(MAX_ANTS, 50);
        assert_eq!(MAX_WAYPOINTS, 100);
        assert_eq!(MAX_OBSTACLES, 50);
    }
}
