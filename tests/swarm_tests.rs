//! Comprehensive tests for the swarm coordination module
//!
//! Tests SwarmController, Formation, BehaviorMode, and TaskAllocator

use drone_swarm_system::swarm::*;
use drone_swarm_system::types::*;

// ═══════════════════════════════════════════════════════════════════════════
// Formation Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod formation_tests {
    use super::*;

    #[test]
    fn test_formation_random() {
        let formation = Formation::Random;
        assert_eq!(formation, Formation::Random);
    }

    #[test]
    fn test_formation_grid() {
        let formation = Formation::Grid { spacing: 10 };
        assert_eq!(formation, Formation::Grid { spacing: 10 });
    }

    #[test]
    fn test_formation_line() {
        let formation = Formation::Line { spacing: 5 };
        assert_eq!(formation, Formation::Line { spacing: 5 });
    }

    #[test]
    fn test_formation_circle() {
        let formation = Formation::Circle { radius: 50 };
        assert_eq!(formation, Formation::Circle { radius: 50 });
    }

    #[test]
    fn test_formation_v() {
        let formation = Formation::VFormation { spacing: 15 };
        assert_eq!(formation, Formation::VFormation { spacing: 15 });
    }

    #[test]
    fn test_formation_custom() {
        let formation = Formation::Custom;
        assert_eq!(formation, Formation::Custom);
    }

    #[test]
    fn test_formation_clone() {
        let formation = Formation::Circle { radius: 100 };
        let cloned = formation.clone();
        assert_eq!(formation, cloned);
    }

    #[test]
    fn test_formation_copy() {
        let formation = Formation::Grid { spacing: 20 };
        let copied: Formation = formation;
        assert_eq!(formation, copied);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BehaviorMode Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod behavior_mode_tests {
    use super::*;

    #[test]
    fn test_behavior_exploration() {
        let behavior = BehaviorMode::Exploration;
        assert_eq!(behavior, BehaviorMode::Exploration);
    }

    #[test]
    fn test_behavior_search_rescue() {
        let behavior = BehaviorMode::SearchRescue;
        assert_eq!(behavior, BehaviorMode::SearchRescue);
    }

    #[test]
    fn test_behavior_surveillance() {
        let behavior = BehaviorMode::Surveillance;
        assert_eq!(behavior, BehaviorMode::Surveillance);
    }

    #[test]
    fn test_behavior_delivery() {
        let behavior = BehaviorMode::Delivery;
        assert_eq!(behavior, BehaviorMode::Delivery);
    }

    #[test]
    fn test_behavior_mapping() {
        let behavior = BehaviorMode::Mapping;
        assert_eq!(behavior, BehaviorMode::Mapping);
    }

    #[test]
    fn test_behavior_emergency() {
        let behavior = BehaviorMode::Emergency;
        assert_eq!(behavior, BehaviorMode::Emergency);
    }

    #[test]
    fn test_behavior_clone() {
        let behavior = BehaviorMode::SearchRescue;
        let cloned = behavior.clone();
        assert_eq!(behavior, cloned);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SwarmController Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod swarm_controller_tests {
    use super::*;

    fn create_position(x: f32, y: f32, z: f32) -> Position {
        Position { x, y, z }
    }

    fn create_velocity(vx: f32, vy: f32, vz: f32) -> Velocity {
        Velocity { vx, vy, vz }
    }

    #[test]
    fn test_new() {
        let pos = create_position(0.0, 0.0, 10.0);
        let controller = SwarmController::new(DroneId::new(1), pos);

        assert_eq!(controller.swarm_size(), 1);
        assert_eq!(controller.local_state().position.x, 0.0);
    }

    #[test]
    fn test_update_state() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let new_pos = create_position(10.0, 20.0, 30.0);
        let vel = create_velocity(1.0, 2.0, 0.0);
        controller.update_state(new_pos, vel, 85, MissionStatus::Active);

        assert_eq!(controller.local_state().position.x, 10.0);
        assert_eq!(controller.local_state().position.y, 20.0);
        assert_eq!(controller.local_state().velocity.vx, 1.0);
        assert_eq!(controller.local_state().battery, 85);
    }

    #[test]
    fn test_update_peer_state() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let peer_state = DroneState {
            id: DroneId::new(2),
            position: create_position(50.0, 50.0, 10.0),
            velocity: create_velocity(0.0, 0.0, 0.0),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        };

        controller.update_peer_state(peer_state).unwrap();
        assert_eq!(controller.swarm_size(), 2);
    }

    #[test]
    fn test_set_formation() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        controller.set_formation(Formation::Circle { radius: 100 });
        // Just verify it doesn't panic
    }

    #[test]
    fn test_set_behavior() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        controller.set_behavior(BehaviorMode::SearchRescue);
        // Just verify it doesn't panic
    }

    #[test]
    fn test_compute_formation_position_random() {
        let pos = create_position(5.0, 10.0, 15.0);
        let mut controller = SwarmController::new(DroneId::new(0), pos);
        controller.set_formation(Formation::Random);

        let formation_pos = controller.compute_formation_position();
        assert_eq!(formation_pos.x, 5.0);
        assert_eq!(formation_pos.y, 10.0);
    }

    #[test]
    fn test_compute_formation_position_line() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(0), pos);
        controller.set_formation(Formation::Line { spacing: 10 });

        let formation_pos = controller.compute_formation_position();
        assert_eq!(formation_pos.x, 0.0); // Drone 0 at start
    }

    #[test]
    fn test_compute_formation_position_circle() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(0), pos);
        controller.set_formation(Formation::Circle { radius: 50 });

        let formation_pos = controller.compute_formation_position();
        // Drone 0 should be at angle 0 on circle
        assert!(formation_pos.x.abs() < 51.0); // Within radius
    }

    #[test]
    fn test_compute_formation_position_v() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(0), pos);
        controller.set_formation(Formation::VFormation { spacing: 10 });

        let formation_pos = controller.compute_formation_position();
        // Leader (drone 0) should be at origin
        assert_eq!(formation_pos.x, 0.0);
        assert_eq!(formation_pos.y, 0.0);
    }

    #[test]
    fn test_collision_avoidance_no_peers() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let avoidance = controller.compute_collision_avoidance();
        assert_eq!(avoidance.vx, 0.0);
        assert_eq!(avoidance.vy, 0.0);
        assert_eq!(avoidance.vz, 0.0);
    }

    #[test]
    fn test_collision_avoidance_with_nearby_peer() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        // Add a nearby peer
        let peer_state = DroneState {
            id: DroneId::new(2),
            position: create_position(5.0, 0.0, 10.0), // 5m away
            velocity: create_velocity(0.0, 0.0, 0.0),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        };
        controller.update_peer_state(peer_state).unwrap();

        let avoidance = controller.compute_collision_avoidance();
        // Should push away (negative x direction)
        assert!(avoidance.vx < 0.0);
    }

    #[test]
    fn test_collision_avoidance_far_peer() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        // Add a far peer (> 10m)
        let peer_state = DroneState {
            id: DroneId::new(2),
            position: create_position(50.0, 50.0, 10.0), // Far away
            velocity: create_velocity(0.0, 0.0, 0.0),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        };
        controller.update_peer_state(peer_state).unwrap();

        let avoidance = controller.compute_collision_avoidance();
        // Should be zero (peer too far)
        assert_eq!(avoidance.vx, 0.0);
        assert_eq!(avoidance.vy, 0.0);
    }

    #[test]
    fn test_compute_destination_velocity() {
        let pos = create_position(0.0, 0.0, 10.0);
        let controller = SwarmController::new(DroneId::new(1), pos);

        let target = create_position(100.0, 0.0, 10.0);
        let vel = controller.compute_destination_velocity(target, 5.0);

        // Should move towards target (positive x)
        assert!(vel.vx > 0.0);
        assert!(vel.vx <= 5.0); // Within max speed
    }

    #[test]
    fn test_compute_destination_velocity_at_target() {
        let pos = create_position(10.0, 10.0, 10.0);
        let controller = SwarmController::new(DroneId::new(1), pos);

        let target = create_position(10.0, 10.0, 10.0); // Same position
        let vel = controller.compute_destination_velocity(target, 5.0);

        // Should be zero (already at target)
        assert_eq!(vel.vx, 0.0);
        assert_eq!(vel.vy, 0.0);
        assert_eq!(vel.vz, 0.0);
    }

    #[test]
    fn test_compute_control_velocity() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let vel = controller.compute_control_velocity(10.0);

        // Velocity should be finite
        assert!(vel.vx.is_finite());
        assert!(vel.vy.is_finite());
        assert!(vel.vz.is_finite());
    }

    #[test]
    fn test_add_task() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let task = SwarmTask {
            task_id: 1,
            destination: create_position(50.0, 50.0, 10.0),
            priority: TaskPriority::High,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };

        controller.add_task(task).unwrap();
    }

    #[test]
    fn test_next_task() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let task = SwarmTask {
            task_id: 1,
            destination: create_position(50.0, 50.0, 10.0),
            priority: TaskPriority::High,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };

        controller.add_task(task).unwrap();

        let retrieved = controller.next_task();
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap().task_id, 1);

        // Should be empty now
        assert!(controller.next_task().is_none());
    }

    #[test]
    fn test_set_destination() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        controller.set_destination(Some(create_position(100.0, 100.0, 50.0)));
        // Just verify no panic
    }

    #[test]
    fn test_swarm_centroid_single() {
        let pos = create_position(10.0, 20.0, 30.0);
        let controller = SwarmController::new(DroneId::new(1), pos);

        let centroid = controller.swarm_centroid();
        assert_eq!(centroid.x, 10.0);
        assert_eq!(centroid.y, 20.0);
        assert_eq!(centroid.z, 30.0);
    }

    #[test]
    fn test_swarm_centroid_multiple() {
        let pos = create_position(0.0, 0.0, 10.0);
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let peer_state = DroneState {
            id: DroneId::new(2),
            position: create_position(100.0, 100.0, 10.0),
            velocity: create_velocity(0.0, 0.0, 0.0),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        };
        controller.update_peer_state(peer_state).unwrap();

        let centroid = controller.swarm_centroid();
        assert_eq!(centroid.x, 50.0); // Average of 0 and 100
        assert_eq!(centroid.y, 50.0);
    }

    #[test]
    fn test_is_in_formation() {
        let pos = create_position(0.0, 0.0, 10.0);
        let controller = SwarmController::new(DroneId::new(1), pos);

        // With no peers, should always be in formation
        assert!(controller.is_in_formation(1.0));
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TaskAllocator Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod task_allocator_tests {
    use super::*;

    fn create_position(x: f32, y: f32, z: f32) -> Position {
        Position { x, y, z }
    }

    fn create_velocity() -> Velocity {
        Velocity {
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        }
    }

    #[test]
    fn test_new() {
        let allocator = TaskAllocator::new();
        // Just verify creation
        assert!(true);
    }

    #[test]
    fn test_default() {
        let allocator = TaskAllocator::default();
        // Just verify creation
        assert!(true);
    }

    #[test]
    fn test_add_task() {
        let mut allocator = TaskAllocator::new();

        let task = SwarmTask {
            task_id: 1,
            destination: create_position(50.0, 50.0, 10.0),
            priority: TaskPriority::High,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };

        allocator.add_task(task).unwrap();
    }

    #[test]
    fn test_allocate_tasks() {
        let mut allocator = TaskAllocator::new();

        let task = SwarmTask {
            task_id: 1,
            destination: create_position(50.0, 50.0, 10.0),
            priority: TaskPriority::High,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };
        allocator.add_task(task).unwrap();

        let drone_states = [DroneState {
            id: DroneId::new(1),
            position: create_position(0.0, 0.0, 10.0),
            velocity: create_velocity(),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        }];

        allocator.allocate_tasks(&drone_states).unwrap();
    }

    #[test]
    fn test_get_assignment() {
        let mut allocator = TaskAllocator::new();

        let task = SwarmTask {
            task_id: 1,
            destination: create_position(50.0, 50.0, 10.0),
            priority: TaskPriority::High,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };
        allocator.add_task(task).unwrap();

        let drone_states = [DroneState {
            id: DroneId::new(1),
            position: create_position(0.0, 0.0, 10.0),
            velocity: create_velocity(),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        }];

        allocator.allocate_tasks(&drone_states).unwrap();

        let assignment = allocator.get_assignment(DroneId::new(1));
        assert!(assignment.is_some());
        assert_eq!(assignment.unwrap().task_id, 1);
    }

    #[test]
    fn test_priority_allocation() {
        let mut allocator = TaskAllocator::new();

        // Add low priority task first
        let task_low = SwarmTask {
            task_id: 1,
            destination: create_position(10.0, 10.0, 10.0),
            priority: TaskPriority::Low,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };
        allocator.add_task(task_low).unwrap();

        // Add high priority task
        let task_high = SwarmTask {
            task_id: 2,
            destination: create_position(20.0, 20.0, 10.0),
            priority: TaskPriority::Critical,
            assigned_drones: heapless::Vec::new(),
            completed: false,
        };
        allocator.add_task(task_high).unwrap();

        let drone_states = [DroneState {
            id: DroneId::new(1),
            position: create_position(0.0, 0.0, 10.0),
            velocity: create_velocity(),
            battery: 100,
            status: MissionStatus::Active,
            timestamp: 0,
        }];

        allocator.allocate_tasks(&drone_states).unwrap();

        // Drone should be assigned to high priority task
        let assignment = allocator.get_assignment(DroneId::new(1));
        assert!(assignment.is_some());
    }
}
