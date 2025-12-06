//! Comprehensive tests for core types
//!
//! Tests all core type definitions, constructors, and methods

use drone_swarm_system::types::*;
use heapless::Vec;

#[cfg(test)]
mod drone_id_tests {
    use super::*;

    #[test]
    fn test_drone_id_new() {
        let id = DroneId::new(42);
        assert_eq!(id.as_u64(), 42);
    }

    #[test]
    fn test_drone_id_as_u64() {
        let id = DroneId::new(12345);
        assert_eq!(id.as_u64(), 12345);
    }

    #[test]
    fn test_drone_id_equality() {
        let id1 = DroneId::new(100);
        let id2 = DroneId::new(100);
        let id3 = DroneId::new(200);

        assert_eq!(id1, id2);
        assert_ne!(id1, id3);
    }

    #[test]
    fn test_drone_id_clone() {
        let id1 = DroneId::new(999);
        let id2 = id1.clone();

        assert_eq!(id1, id2);
    }

    #[test]
    fn test_drone_id_copy() {
        let id1 = DroneId::new(555);
        let id2 = id1; // Copy semantics

        assert_eq!(id1, id2);
    }

    #[test]
    fn test_drone_id_display() {
        let id = DroneId::new(0x123456789ABCDEF0);
        let display = format!("{}", id);

        assert!(display.contains("Drone-"));
        assert!(display.contains("123456789ABCDEF0"));
    }

    #[test]
    fn test_drone_id_display_zero() {
        let id = DroneId::new(0);
        let display = format!("{}", id);

        assert_eq!(display, "Drone-0000000000000000");
    }

    #[test]
    fn test_drone_id_display_max() {
        let id = DroneId::new(u64::MAX);
        let display = format!("{}", id);

        assert_eq!(display, "Drone-FFFFFFFFFFFFFFFF");
    }

    #[test]
    fn test_drone_id_const_new() {
        // Test that new() is const
        const ID: DroneId = DroneId::new(42);
        assert_eq!(ID.as_u64(), 42);
    }

    #[test]
    fn test_drone_id_const_as_u64() {
        const ID: DroneId = DroneId::new(100);
        const VALUE: u64 = ID.as_u64();
        assert_eq!(VALUE, 100);
    }
}

#[cfg(test)]
mod position_tests {
    use super::*;

    #[test]
    fn test_position_distance_to_self() {
        let pos = Position {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        };
        let distance = pos.distance_to(&pos);

        assert!(
            (distance - 0.0).abs() < 1e-6,
            "Distance to self should be 0"
        );
    }

    #[test]
    fn test_position_distance_simple() {
        let pos1 = Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let pos2 = Position {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };

        let distance = pos1.distance_to(&pos2);

        assert!(
            (distance - 5.0).abs() < 1e-6,
            "3-4-5 triangle should have distance 5"
        );
    }

    #[test]
    fn test_position_distance_3d() {
        let pos1 = Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let pos2 = Position {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };

        let distance = pos1.distance_to(&pos2);
        let expected = libm::sqrtf(3.0); // sqrt(1^2 + 1^2 + 1^2)

        assert!((distance - expected).abs() < 1e-6);
    }

    #[test]
    fn test_position_distance_symmetry() {
        let pos1 = Position {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        };
        let pos2 = Position {
            x: 40.0,
            y: 50.0,
            z: 60.0,
        };

        let dist12 = pos1.distance_to(&pos2);
        let dist21 = pos2.distance_to(&pos1);

        assert!(
            (dist12 - dist21).abs() < 1e-6,
            "Distance should be symmetric"
        );
    }

    #[test]
    fn test_position_distance_negative_coords() {
        let pos1 = Position {
            x: -10.0,
            y: -20.0,
            z: -30.0,
        };
        let pos2 = Position {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        };

        let distance = pos1.distance_to(&pos2);

        assert!(distance > 0.0, "Distance should always be positive");
    }

    #[test]
    fn test_position_clone() {
        let pos1 = Position {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let pos2 = pos1.clone();

        assert_eq!(pos1, pos2);
    }

    #[test]
    fn test_position_copy() {
        let pos1 = Position {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let pos2 = pos1; // Copy semantics

        assert_eq!(pos1, pos2);
    }
}

#[cfg(test)]
mod velocity_tests {
    use super::*;

    #[test]
    fn test_velocity_creation() {
        let vel = Velocity {
            vx: 1.0,
            vy: 2.0,
            vz: 3.0,
        };

        assert_eq!(vel.vx, 1.0);
        assert_eq!(vel.vy, 2.0);
        assert_eq!(vel.vz, 3.0);
    }

    #[test]
    fn test_velocity_zero() {
        let vel = Velocity {
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };

        assert_eq!(vel.vx, 0.0);
        assert_eq!(vel.vy, 0.0);
        assert_eq!(vel.vz, 0.0);
    }

    #[test]
    fn test_velocity_negative() {
        let vel = Velocity {
            vx: -5.0,
            vy: -10.0,
            vz: -15.0,
        };

        assert_eq!(vel.vx, -5.0);
        assert_eq!(vel.vy, -10.0);
        assert_eq!(vel.vz, -15.0);
    }

    #[test]
    fn test_velocity_equality() {
        let vel1 = Velocity {
            vx: 1.0,
            vy: 2.0,
            vz: 3.0,
        };
        let vel2 = Velocity {
            vx: 1.0,
            vy: 2.0,
            vz: 3.0,
        };
        let vel3 = Velocity {
            vx: 1.0,
            vy: 2.0,
            vz: 4.0,
        };

        assert_eq!(vel1, vel2);
        assert_ne!(vel1, vel3);
    }

    #[test]
    fn test_velocity_clone() {
        let vel1 = Velocity {
            vx: 5.5,
            vy: 6.6,
            vz: 7.7,
        };
        let vel2 = vel1.clone();

        assert_eq!(vel1, vel2);
    }
}

#[cfg(test)]
mod drone_state_tests {
    use super::*;

    #[test]
    fn test_drone_state_creation() {
        let state = DroneState {
            id: DroneId::new(1),
            position: Position {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            velocity: Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            },
            battery: 100,
            status: MissionStatus::Idle,
            timestamp: 1000,
        };

        assert_eq!(state.id, DroneId::new(1));
        assert_eq!(state.battery, 100);
        assert_eq!(state.status, MissionStatus::Idle);
        assert_eq!(state.timestamp, 1000);
    }

    #[test]
    fn test_drone_state_low_battery() {
        let state = DroneState {
            id: DroneId::new(2),
            position: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            velocity: Velocity {
                vx: 1.0,
                vy: 2.0,
                vz: 3.0,
            },
            battery: 15,
            status: MissionStatus::Returning,
            timestamp: 2000,
        };

        assert_eq!(state.battery, 15);
        assert_eq!(state.status, MissionStatus::Returning);
    }

    #[test]
    fn test_drone_state_clone() {
        let state1 = DroneState {
            id: DroneId::new(3),
            position: Position {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            velocity: Velocity {
                vx: 0.1,
                vy: 0.2,
                vz: 0.3,
            },
            battery: 75,
            status: MissionStatus::Active,
            timestamp: 3000,
        };

        let state2 = state1.clone();

        assert_eq!(state1.id, state2.id);
        assert_eq!(state1.battery, state2.battery);
        assert_eq!(state1.status, state2.status);
    }
}

#[cfg(test)]
mod mission_status_tests {
    use super::*;

    #[test]
    fn test_mission_status_equality() {
        assert_eq!(MissionStatus::Idle, MissionStatus::Idle);
        assert_eq!(MissionStatus::Active, MissionStatus::Active);
        assert_eq!(MissionStatus::Returning, MissionStatus::Returning);
        assert_eq!(MissionStatus::Emergency, MissionStatus::Emergency);
        assert_eq!(MissionStatus::Failed, MissionStatus::Failed);

        assert_ne!(MissionStatus::Idle, MissionStatus::Active);
        assert_ne!(MissionStatus::Active, MissionStatus::Failed);
    }

    #[test]
    fn test_mission_status_clone() {
        let status1 = MissionStatus::Active;
        let status2 = status1.clone();

        assert_eq!(status1, status2);
    }

    #[test]
    fn test_mission_status_copy() {
        let status1 = MissionStatus::Emergency;
        let status2 = status1; // Copy semantics

        assert_eq!(status1, status2);
    }

    #[test]
    fn test_all_mission_statuses() {
        let statuses = [
            MissionStatus::Idle,
            MissionStatus::Active,
            MissionStatus::Returning,
            MissionStatus::Emergency,
            MissionStatus::Failed,
        ];

        // Should be able to create and use all statuses
        for status in &statuses {
            let _cloned = status.clone();
        }
    }
}

#[cfg(test)]
mod swarm_error_tests {
    use super::*;

    #[test]
    fn test_swarm_error_equality() {
        assert_eq!(SwarmError::NetworkError, SwarmError::NetworkError);
        assert_eq!(SwarmError::CryptoError, SwarmError::CryptoError);
        assert_ne!(SwarmError::NetworkError, SwarmError::CryptoError);
    }

    #[test]
    fn test_swarm_error_display_network_error() {
        let error = SwarmError::NetworkError;
        let display = format!("{}", error);

        assert_eq!(display, "Network communication error");
    }

    #[test]
    fn test_swarm_error_display_crypto_error() {
        let error = SwarmError::CryptoError;
        let display = format!("{}", error);

        assert_eq!(display, "Cryptographic operation failed");
    }

    #[test]
    fn test_swarm_error_display_invalid_message() {
        let error = SwarmError::InvalidMessage;
        let display = format!("{}", error);

        assert_eq!(display, "Invalid message format");
    }

    #[test]
    fn test_swarm_error_display_authentication_failed() {
        let error = SwarmError::AuthenticationFailed;
        let display = format!("{}", error);

        assert_eq!(display, "Authentication failed");
    }

    #[test]
    fn test_swarm_error_display_consensus_error() {
        let error = SwarmError::ConsensusError;
        let display = format!("{}", error);

        assert_eq!(display, "Consensus not reached");
    }

    #[test]
    fn test_swarm_error_display_buffer_full() {
        let error = SwarmError::BufferFull;
        let display = format!("{}", error);

        assert_eq!(display, "Buffer overflow");
    }

    #[test]
    fn test_swarm_error_display_timeout() {
        let error = SwarmError::Timeout;
        let display = format!("{}", error);

        assert_eq!(display, "Operation timeout");
    }

    #[test]
    fn test_swarm_error_display_invalid_drone_id() {
        let error = SwarmError::InvalidDroneId;
        let display = format!("{}", error);

        assert_eq!(display, "Invalid drone ID");
    }

    #[test]
    fn test_swarm_error_display_permission_denied() {
        let error = SwarmError::PermissionDenied;
        let display = format!("{}", error);

        assert_eq!(display, "Operation not permitted");
    }

    #[test]
    fn test_swarm_error_display_resource_exhausted() {
        let error = SwarmError::ResourceExhausted;
        let display = format!("{}", error);

        assert_eq!(display, "Resource exhausted");
    }

    #[test]
    fn test_swarm_error_display_hardware_fault() {
        let error = SwarmError::HardwareFault;
        let display = format!("{}", error);

        assert_eq!(display, "Hardware fault detected");
    }

    #[test]
    fn test_swarm_error_display_config_error() {
        let error = SwarmError::ConfigError;
        let display = format!("{}", error);

        assert_eq!(display, "Configuration error");
    }

    #[test]
    fn test_swarm_error_display_swarm_size_exceeded() {
        let error = SwarmError::SwarmSizeExceeded;
        let display = format!("{}", error);

        assert_eq!(display, "Swarm size exceeded");
    }

    #[test]
    fn test_swarm_error_display_invalid_parameter() {
        let error = SwarmError::InvalidParameter;
        let display = format!("{}", error);

        assert_eq!(display, "Invalid parameter");
    }

    #[test]
    fn test_swarm_error_clone() {
        let error1 = SwarmError::NetworkError;
        let error2 = error1.clone();

        assert_eq!(error1, error2);
    }

    #[test]
    fn test_result_type_ok() {
        let result: Result<u32> = Ok(42);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42);
    }

    #[test]
    fn test_result_type_err() {
        let result: Result<u32> = Err(SwarmError::Timeout);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), SwarmError::Timeout);
    }
}

#[cfg(test)]
mod network_address_tests {
    use super::*;

    #[test]
    fn test_network_address_new() {
        let addr = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 1], 8080);

        assert_eq!(addr.port, 8080);
        assert_eq!(addr.addr[12], 127);
        assert_eq!(addr.addr[13], 0);
        assert_eq!(addr.addr[14], 0);
        assert_eq!(addr.addr[15], 1);
    }

    #[test]
    fn test_network_address_ipv6() {
        let addr =
            NetworkAddress::new([0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], 9000);

        assert_eq!(addr.port, 9000);
        assert_eq!(addr.addr[0], 0xfe);
        assert_eq!(addr.addr[1], 0x80);
    }

    #[test]
    fn test_network_address_equality() {
        let addr1 = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 1], 5000);
        let addr2 = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 1], 5000);
        let addr3 = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 2], 5000);

        assert_eq!(addr1, addr2);
        assert_ne!(addr1, addr3);
    }

    #[test]
    fn test_network_address_clone() {
        let addr1 = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 168, 1, 1], 3000);
        let addr2 = addr1.clone();

        assert_eq!(addr1, addr2);
    }

    #[test]
    fn test_network_address_const_new() {
        const ADDR: NetworkAddress = NetworkAddress::new([0; 16], 8080);
        assert_eq!(ADDR.port, 8080);
    }
}

#[cfg(test)]
mod security_level_tests {
    use super::*;

    #[test]
    fn test_security_level_ordering() {
        assert!(SecurityLevel::Public < SecurityLevel::Internal);
        assert!(SecurityLevel::Internal < SecurityLevel::Sensitive);
        assert!(SecurityLevel::Sensitive < SecurityLevel::Critical);
    }

    #[test]
    fn test_security_level_equality() {
        assert_eq!(SecurityLevel::Public, SecurityLevel::Public);
        assert_eq!(SecurityLevel::Critical, SecurityLevel::Critical);
        assert_ne!(SecurityLevel::Public, SecurityLevel::Critical);
    }

    #[test]
    fn test_security_level_numeric_values() {
        assert_eq!(SecurityLevel::Public as u8, 0);
        assert_eq!(SecurityLevel::Internal as u8, 1);
        assert_eq!(SecurityLevel::Sensitive as u8, 2);
        assert_eq!(SecurityLevel::Critical as u8, 3);
    }

    #[test]
    fn test_security_level_clone() {
        let level1 = SecurityLevel::Sensitive;
        let level2 = level1.clone();

        assert_eq!(level1, level2);
    }
}

#[cfg(test)]
mod task_priority_tests {
    use super::*;

    #[test]
    fn test_task_priority_ordering() {
        assert!(TaskPriority::Low < TaskPriority::Normal);
        assert!(TaskPriority::Normal < TaskPriority::High);
        assert!(TaskPriority::High < TaskPriority::Critical);
    }

    #[test]
    fn test_task_priority_equality() {
        assert_eq!(TaskPriority::Low, TaskPriority::Low);
        assert_eq!(TaskPriority::Critical, TaskPriority::Critical);
        assert_ne!(TaskPriority::Low, TaskPriority::High);
    }

    #[test]
    fn test_task_priority_numeric_values() {
        assert_eq!(TaskPriority::Low as u8, 0);
        assert_eq!(TaskPriority::Normal as u8, 1);
        assert_eq!(TaskPriority::High as u8, 2);
        assert_eq!(TaskPriority::Critical as u8, 3);
    }

    #[test]
    fn test_task_priority_clone() {
        let priority1 = TaskPriority::High;
        let priority2 = priority1.clone();

        assert_eq!(priority1, priority2);
    }
}

#[cfg(test)]
mod swarm_task_tests {
    use super::*;

    #[test]
    fn test_swarm_task_creation() {
        let mut assigned_drones: Vec<DroneId, 32> = Vec::new();
        assigned_drones.push(DroneId::new(1)).unwrap();
        assigned_drones.push(DroneId::new(2)).unwrap();

        let task = SwarmTask {
            task_id: 100,
            target: Position {
                x: 50.0,
                y: 60.0,
                z: 70.0,
            },
            priority: TaskPriority::High,
            assigned_drones,
            completed: false,
        };

        assert_eq!(task.task_id, 100);
        assert_eq!(task.priority, TaskPriority::High);
        assert!(!task.completed);
        assert_eq!(task.assigned_drones.len(), 2);
    }

    #[test]
    fn test_swarm_task_completion() {
        let task = SwarmTask {
            task_id: 200,
            target: Position {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            priority: TaskPriority::Normal,
            assigned_drones: Vec::new(),
            completed: true,
        };

        assert!(task.completed);
    }

    #[test]
    fn test_swarm_task_with_multiple_drones() {
        let mut assigned_drones: Vec<DroneId, 32> = Vec::new();

        for i in 1..=10 {
            assigned_drones.push(DroneId::new(i)).unwrap();
        }

        let task = SwarmTask {
            task_id: 300,
            target: Position {
                x: 100.0,
                y: 200.0,
                z: 300.0,
            },
            priority: TaskPriority::Critical,
            assigned_drones,
            completed: false,
        };

        assert_eq!(task.assigned_drones.len(), 10);
        assert_eq!(task.priority, TaskPriority::Critical);
    }

    #[test]
    fn test_swarm_task_clone() {
        let mut assigned_drones: Vec<DroneId, 32> = Vec::new();
        assigned_drones.push(DroneId::new(5)).unwrap();

        let task1 = SwarmTask {
            task_id: 400,
            target: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            priority: TaskPriority::Low,
            assigned_drones,
            completed: false,
        };

        let task2 = task1.clone();

        assert_eq!(task1.task_id, task2.task_id);
        assert_eq!(task1.priority, task2.priority);
        assert_eq!(task1.completed, task2.completed);
    }
}
