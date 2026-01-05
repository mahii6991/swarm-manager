//! Scalability Tests for Drone Swarm System
//!
//! Tests system behavior at scale (50, 75, 100+ drones)

use drone_swarm_system::types::{DroneId, DroneState, MissionStatus, Position, Velocity, NetworkAddress};
use drone_swarm_system::control::swarm::{SwarmController, Formation};
use drone_swarm_system::consensus::raft::{ConsensusEngine, ConsensusMessage, SwarmCommand, NodeState};
use drone_swarm_system::algorithms::pso::basic::{GlobalBestPSO, PSOOptions, Bounds as PSOBounds};
use drone_swarm_system::algorithms::gwo::{GWOOptimizer, GWOConfig, Bounds as GWOBounds};
use drone_swarm_system::control::mission::{Mission, Waypoint, MissionState, SurveyArea, SurveyPattern, generate_survey_mission, survey_to_mission, MAX_WAYPOINTS};
use drone_swarm_system::system::telemetry::{TelemetryMonitor, DroneStatus, HealthStatus};
use drone_swarm_system::network::mesh::{MeshNodeId, MeshMessage, MeshNeighbor, MessagePriority, EmergencyType, CommandAction, CommandTarget, MAX_MESH_NODES, MAX_HOPS};
use drone_swarm_system::network::core::MeshNetwork;
use std::collections::HashSet;
use std::time::Instant;

// Swarm size constants
const SMALL_SWARM: usize = 50;
const MEDIUM_SWARM: usize = 75;
const LARGE_SWARM: usize = 100;

// ============================================================================
// Drone Creation and Position Tests
// ============================================================================

mod drone_creation_tests {
    use super::*;

    #[test]
    fn test_create_50_drone_ids() {
        let start = Instant::now();
        let ids: Vec<DroneId> = (0..SMALL_SWARM as u64)
            .map(DroneId::new)
            .collect();

        let elapsed = start.elapsed();
        assert_eq!(ids.len(), SMALL_SWARM);
        assert!(elapsed.as_millis() < 100);

        // Verify unique IDs
        let unique: HashSet<u64> = ids.iter().map(|d| d.as_u64()).collect();
        assert_eq!(unique.len(), SMALL_SWARM);
    }

    #[test]
    fn test_create_75_drone_ids() {
        let ids: Vec<DroneId> = (0..MEDIUM_SWARM as u64)
            .map(DroneId::new)
            .collect();

        assert_eq!(ids.len(), MEDIUM_SWARM);
    }

    #[test]
    fn test_create_100_drone_ids() {
        let ids: Vec<DroneId> = (0..LARGE_SWARM as u64)
            .map(DroneId::new)
            .collect();

        assert_eq!(ids.len(), LARGE_SWARM);
    }

    #[test]
    fn test_position_grid_100_drones() {
        let grid_size = 10;
        let spacing = 10.0;

        let positions: Vec<Position> = (0..LARGE_SWARM)
            .map(|i| {
                let row = i / grid_size;
                let col = i % grid_size;
                Position {
                    x: col as f32 * spacing,
                    y: row as f32 * spacing,
                    z: 50.0,
                }
            })
            .collect();

        assert_eq!(positions.len(), LARGE_SWARM);

        // Verify grid spacing
        assert!((positions[0].x - 0.0).abs() < 0.01);
        assert!((positions[1].x - 10.0).abs() < 0.01);
        assert!((positions[10].y - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_distance_calculations_100_pairs() {
        let positions: Vec<Position> = (0..LARGE_SWARM)
            .map(|i| Position {
                x: i as f32 * 10.0,
                y: 0.0,
                z: 50.0,
            })
            .collect();

        let start = Instant::now();

        // Calculate all pairwise distances
        let mut total_distance = 0.0f32;
        let mut pair_count = 0;
        for i in 0..positions.len() {
            for j in (i + 1)..positions.len() {
                total_distance += positions[i].distance_to(&positions[j]);
                pair_count += 1;
            }
        }

        let elapsed = start.elapsed();
        let expected_pairs = LARGE_SWARM * (LARGE_SWARM - 1) / 2;

        assert_eq!(pair_count, expected_pairs);
        assert!(elapsed.as_millis() < 100);
        assert!(total_distance > 0.0);
    }
}

// ============================================================================
// Swarm Controller Tests
// ============================================================================

mod swarm_controller_tests {
    use super::*;

    #[test]
    fn test_swarm_controller_with_peers() {
        let pos = Position { x: 0.0, y: 0.0, z: 50.0 };
        let mut controller = SwarmController::new(DroneId::new(0), pos);

        // Add 49 peers (50 total including self, limit is 128 in FnvIndexMap)
        for i in 1..SMALL_SWARM as u64 {
            let peer_state = DroneState {
                id: DroneId::new(i),
                position: Position {
                    x: (i % 10) as f32 * 10.0,
                    y: (i / 10) as f32 * 10.0,
                    z: 50.0,
                },
                velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            };
            let _ = controller.update_peer_state(peer_state);
        }

        assert_eq!(controller.swarm_size(), SMALL_SWARM);
    }

    #[test]
    fn test_formation_computation_with_peers() {
        let pos = Position { x: 0.0, y: 0.0, z: 50.0 };
        let mut controller = SwarmController::new(DroneId::new(0), pos);

        // Add peers
        for i in 1..16u64 {
            let peer_state = DroneState {
                id: DroneId::new(i),
                position: Position {
                    x: (i % 4) as f32 * 10.0,
                    y: (i / 4) as f32 * 10.0,
                    z: 50.0,
                },
                velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            };
            let _ = controller.update_peer_state(peer_state);
        }

        // Test different formations
        controller.set_formation(Formation::Grid { spacing: 10 });
        let grid_pos = controller.compute_formation_position();
        assert!(grid_pos.z == 50.0);

        controller.set_formation(Formation::Circle { radius: 50 });
        let circle_pos = controller.compute_formation_position();
        assert!(circle_pos.z == 50.0);

        controller.set_formation(Formation::VFormation { spacing: 15 });
        let v_pos = controller.compute_formation_position();
        assert!(v_pos.z == 50.0);
    }

    #[test]
    fn test_centroid_calculation() {
        let pos = Position { x: 0.0, y: 0.0, z: 50.0 };
        let mut controller = SwarmController::new(DroneId::new(0), pos);

        // Add peers in a symmetric pattern
        for i in 1..5u64 {
            let angle = (i as f32) * std::f32::consts::PI / 2.0;
            let peer_state = DroneState {
                id: DroneId::new(i),
                position: Position {
                    x: 100.0 * angle.cos(),
                    y: 100.0 * angle.sin(),
                    z: 50.0,
                },
                velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            };
            let _ = controller.update_peer_state(peer_state);
        }

        let centroid = controller.swarm_centroid();
        // Centroid should be near origin due to symmetric placement
        assert!(centroid.x.abs() < 50.0);
        assert!(centroid.y.abs() < 50.0);
        assert!((centroid.z - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_collision_avoidance_computation() {
        let pos = Position { x: 50.0, y: 50.0, z: 50.0 };
        let mut controller = SwarmController::new(DroneId::new(0), pos);

        // Add nearby peers
        for i in 1..5u64 {
            let peer_state = DroneState {
                id: DroneId::new(i),
                position: Position {
                    x: 50.0 + (i as f32) * 5.0,
                    y: 50.0,
                    z: 50.0,
                },
                velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            };
            let _ = controller.update_peer_state(peer_state);
        }

        let avoidance = controller.compute_collision_avoidance();
        // Should have some repulsion velocity
        assert!(avoidance.vx != 0.0 || avoidance.vy != 0.0 || avoidance.vz != 0.0);
    }
}

// ============================================================================
// Network Tests
// ============================================================================

mod network_tests {
    use super::*;

    #[test]
    fn test_network_topology_50_nodes() {
        let nodes: Vec<NetworkAddress> = (0..SMALL_SWARM)
            .map(|i| {
                NetworkAddress::new(
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, i as u8],
                    5000 + i as u16,
                )
            })
            .collect();

        assert_eq!(nodes.len(), SMALL_SWARM);

        // Verify all addresses are unique
        let unique: HashSet<_> = nodes.iter().collect();
        assert_eq!(unique.len(), SMALL_SWARM);
    }

    #[test]
    fn test_mesh_network_creation() {
        let networks: Vec<MeshNetwork> = (0..32u64)
            .map(|i| MeshNetwork::new(DroneId::new(i)))
            .collect();

        assert_eq!(networks.len(), 32);

        for net in &networks {
            assert_eq!(net.neighbor_count(), 0);
        }
    }
}

// ============================================================================
// Mesh Protocol Tests
// ============================================================================

mod mesh_protocol_tests {
    use super::*;

    #[test]
    fn test_create_64_mesh_node_ids() {
        let node_ids: Vec<MeshNodeId> = (0..64u8)
            .map(MeshNodeId::new)
            .collect();

        assert_eq!(node_ids.len(), MAX_MESH_NODES);

        for (i, id) in node_ids.iter().enumerate() {
            assert_eq!(id.as_u8(), i as u8);
        }
    }

    #[test]
    fn test_mesh_heartbeat_messages() {
        let messages: Vec<MeshMessage> = (0..SMALL_SWARM.min(64) as u8)
            .map(|i| {
                MeshMessage::heartbeat(
                    MeshNodeId::new(i),
                    [i as f32 * 10.0, 0.0, 50.0],
                    100 - i,
                    5,
                    i as u64 * 1000,
                )
            })
            .collect();

        assert_eq!(messages.len(), SMALL_SWARM.min(64));

        // Verify all messages have valid checksums
        for msg in &messages {
            assert!(msg.verify_checksum());
            assert_eq!(msg.priority, MessagePriority::Normal);
        }
    }

    #[test]
    fn test_position_update_messages() {
        let messages: Vec<MeshMessage> = (0..64u8)
            .map(|i| {
                MeshMessage::position_update(
                    MeshNodeId::new(i),
                    [i as f32, i as f32, 50.0],
                    [1.0, 0.0, 0.0],
                    0.0,
                    i as u64 * 100,
                )
            })
            .collect();

        assert_eq!(messages.len(), 64);

        for msg in &messages {
            assert!(msg.verify_checksum());
            assert_eq!(msg.priority, MessagePriority::Normal);
        }
    }

    #[test]
    fn test_emergency_messages() {
        let emergency_types = [
            EmergencyType::LowBattery,
            EmergencyType::MotorFailure,
            EmergencyType::GpsLost,
            EmergencyType::CollisionWarning,
            EmergencyType::LinkLost,
            EmergencyType::Geofence,
            EmergencyType::Manual,
        ];

        let messages: Vec<MeshMessage> = emergency_types
            .iter()
            .enumerate()
            .map(|(i, et)| {
                MeshMessage::emergency(
                    MeshNodeId::new(i as u8),
                    *et,
                    [0.0, 0.0, 50.0],
                    i as u64 * 1000,
                )
            })
            .collect();

        for msg in &messages {
            assert_eq!(msg.priority, MessagePriority::Critical);
            assert!(msg.verify_checksum());
        }
    }

    #[test]
    fn test_command_messages() {
        let actions = [
            CommandAction::Arm,
            CommandAction::Disarm,
            CommandAction::Takeoff { altitude: 50.0 },
            CommandAction::Land,
            CommandAction::ReturnToLaunch,
            CommandAction::EmergencyStop,
        ];

        for (i, action) in actions.iter().enumerate() {
            let msg = MeshMessage::command(
                MeshNodeId::new(0),
                CommandTarget::Broadcast,
                action.clone(),
                i as u64 * 1000,
            );
            assert_eq!(msg.priority, MessagePriority::High);
            assert!(msg.verify_checksum());
        }
    }

    #[test]
    fn test_message_ttl_propagation() {
        let mut msg = MeshMessage::heartbeat(
            MeshNodeId::new(0),
            [0.0, 0.0, 50.0],
            100,
            5,
            0,
        );

        assert_eq!(msg.ttl, MAX_HOPS);

        // Decrement TTL MAX_HOPS times
        for _ in 0..MAX_HOPS {
            assert!(msg.decrement_ttl());
        }

        // TTL should now be exhausted
        assert!(!msg.decrement_ttl());
        assert_eq!(msg.ttl, 0);
    }

    #[test]
    fn test_mesh_neighbor_tracking() {
        let mut neighbors: Vec<MeshNeighbor> = (0..32u8)
            .map(|i| MeshNeighbor::new(MeshNodeId::new(i)))
            .collect();

        // Simulate heartbeat updates
        for (i, neighbor) in neighbors.iter_mut().enumerate() {
            neighbor.update_from_heartbeat(
                [i as f32 * 10.0, 0.0, 50.0],
                100 - (i as u8 % 50),
                -50,
                1000,
            );
        }

        // All neighbors should be active
        for neighbor in &neighbors {
            assert!(neighbor.is_active);
            assert_eq!(neighbor.hop_count, 1);
        }
    }
}

// ============================================================================
// Consensus Tests
// ============================================================================

mod consensus_tests {
    use super::*;

    #[test]
    fn test_consensus_engine_with_members() {
        let mut engine = ConsensusEngine::new(DroneId::new(0), 150);

        // Add swarm members
        for i in 0..SMALL_SWARM as u64 {
            let result = engine.add_member(DroneId::new(i));
            assert!(result.is_ok());
        }

        assert_eq!(engine.state(), NodeState::Follower);
    }

    #[test]
    fn test_consensus_with_100_members() {
        let mut engine = ConsensusEngine::new(DroneId::new(0), 150);

        // Add 100 members (max is 100 in swarm_members Vec)
        for i in 0..LARGE_SWARM as u64 {
            let result = engine.add_member(DroneId::new(i));
            assert!(result.is_ok());
        }

        assert_eq!(engine.state(), NodeState::Follower);
    }

    #[test]
    fn test_consensus_message_creation() {
        let messages: Vec<ConsensusMessage> = (0..SMALL_SWARM)
            .map(|i| ConsensusMessage::RequestVote {
                term: 1,
                candidate_id: DroneId::new(i as u64),
                last_log_index: 0,
                last_log_term: 0,
            })
            .collect();

        assert_eq!(messages.len(), SMALL_SWARM);
    }

    #[test]
    fn test_swarm_command_variants() {
        let commands = vec![
            SwarmCommand::AssignTask { drone: DroneId::new(1), task_id: 42 },
            SwarmCommand::AddDrone { drone: DroneId::new(2) },
            SwarmCommand::RemoveDrone { drone: DroneId::new(3) },
            SwarmCommand::EmergencyStop,
            SwarmCommand::ChangeFormation { formation_type: 1 },
        ];

        assert_eq!(commands.len(), 5);
    }

    #[test]
    fn test_consensus_quorum_calculation() {
        // For various cluster sizes, verify quorum is majority
        for cluster_size in [5, 7, 9, 11, 15, 21, 51] {
            let quorum = (cluster_size / 2) + 1;
            assert!(quorum > cluster_size / 2, "Quorum must be majority");
            assert!(quorum <= cluster_size, "Quorum cannot exceed cluster size");
        }
    }
}

// ============================================================================
// PSO Algorithm Tests
// ============================================================================

mod pso_tests {
    use super::*;

    #[test]
    fn test_pso_with_50_particles() {
        let bounds = PSOBounds::uniform(3, -100.0, 100.0).unwrap();
        let options = PSOOptions::default();

        let pso = GlobalBestPSO::new(SMALL_SWARM, 3, bounds, options);
        assert!(pso.is_ok());
    }

    #[test]
    fn test_pso_with_100_particles() {
        let bounds = PSOBounds::uniform(3, -100.0, 100.0).unwrap();
        let options = PSOOptions::default();

        let pso = GlobalBestPSO::new(LARGE_SWARM, 3, bounds, options);
        assert!(pso.is_ok());
    }

    #[test]
    fn test_pso_optimization() {
        let bounds = PSOBounds::uniform(2, -10.0, 10.0).unwrap();
        let options = PSOOptions::balanced();

        let mut pso = GlobalBestPSO::new(30, 2, bounds, options).unwrap();

        for _ in 0..20 {
            let _ = pso.step(|pos| pos.iter().map(|x| x * x).sum());
        }

        let best = pso.best_position();
        assert_eq!(best.len(), 2);
    }

    #[test]
    fn test_pso_throughput() {
        let bounds = PSOBounds::uniform(3, -100.0, 100.0).unwrap();
        let options = PSOOptions::default();

        let mut pso = GlobalBestPSO::new(50, 3, bounds, options).unwrap();

        let start = Instant::now();

        for _ in 0..100 {
            let _ = pso.step(|pos| pos.iter().map(|x| x * x).sum());
        }

        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 500, "PSO should complete 100 iterations in <500ms");
    }
}

// ============================================================================
// GWO Algorithm Tests
// ============================================================================

mod gwo_tests {
    use super::*;

    #[test]
    fn test_gwo_with_50_wolves() {
        let config = GWOConfig {
            num_wolves: SMALL_SWARM,
            dimensions: 3,
            max_iterations: 100,
            ..GWOConfig::default()
        };

        let bounds = GWOBounds::uniform(3, -100.0, 100.0).unwrap();
        let gwo = GWOOptimizer::new(config, bounds);
        assert!(gwo.is_ok());
    }

    #[test]
    fn test_gwo_with_50_wolves_max() {
        // MAX_WOLVES is 50
        let config = GWOConfig {
            num_wolves: 50,
            dimensions: 3,
            max_iterations: 50,
            ..GWOConfig::default()
        };

        let bounds = GWOBounds::uniform(3, -100.0, 100.0).unwrap();
        let gwo = GWOOptimizer::new(config, bounds);
        assert!(gwo.is_ok());
    }

    #[test]
    fn test_gwo_optimization() {
        let config = GWOConfig {
            num_wolves: 20,
            dimensions: 2,
            max_iterations: 30,
            ..GWOConfig::default()
        };

        let bounds = GWOBounds::uniform(2, -10.0, 10.0).unwrap();
        let mut gwo = GWOOptimizer::new(config, bounds).unwrap();

        let result = gwo.optimize(|pos| pos.iter().take(2).map(|x| x * x).sum());
        assert!(result.is_ok());
    }

    #[test]
    fn test_gwo_throughput() {
        let config = GWOConfig {
            num_wolves: 30,
            dimensions: 3,
            max_iterations: 100,
            ..GWOConfig::default()
        };

        let bounds = GWOBounds::uniform(3, -100.0, 100.0).unwrap();
        let mut gwo = GWOOptimizer::new(config, bounds).unwrap();

        let start = Instant::now();

        let _ = gwo.optimize(|pos| pos.iter().take(3).map(|x| x * x).sum());

        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 500, "GWO should complete optimization in <500ms");
    }
}

// ============================================================================
// Mission Planning Tests
// ============================================================================

mod mission_planning_tests {
    use super::*;

    #[test]
    fn test_mission_with_64_waypoints() {
        let mut mission = Mission::new();

        // Add maximum waypoints (64)
        for i in 0..MAX_WAYPOINTS {
            let wp = Waypoint::goto([i as f32 * 10.0, 0.0, -50.0]);
            assert!(mission.add_waypoint(wp));
        }

        assert_eq!(mission.waypoint_count(), MAX_WAYPOINTS);
    }

    #[test]
    fn test_mission_total_distance() {
        let mut mission = Mission::new();

        // Add 10 waypoints in a line
        for i in 0..10 {
            let wp = Waypoint::goto([i as f32 * 100.0, 0.0, -50.0]);
            mission.add_waypoint(wp);
        }

        let distance = mission.total_distance();
        // 9 segments of 100m each = 900m
        assert!((distance - 900.0).abs() < 1.0);
    }

    #[test]
    fn test_mission_execution() {
        let mut mission = Mission::new();

        for i in 0..5 {
            let wp = Waypoint::goto([i as f32 * 10.0, 0.0, -50.0]);
            mission.add_waypoint(wp);
        }

        mission.start();
        assert_eq!(mission.state(), MissionState::Active);
        assert_eq!(mission.current_index(), 0);

        // Advance through all waypoints
        for expected_idx in 1..5 {
            assert!(mission.advance());
            assert_eq!(mission.current_index(), expected_idx);
        }

        // Mission should complete
        assert!(!mission.advance());
        assert_eq!(mission.state(), MissionState::Completed);
    }

    #[test]
    fn test_mission_pause_resume() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -50.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -50.0]));

        mission.start();
        mission.pause();
        assert_eq!(mission.state(), MissionState::Paused);

        mission.resume();
        assert_eq!(mission.state(), MissionState::Active);
    }

    #[test]
    fn test_survey_area_default() {
        let area = SurveyArea::default();

        assert_eq!(area.vertices.len(), 4);
        assert!((area.altitude - 30.0).abs() < 0.01);
        assert!((area.spacing - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_lawnmower_pattern_generation() {
        let area = SurveyArea::default();
        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);

        assert!(!points.is_empty());

        // All points should be at survey altitude (negative in NED)
        for p in &points {
            assert!((p[2] + area.altitude).abs() < 0.01);
        }
    }

    #[test]
    fn test_expanding_square_pattern() {
        let area = SurveyArea::default();
        let points = generate_survey_mission(&area, SurveyPattern::ExpandingSquare);

        assert!(!points.is_empty());

        // First point should be at center
        let center_x = 50.0;
        let center_y = 50.0;

        assert!((points[0][0] - center_x).abs() < 1.0);
        assert!((points[0][1] - center_y).abs() < 1.0);
    }

    #[test]
    fn test_spiral_pattern() {
        let area = SurveyArea::default();
        let points = generate_survey_mission(&area, SurveyPattern::Spiral { inward: false });

        assert!(!points.is_empty());
    }

    #[test]
    fn test_survey_to_mission_conversion() {
        let area = SurveyArea::default();
        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);
        let mission = survey_to_mission(&points, false);

        assert!(mission.waypoint_count() > 0);
    }
}

// ============================================================================
// Telemetry Tests
// ============================================================================

mod telemetry_tests {
    use super::*;

    #[test]
    fn test_telemetry_monitor_creation() {
        let monitor = TelemetryMonitor::new();
        let stats = monitor.get_swarm_stats();

        assert_eq!(stats.active_drones, 0);
    }

    #[test]
    fn test_telemetry_with_32_drones() {
        let mut monitor = TelemetryMonitor::new();

        let start = Instant::now();

        // Add 32 drones (maximum capacity)
        for i in 0..32u8 {
            let mut status = DroneStatus::default();
            status.drone_id = i;
            status.position = [i as f32 * 10.0, 0.0, -50.0];
            status.battery_percent = 100 - (i % 50);
            status.rssi = -50 - (i as i8 % 30);
            status.gps_satellites = 12;
            status.armed = true;
            status.health = HealthStatus::Healthy;

            monitor.update_drone(i, status);
        }

        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 100);

        let stats = monitor.get_swarm_stats();
        assert!(stats.active_drones > 0);
    }

    #[test]
    fn test_drone_status_creation() {
        let statuses: Vec<DroneStatus> = (0..LARGE_SWARM)
            .map(|i| {
                let mut status = DroneStatus::default();
                status.drone_id = i as u8;
                status.position = [i as f32, i as f32, -50.0];
                status.battery_percent = 100 - ((i as u8) % 100);
                status.rssi = -50;
                status.gps_satellites = 12;
                status.armed = true;
                status
            })
            .collect();

        assert_eq!(statuses.len(), LARGE_SWARM);
    }
}

// ============================================================================
// Memory and Performance Tests
// ============================================================================

mod memory_tests {
    use super::*;
    use std::mem::size_of;

    #[test]
    fn test_memory_size_drone_id() {
        assert_eq!(size_of::<DroneId>(), 8);
    }

    #[test]
    fn test_memory_size_position() {
        assert_eq!(size_of::<Position>(), 12); // 3 x f32
    }

    #[test]
    fn test_memory_size_velocity() {
        assert_eq!(size_of::<Velocity>(), 12); // 3 x f32
    }

    #[test]
    fn test_memory_size_mesh_node_id() {
        assert_eq!(size_of::<MeshNodeId>(), 1); // u8
    }

    #[test]
    fn test_memory_size_message_priority() {
        assert_eq!(size_of::<MessagePriority>(), 1);
    }

    #[test]
    fn test_memory_estimation_100_drones() {
        let drone_id_size = size_of::<DroneId>();
        let position_size = size_of::<Position>();
        let velocity_size = size_of::<Velocity>();

        // Estimate memory for 100 drone states
        let per_drone = drone_id_size + position_size + velocity_size + 32; // +32 for overhead
        let total = per_drone * LARGE_SWARM;

        // Should be reasonable (< 100KB for basic state)
        assert!(total < 100_000);

        println!("Estimated memory for {} drones: {} bytes ({} KB)",
            LARGE_SWARM, total, total / 1024);
    }
}

// ============================================================================
// Throughput Tests
// ============================================================================

mod throughput_tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn test_position_update_throughput() {
        let positions: Vec<Position> = (0..LARGE_SWARM)
            .map(|i| Position {
                x: i as f32,
                y: i as f32,
                z: 50.0,
            })
            .collect();

        let start = Instant::now();

        // Simulate 1000 position updates
        for _ in 0..1000 {
            for i in 0..LARGE_SWARM {
                let _updated = Position {
                    x: positions[i].x + 0.1,
                    y: positions[i].y + 0.1,
                    z: positions[i].z,
                };
            }
        }

        let elapsed = start.elapsed();
        let updates_per_sec = (1000 * LARGE_SWARM) as f64 / elapsed.as_secs_f64();

        // Should handle at least 100k updates per second
        assert!(updates_per_sec > 100_000.0);

        println!("Position updates per second: {:.0}", updates_per_sec);
    }

    #[test]
    fn test_distance_calculation_throughput() {
        let positions: Vec<Position> = (0..LARGE_SWARM)
            .map(|i| Position {
                x: i as f32 * 10.0,
                y: i as f32 * 5.0,
                z: 50.0,
            })
            .collect();

        let start = Instant::now();

        // Calculate all pairwise distances
        let mut total_distance = 0.0f32;
        for i in 0..LARGE_SWARM {
            for j in (i + 1)..LARGE_SWARM {
                total_distance += positions[i].distance_to(&positions[j]);
            }
        }

        let elapsed = start.elapsed();
        let pairs = (LARGE_SWARM * (LARGE_SWARM - 1)) / 2;
        let calcs_per_sec = pairs as f64 / elapsed.as_secs_f64();

        assert!(total_distance > 0.0);

        // Should handle millions of distance calculations per second
        assert!(calcs_per_sec > 100_000.0);

        println!("Distance calculations per second: {:.0}", calcs_per_sec);
    }

    #[test]
    fn test_message_creation_throughput() {
        let start = Instant::now();

        let mut messages = Vec::with_capacity(10000);

        for i in 0..10000u32 {
            let msg = MeshMessage::heartbeat(
                MeshNodeId::new((i % 64) as u8),
                [(i % 100) as f32, (i % 100) as f32, 50.0],
                100,
                5,
                i as u64,
            );
            messages.push(msg);
        }

        let elapsed = start.elapsed();
        let msgs_per_sec = 10000.0 / elapsed.as_secs_f64();

        assert_eq!(messages.len(), 10000);

        // Should create at least 100k messages per second
        assert!(msgs_per_sec > 100_000.0);

        println!("Messages created per second: {:.0}", msgs_per_sec);
    }
}

// ============================================================================
// Stress Tests
// ============================================================================

mod stress_tests {
    use super::*;

    #[test]
    fn test_rapid_formation_changes() {
        let pos = Position { x: 0.0, y: 0.0, z: 50.0 };
        let mut controller = SwarmController::new(DroneId::new(0), pos);

        // Rapidly change formations
        let formations = [
            Formation::Random,
            Formation::Grid { spacing: 10 },
            Formation::Line { spacing: 10 },
            Formation::Circle { radius: 50 },
            Formation::VFormation { spacing: 15 },
        ];

        let start = Instant::now();

        for _ in 0..1000 {
            for formation in &formations {
                controller.set_formation(*formation);
                let _ = controller.compute_formation_position();
            }
        }

        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 500, "5000 formation changes should complete in <500ms");
    }

    #[test]
    fn test_mission_manipulation() {
        let mut mission = Mission::new();

        // Add waypoints
        for i in 0..50 {
            let wp = Waypoint::goto([i as f32, 0.0, -50.0]);
            mission.add_waypoint(wp);
        }

        assert_eq!(mission.waypoint_count(), 50);

        // Remove some waypoints
        for _ in 0..10 {
            mission.remove_waypoint(0);
        }

        assert_eq!(mission.waypoint_count(), 40);

        // Start and jump around
        mission.start();

        for i in 0..20 {
            mission.jump_to(i % mission.waypoint_count());
        }
    }

    #[test]
    fn test_neighbor_mesh_simulation() {
        // Simulate neighbor discovery and updates
        let mut neighbors: Vec<MeshNeighbor> = (0..64u8)
            .map(|i| MeshNeighbor::new(MeshNodeId::new(i)))
            .collect();

        // Simulate heartbeat updates
        for cycle in 0..100u64 {
            for (i, neighbor) in neighbors.iter_mut().enumerate() {
                neighbor.update_from_heartbeat(
                    [i as f32, cycle as f32, 50.0],
                    100 - ((i as u8) % 50),
                    -50,
                    cycle * 1000,
                );
            }
        }

        // All neighbors should be active
        for neighbor in &neighbors {
            assert!(neighbor.is_active);
        }
    }

    #[test]
    fn test_collision_detection_stress() {
        let positions: Vec<Position> = (0..LARGE_SWARM)
            .map(|i| Position {
                x: (i % 10) as f32 * 10.0,
                y: (i / 10) as f32 * 10.0,
                z: 50.0,
            })
            .collect();

        let min_separation = 5.0;

        let start = Instant::now();

        // Run collision detection multiple times
        for _ in 0..100 {
            let mut collision_count = 0;
            for i in 0..positions.len() {
                for j in (i + 1)..positions.len() {
                    let dist = positions[i].distance_to(&positions[j]);
                    if dist < min_separation {
                        collision_count += 1;
                    }
                }
            }
            assert_eq!(collision_count, 0, "Grid should have no collisions");
        }

        let elapsed = start.elapsed();
        assert!(elapsed.as_millis() < 1000, "100 collision checks should complete in <1s");
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

fn create_grid_positions(count: usize, spacing: f32) -> Vec<Position> {
    let cols = (count as f32).sqrt().ceil() as usize;
    let mut positions = Vec::with_capacity(count);

    for i in 0..count {
        let row = i / cols;
        let col = i % cols;
        positions.push(Position {
            x: col as f32 * spacing,
            y: row as f32 * spacing,
            z: 50.0,
        });
    }

    positions
}

fn verify_minimum_separation(positions: &[Position], min_sep: f32) {
    for i in 0..positions.len() {
        for j in (i + 1)..positions.len() {
            let dist = positions[i].distance_to(&positions[j]);
            assert!(
                dist >= min_sep - 0.001,
                "Positions {} and {} are too close: {:.2}m (min: {:.2}m)",
                i, j, dist, min_sep
            );
        }
    }
}

#[test]
fn test_grid_helper() {
    let positions = create_grid_positions(100, 10.0);
    assert_eq!(positions.len(), 100);
    verify_minimum_separation(&positions, 8.0);
}
