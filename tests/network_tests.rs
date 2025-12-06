//! Comprehensive tests for mesh networking
//!
//! Tests network message handling, neighbor discovery, routing, and link quality

use drone_swarm_system::network::*;
use drone_swarm_system::types::*;
use heapless::Vec;

#[cfg(test)]
mod neighbor_tests {
    use super::*;

    fn create_test_neighbor() -> Neighbor {
        Neighbor {
            id: DroneId::new(1),
            address: NetworkAddress::new([0; 16], 8080),
            position: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            link_quality: 0.8,
            last_seen: 1000,
            rtt_ms: 50,
        }
    }

    #[test]
    fn test_neighbor_is_alive_recently_seen() {
        let neighbor = create_test_neighbor();
        let current_time = 1500; // 500ms after last_seen
        let timeout_ms = 1000;

        assert!(neighbor.is_alive(current_time, timeout_ms));
    }

    #[test]
    fn test_neighbor_is_alive_timeout() {
        let neighbor = create_test_neighbor();
        let current_time = 3000; // 2000ms after last_seen
        let timeout_ms = 1000;

        assert!(!neighbor.is_alive(current_time, timeout_ms));
    }

    #[test]
    fn test_neighbor_is_alive_edge_case() {
        let neighbor = create_test_neighbor();
        let current_time = 2000; // Exactly timeout
        let timeout_ms = 1000;

        assert!(!neighbor.is_alive(current_time, timeout_ms));
    }

    #[test]
    fn test_neighbor_update_quality_success() {
        let mut neighbor = create_test_neighbor();
        let initial_quality = neighbor.link_quality;

        neighbor.update_quality(true);

        // Quality should increase towards 1.0
        assert!(neighbor.link_quality > initial_quality);
        assert!(neighbor.link_quality <= 1.0);
    }

    #[test]
    fn test_neighbor_update_quality_failure() {
        let mut neighbor = create_test_neighbor();
        let initial_quality = neighbor.link_quality;

        neighbor.update_quality(false);

        // Quality should decrease towards 0.0
        assert!(neighbor.link_quality < initial_quality);
        assert!(neighbor.link_quality >= 0.0);
    }

    #[test]
    fn test_neighbor_update_quality_multiple_successes() {
        let mut neighbor = create_test_neighbor();
        neighbor.link_quality = 0.5;

        for _ in 0..10 {
            neighbor.update_quality(true);
        }

        // After many successes, quality should be near 1.0
        assert!(neighbor.link_quality > 0.9);
    }

    #[test]
    fn test_neighbor_update_quality_multiple_failures() {
        let mut neighbor = create_test_neighbor();
        neighbor.link_quality = 0.5;

        for _ in 0..10 {
            neighbor.update_quality(false);
        }

        // After many failures, quality should be near 0.0
        assert!(neighbor.link_quality < 0.1);
    }

    #[test]
    fn test_neighbor_update_quality_alternating() {
        let mut neighbor = create_test_neighbor();
        neighbor.link_quality = 0.5;

        for i in 0..20 {
            neighbor.update_quality(i % 2 == 0);
        }

        // Should stabilize somewhere in the middle
        assert!(neighbor.link_quality > 0.3);
        assert!(neighbor.link_quality < 0.7);
    }
}

#[cfg(test)]
mod network_message_tests {
    use super::*;

    #[test]
    fn test_network_message_hello() {
        let msg = NetworkMessage::Hello {
            sender: DroneId::new(1),
            position: Position {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            sequence: 42,
        };

        match msg {
            NetworkMessage::Hello {
                sender,
                position,
                sequence,
            } => {
                assert_eq!(sender, DroneId::new(1));
                assert_eq!(position.x, 1.0);
                assert_eq!(sequence, 42);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_heartbeat() {
        let msg = NetworkMessage::Heartbeat {
            sender: DroneId::new(2),
            timestamp: 1234567890,
        };

        match msg {
            NetworkMessage::Heartbeat { sender, timestamp } => {
                assert_eq!(sender, DroneId::new(2));
                assert_eq!(timestamp, 1234567890);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_data() {
        let mut payload: Vec<u8, 1024> = Vec::new();
        payload.push(1).unwrap();
        payload.push(2).unwrap();
        payload.push(3).unwrap();

        let msg = NetworkMessage::Data {
            source: DroneId::new(1),
            destination: DroneId::new(2),
            payload: payload.clone(),
            hop_count: 0,
        };

        match msg {
            NetworkMessage::Data {
                source,
                destination,
                payload: p,
                hop_count,
            } => {
                assert_eq!(source, DroneId::new(1));
                assert_eq!(destination, DroneId::new(2));
                assert_eq!(p.len(), 3);
                assert_eq!(hop_count, 0);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_route_request() {
        let msg = NetworkMessage::RouteRequest {
            source: DroneId::new(1),
            destination: DroneId::new(3),
            sequence: 100,
        };

        match msg {
            NetworkMessage::RouteRequest {
                source,
                destination,
                sequence,
            } => {
                assert_eq!(source, DroneId::new(1));
                assert_eq!(destination, DroneId::new(3));
                assert_eq!(sequence, 100);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_route_reply() {
        let msg = NetworkMessage::RouteReply {
            source: DroneId::new(1),
            destination: DroneId::new(3),
            next_hop: DroneId::new(2),
            hop_count: 2,
        };

        match msg {
            NetworkMessage::RouteReply {
                source,
                destination,
                next_hop,
                hop_count,
            } => {
                assert_eq!(source, DroneId::new(1));
                assert_eq!(destination, DroneId::new(3));
                assert_eq!(next_hop, DroneId::new(2));
                assert_eq!(hop_count, 2);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_link_state_update() {
        let mut neighbors: Vec<(DroneId, f32), MAX_NEIGHBORS> = Vec::new();
        neighbors.push((DroneId::new(2), 0.9)).unwrap();
        neighbors.push((DroneId::new(3), 0.8)).unwrap();

        let msg = NetworkMessage::LinkStateUpdate {
            sender: DroneId::new(1),
            neighbors: neighbors.clone(),
            sequence: 50,
        };

        match msg {
            NetworkMessage::LinkStateUpdate {
                sender,
                neighbors: n,
                sequence,
            } => {
                assert_eq!(sender, DroneId::new(1));
                assert_eq!(n.len(), 2);
                assert_eq!(sequence, 50);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_network_message_clone() {
        let msg1 = NetworkMessage::Heartbeat {
            sender: DroneId::new(5),
            timestamp: 999,
        };

        let msg2 = msg1.clone();

        match (msg1, msg2) {
            (
                NetworkMessage::Heartbeat {
                    sender: s1,
                    timestamp: t1,
                },
                NetworkMessage::Heartbeat {
                    sender: s2,
                    timestamp: t2,
                },
            ) => {
                assert_eq!(s1, s2);
                assert_eq!(t1, t2);
            }
            _ => panic!("Clone failed"),
        }
    }
}

#[cfg(test)]
mod network_stats_tests {
    use super::*;

    #[test]
    fn test_network_stats_default() {
        let stats = NetworkStats::default();

        assert_eq!(stats.messages_sent, 0);
        assert_eq!(stats.messages_received, 0);
        assert_eq!(stats.messages_dropped, 0);
        assert_eq!(stats.avg_rtt_ms, 0);
    }

    #[test]
    fn test_network_stats_clone() {
        let stats1 = NetworkStats {
            messages_sent: 100,
            messages_received: 95,
            messages_dropped: 5,
            avg_rtt_ms: 50,
        };

        let stats2 = stats1.clone();

        assert_eq!(stats1.messages_sent, stats2.messages_sent);
        assert_eq!(stats1.messages_received, stats2.messages_received);
        assert_eq!(stats1.messages_dropped, stats2.messages_dropped);
        assert_eq!(stats1.avg_rtt_ms, stats2.avg_rtt_ms);
    }

    #[test]
    fn test_network_stats_copy() {
        let stats1 = NetworkStats {
            messages_sent: 200,
            messages_received: 190,
            messages_dropped: 10,
            avg_rtt_ms: 75,
        };

        let stats2 = stats1; // Copy semantics

        assert_eq!(stats1.messages_sent, stats2.messages_sent);
    }
}

#[cfg(test)]
mod mesh_network_tests {
    use super::*;

    #[test]
    fn test_mesh_network_new() {
        let drone_id = DroneId::new(42);
        let network = MeshNetwork::new(drone_id);

        assert_eq!(network.neighbor_count(), 0);
        let stats = network.statistics();
        assert_eq!(stats.messages_sent, 0);
        assert_eq!(stats.messages_received, 0);
    }

    #[test]
    fn test_mesh_network_neighbor_count_empty() {
        let network = MeshNetwork::new(DroneId::new(1));
        assert_eq!(network.neighbor_count(), 0);
    }

    #[test]
    fn test_mesh_network_statistics_initial() {
        let network = MeshNetwork::new(DroneId::new(1));
        let stats = network.statistics();

        assert_eq!(stats.messages_sent, 0);
        assert_eq!(stats.messages_received, 0);
        assert_eq!(stats.messages_dropped, 0);
        assert_eq!(stats.avg_rtt_ms, 0);
    }

    #[test]
    fn test_mesh_network_process_hello() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        let msg = NetworkMessage::Hello {
            sender: DroneId::new(2),
            position: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            sequence: 1,
        };

        let addr = NetworkAddress::new([0; 16], 8080);
        let result = network.process_message(msg, addr);

        assert!(result.is_ok());
        assert_eq!(network.neighbor_count(), 1);
    }

    #[test]
    fn test_mesh_network_process_heartbeat() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        // First add the neighbor via Hello
        let hello = NetworkMessage::Hello {
            sender: DroneId::new(2),
            position: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            sequence: 1,
        };
        let addr = NetworkAddress::new([0; 16], 8080);
        network.process_message(hello, addr).unwrap();

        // Now send heartbeat
        let heartbeat = NetworkMessage::Heartbeat {
            sender: DroneId::new(2),
            timestamp: 1000,
        };

        let result = network.process_message(heartbeat, addr);
        assert!(result.is_ok());
    }

    #[test]
    fn test_mesh_network_process_data_for_us() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        let mut payload: Vec<u8, 1024> = Vec::new();
        payload.push(42).unwrap();
        payload.push(43).unwrap();

        let msg = NetworkMessage::Data {
            source: DroneId::new(2),
            destination: DroneId::new(1), // For us!
            payload: payload.clone(),
            hop_count: 0,
        };

        let addr = NetworkAddress::new([0; 16], 8080);
        let result = network.process_message(msg, addr);

        assert!(result.is_ok());
        let received_payload = result.unwrap();
        assert!(received_payload.is_some());
        assert_eq!(received_payload.unwrap().len(), 2);

        // Check stats
        let stats = network.statistics();
        assert_eq!(stats.messages_received, 1);
    }

    #[test]
    fn test_mesh_network_send_message_no_route() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        let mut payload: Vec<u8, 1024> = Vec::new();
        payload.push(1).unwrap();
        payload.push(2).unwrap();

        let result = network.send_message(DroneId::new(2), payload);

        // Should succeed (queues and initiates route discovery)
        assert!(result.is_ok());
    }

    #[test]
    #[cfg(not(feature = "hardware"))]
    fn test_mesh_network_broadcast_hello() {
        let mut network = MeshNetwork::new(DroneId::new(1));
        let position = Position {
            x: 5.0,
            y: 10.0,
            z: 15.0,
        };

        let result = network.broadcast_hello(position);

        // In simulation mode, this should succeed
        assert!(result.is_ok());
    }

    #[test]
    #[cfg(feature = "hardware")]
    #[should_panic(expected = "Hardware broadcast not yet implemented")]
    fn test_mesh_network_broadcast_hello_hardware() {
        let mut network = MeshNetwork::new(DroneId::new(1));
        let position = Position {
            x: 5.0,
            y: 10.0,
            z: 15.0,
        };

        let _ = network.broadcast_hello(position);
    }

    #[test]
    #[cfg(not(feature = "hardware"))]
    fn test_mesh_network_send_heartbeat() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        let result = network.send_heartbeat();

        // In simulation mode, this should succeed
        assert!(result.is_ok());
    }

    #[test]
    #[cfg(feature = "hardware")]
    #[should_panic(expected = "Hardware heartbeat transmission not yet implemented")]
    fn test_mesh_network_send_heartbeat_hardware() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        let _ = network.send_heartbeat();
    }

    #[test]
    fn test_mesh_network_prune_neighbors_no_timeout() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        // Add a neighbor
        let hello = NetworkMessage::Hello {
            sender: DroneId::new(2),
            position: Position {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
            sequence: 1,
        };
        let addr = NetworkAddress::new([0; 16], 8080);
        network.process_message(hello, addr).unwrap();

        assert_eq!(network.neighbor_count(), 1);

        // Prune with very long timeout - neighbor should remain
        network.prune_neighbors(100000);

        assert_eq!(network.neighbor_count(), 1);
    }

    #[test]
    fn test_mesh_network_neighbors_iterator_empty() {
        let network = MeshNetwork::new(DroneId::new(1));

        let count = network.neighbors().count();
        assert_eq!(count, 0);
    }

    #[test]
    fn test_mesh_network_neighbors_iterator() {
        let mut network = MeshNetwork::new(DroneId::new(1));

        // Add two neighbors
        for i in 2..=3 {
            let hello = NetworkMessage::Hello {
                sender: DroneId::new(i),
                position: Position {
                    x: 10.0 * i as f32,
                    y: 20.0,
                    z: 30.0,
                },
                sequence: 1,
            };
            let addr = NetworkAddress::new([0; 16], 8080);
            network.process_message(hello, addr).unwrap();
        }

        let count = network.neighbors().count();
        assert_eq!(count, 2);
    }
}

#[cfg(test)]
mod constants_tests {
    use super::*;

    #[test]
    fn test_max_neighbors() {
        assert_eq!(MAX_NEIGHBORS, 32);
    }

    #[test]
    fn test_max_routes() {
        assert_eq!(MAX_ROUTES, 128);
    }

    #[test]
    fn test_max_network_hops() {
        assert_eq!(MAX_NETWORK_HOPS, 15);
    }
}
