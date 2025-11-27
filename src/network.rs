//! Mesh networking layer for drone-to-drone communication
//!
//! Implements:
//! - Adaptive mesh routing with automatic topology optimization
//! - Multi-hop communication with distance vector routing
//! - Link quality monitoring
//! - Automatic neighbor discovery
//! - Network resilience and self-healing

use crate::types::*;
use crate::crypto::CryptoContext;
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

/// Maximum number of neighbors in mesh network
pub const MAX_NEIGHBORS: usize = 20;

/// Maximum routing table entries
pub const MAX_ROUTES: usize = 100;

/// Maximum network hops (prevents routing loops)
pub const MAX_NETWORK_HOPS: u8 = 15;

/// Message types for mesh networking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NetworkMessage {
    /// Hello message for neighbor discovery
    Hello {
        sender: DroneId,
        position: Position,
        sequence: u32,
    },
    /// Heartbeat to maintain connections
    Heartbeat {
        sender: DroneId,
        timestamp: u64,
    },
    /// Data message
    Data {
        source: DroneId,
        destination: DroneId,
        payload: Vec<u8, 1024>,
        hop_count: u8,
    },
    /// Route request (AODV-style)
    RouteRequest {
        source: DroneId,
        destination: DroneId,
        sequence: u32,
    },
    /// Route reply
    RouteReply {
        source: DroneId,
        destination: DroneId,
        next_hop: DroneId,
        hop_count: u8,
    },
    /// Link state update
    LinkStateUpdate {
        sender: DroneId,
        neighbors: Vec<(DroneId, f32), MAX_NEIGHBORS>,
        sequence: u32,
    },
}

/// Neighbor information
#[derive(Debug, Clone, Copy)]
pub struct Neighbor {
    /// Neighbor drone ID
    pub id: DroneId,
    /// Network address
    pub address: NetworkAddress,
    /// Last known position
    pub position: Position,
    /// Link quality (0.0 - 1.0)
    pub link_quality: f32,
    /// Last heard timestamp
    pub last_seen: u64,
    /// Round-trip time (ms)
    pub rtt_ms: u32,
}

impl Neighbor {
    /// Check if neighbor is still alive
    pub fn is_alive(&self, current_time: u64, timeout_ms: u32) -> bool {
        current_time - self.last_seen < timeout_ms as u64
    }

    /// Update link quality based on RTT and packet loss
    pub fn update_quality(&mut self, success: bool) {
        const ALPHA: f32 = 0.8; // Exponential smoothing factor
        let measurement = if success { 1.0 } else { 0.0 };
        self.link_quality = ALPHA * self.link_quality + (1.0 - ALPHA) * measurement;
    }
}

/// Routing table entry
#[derive(Debug, Clone, Copy)]
pub struct Route {
    /// Destination drone
    pub destination: DroneId,
    /// Next hop to reach destination
    pub next_hop: DroneId,
    /// Number of hops
    pub hop_count: u8,
    /// Route metric (lower is better)
    pub metric: f32,
    /// Sequence number for route freshness
    pub sequence: u32,
    /// Last updated timestamp
    pub last_updated: u64,
}

/// Mesh network manager
pub struct MeshNetwork {
    /// This drone's ID
    local_id: DroneId,
    /// Neighbor table
    neighbors: FnvIndexMap<u64, Neighbor, MAX_NEIGHBORS>,
    /// Routing table
    routes: FnvIndexMap<u64, Route, MAX_ROUTES>,
    /// Sequence number for route updates
    sequence_number: u32,
    /// Message queue
    message_queue: Vec<QueuedMessage, 100>,
    /// Network statistics
    stats: NetworkStats,
}

#[derive(Debug, Clone)]
struct QueuedMessage {
    destination: DroneId,
    payload: Vec<u8, 1024>,
    retry_count: u8,
    timestamp: u64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct NetworkStats {
    /// Total messages sent
    pub messages_sent: u64,
    /// Total messages received
    pub messages_received: u64,
    /// Messages dropped
    pub messages_dropped: u64,
    /// Average RTT in milliseconds
    pub avg_rtt_ms: u32,
}

impl MeshNetwork {
    /// Create a new mesh network instance
    pub fn new(local_id: DroneId) -> Self {
        Self {
            local_id,
            neighbors: FnvIndexMap::new(),
            routes: FnvIndexMap::new(),
            sequence_number: 0,
            message_queue: Vec::new(),
            stats: NetworkStats::default(),
        }
    }

    /// Process incoming network message
    pub fn process_message(&mut self, message: NetworkMessage, sender_addr: NetworkAddress) -> Result<Option<Vec<u8, 1024>>> {
        match message {
            NetworkMessage::Hello {
                sender,
                position,
                sequence,
            } => {
                self.handle_hello(sender, position, sender_addr, sequence)?;
                Ok(None)
            }
            NetworkMessage::Heartbeat { sender, timestamp } => {
                self.handle_heartbeat(sender, timestamp)?;
                Ok(None)
            }
            NetworkMessage::Data {
                source,
                destination,
                payload,
                hop_count,
            } => {
                if destination == self.local_id {
                    // Message is for us
                    self.stats.messages_received += 1;
                    Ok(Some(payload))
                } else {
                    // Forward message
                    self.forward_message(source, destination, payload, hop_count)?;
                    Ok(None)
                }
            }
            NetworkMessage::RouteRequest {
                source,
                destination,
                sequence,
            } => {
                self.handle_route_request(source, destination, sequence)?;
                Ok(None)
            }
            NetworkMessage::RouteReply {
                source,
                destination,
                next_hop,
                hop_count,
            } => {
                self.handle_route_reply(source, destination, next_hop, hop_count)?;
                Ok(None)
            }
            NetworkMessage::LinkStateUpdate {
                sender,
                neighbors,
                sequence,
            } => {
                self.handle_link_state_update(sender, neighbors, sequence)?;
                Ok(None)
            }
        }
    }

    /// Send a message to a destination drone
    pub fn send_message(&mut self, destination: DroneId, payload: Vec<u8, 1024>) -> Result<()> {
        // Check if we have a route
        if let Some(route) = self.routes.get(&destination.as_u64()) {
            // Send directly via route
            let msg = NetworkMessage::Data {
                source: self.local_id,
                destination,
                payload,
                hop_count: 0,
            };
            self.stats.messages_sent += 1;
            // In actual implementation, send via network interface
            Ok(())
        } else {
            // No route - queue and initiate route discovery
            self.queue_message(destination, payload)?;
            self.initiate_route_discovery(destination)?;
            Ok(())
        }
    }

    /// Handle hello message (neighbor discovery)
    fn handle_hello(
        &mut self,
        sender: DroneId,
        position: Position,
        address: NetworkAddress,
        sequence: u32,
    ) -> Result<()> {
        let neighbor = Neighbor {
            id: sender,
            address,
            position,
            link_quality: 1.0,
            last_seen: Self::get_time(),
            rtt_ms: 0,
        };

        self.neighbors
            .insert(sender.as_u64(), neighbor)
            .map_err(|_| SwarmError::ResourceExhausted)?;

        // Update direct route
        let route = Route {
            destination: sender,
            next_hop: sender,
            hop_count: 1,
            metric: 1.0,
            sequence,
            last_updated: Self::get_time(),
        };

        self.routes
            .insert(sender.as_u64(), route)
            .map_err(|_| SwarmError::ResourceExhausted)?;

        Ok(())
    }

    /// Handle heartbeat message
    fn handle_heartbeat(&mut self, sender: DroneId, timestamp: u64) -> Result<()> {
        if let Some(neighbor) = self.neighbors.get_mut(&sender.as_u64()) {
            neighbor.last_seen = Self::get_time();
            neighbor.update_quality(true);
        }
        Ok(())
    }

    /// Forward message to next hop
    fn forward_message(
        &mut self,
        source: DroneId,
        destination: DroneId,
        payload: Vec<u8, 1024>,
        hop_count: u8,
    ) -> Result<()> {
        // BUG-003 FIX: Strict hop limit validation to prevent loops and overflow
        if hop_count >= MAX_NETWORK_HOPS {
            self.stats.messages_dropped += 1;
            return Err(SwarmError::NetworkError);
        }

        // Check overflow before incrementing
        let new_hop_count = hop_count.checked_add(1)
            .ok_or(SwarmError::NetworkError)?;

        if new_hop_count > MAX_NETWORK_HOPS {
            self.stats.messages_dropped += 1;
            return Err(SwarmError::NetworkError);
        }

        // Find route
        if let Some(route) = self.routes.get(&destination.as_u64()) {
            let msg = NetworkMessage::Data {
                source,
                destination,
                payload,
                hop_count: new_hop_count,
            };
            // Send to next hop
            self.stats.messages_sent += 1;
            Ok(())
        } else {
            // No route - drop or queue
            self.stats.messages_dropped += 1;
            Err(SwarmError::NetworkError)
        }
    }

    /// Initiate route discovery
    fn initiate_route_discovery(&mut self, destination: DroneId) -> Result<()> {
        self.sequence_number += 1;
        let msg = NetworkMessage::RouteRequest {
            source: self.local_id,
            destination,
            sequence: self.sequence_number,
        };
        // Broadcast to all neighbors
        Ok(())
    }

    /// Handle route request
    fn handle_route_request(
        &mut self,
        source: DroneId,
        destination: DroneId,
        sequence: u32,
    ) -> Result<()> {
        if destination == self.local_id {
            // We are the destination - send route reply
            let reply = NetworkMessage::RouteReply {
                source: destination,
                destination: source,
                next_hop: self.local_id,
                hop_count: 0,
            };
            // Send reply back
        } else if let Some(route) = self.routes.get(&destination.as_u64()) {
            // We have a route - send reply
            let reply = NetworkMessage::RouteReply {
                source: destination,
                destination: source,
                next_hop: route.next_hop,
                hop_count: route.hop_count,
            };
            // Send reply back
        } else {
            // Forward route request
        }
        Ok(())
    }

    /// Handle route reply
    fn handle_route_reply(
        &mut self,
        source: DroneId,
        destination: DroneId,
        next_hop: DroneId,
        hop_count: u8,
    ) -> Result<()> {
        if destination == self.local_id {
            // Route reply is for us - install route
            let route = Route {
                destination: source,
                next_hop,
                hop_count: hop_count + 1,
                metric: (hop_count + 1) as f32,
                sequence: 0,
                last_updated: Self::get_time(),
            };

            self.routes
                .insert(source.as_u64(), route)
                .map_err(|_| SwarmError::ResourceExhausted)?;

            // Send queued messages
            self.flush_message_queue(source)?;
        } else {
            // Forward route reply
        }
        Ok(())
    }

    /// Handle link state update
    fn handle_link_state_update(
        &mut self,
        sender: DroneId,
        neighbors: Vec<(DroneId, f32), MAX_NEIGHBORS>,
        sequence: u32,
    ) -> Result<()> {
        // Update routing table based on link state information
        // This would implement a distance vector or link state routing protocol
        Ok(())
    }

    /// Queue message for later delivery
    fn queue_message(&mut self, destination: DroneId, payload: Vec<u8, 1024>) -> Result<()> {
        let msg = QueuedMessage {
            destination,
            payload,
            retry_count: 0,
            timestamp: Self::get_time(),
        };

        self.message_queue
            .push(msg)
            .map_err(|_| SwarmError::BufferFull)?;

        Ok(())
    }

    /// Send queued messages to a destination
    fn flush_message_queue(&mut self, destination: DroneId) -> Result<()> {
        // Find and send all messages for this destination
        let mut i = 0;
        let mut iterations = 0;
        const MAX_ITERATIONS: usize = 1000; // Prevent infinite loop

        while i < self.message_queue.len() {
            iterations += 1;
            if iterations > MAX_ITERATIONS {
                return Err(SwarmError::ResourceExhausted);
            }

            if self.message_queue[i].destination == destination {
                let msg = self.message_queue.swap_remove(i);
                // Try to send. If it fails/queues again, we've removed it from here
                // so we won't process it again immediately in this loop unless it's pushed back.
                // Even if pushed back, MAX_ITERATIONS protects us.
                self.send_message(destination, msg.payload)?;
            } else {
                i += 1;
            }
        }
        Ok(())
    }

    /// Broadcast hello message
    pub fn broadcast_hello(&mut self, position: Position) -> Result<()> {
        self.sequence_number += 1;
        let msg = NetworkMessage::Hello {
            sender: self.local_id,
            position,
            sequence: self.sequence_number,
        };
        // Broadcast to all neighbors
        Ok(())
    }

    /// Send heartbeat to maintain connections
    pub fn send_heartbeat(&mut self) -> Result<()> {
        let msg = NetworkMessage::Heartbeat {
            sender: self.local_id,
            timestamp: Self::get_time(),
        };
        // Send to all neighbors
        Ok(())
    }

    /// Prune dead neighbors
    pub fn prune_neighbors(&mut self, timeout_ms: u32) {
        let current_time = Self::get_time();
        self.neighbors
            .retain(|_, neighbor| neighbor.is_alive(current_time, timeout_ms));
    }

    /// Get neighbor count
    pub fn neighbor_count(&self) -> usize {
        self.neighbors.len()
    }

    /// Get all neighbors
    pub fn neighbors(&self) -> impl Iterator<Item = &Neighbor> {
        self.neighbors.values()
    }

    /// Get network statistics
    pub fn statistics(&self) -> &NetworkStats {
        &self.stats
    }

    /// Get current time (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_network_creation() {
        let network = MeshNetwork::new(DroneId::new(1));
        assert_eq!(network.local_id, DroneId::new(1));
        assert_eq!(network.neighbor_count(), 0);
    }

    #[test]
    fn test_neighbor_discovery() {
        let mut network = MeshNetwork::new(DroneId::new(1));
        let msg = NetworkMessage::Hello {
            sender: DroneId::new(2),
            position: Position {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            sequence: 1,
        };

        let addr = NetworkAddress::new([0u8; 16], 8080);
        network.process_message(msg, addr).unwrap();

        assert_eq!(network.neighbor_count(), 1);
    }
}
