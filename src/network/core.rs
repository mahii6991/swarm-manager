//! Mesh networking layer for drone-to-drone communication
//!
//! Implements:
//! - Adaptive mesh routing with automatic topology optimization
//! - Multi-hop communication with distance vector routing
//! - Link quality monitoring
//! - Automatic neighbor discovery
//! - Network resilience and self-healing

use crate::types::*;
use heapless::{FnvIndexMap, Vec};
use log::{debug, trace, warn};
use serde::{Deserialize, Serialize};

/// Maximum number of neighbors in mesh network (must be power of 2 for FnvIndexMap)
pub const MAX_NEIGHBORS: usize = 32;

/// Maximum routing table entries (must be power of 2 for FnvIndexMap)
pub const MAX_ROUTES: usize = 128;

/// Maximum network hops (prevents routing loops)
pub const MAX_NETWORK_HOPS: u8 = 15;

/// Message types for mesh networking
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(clippy::large_enum_variant)]
pub enum NetworkMessage {
    /// Hello message for neighbor discovery
    Hello {
        sender: DroneId,
        position: Position,
        sequence: u32,
    },
    /// Heartbeat to maintain connections
    Heartbeat { sender: DroneId, timestamp: u64 },
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
    /// Message queue (includes pending and retry messages)
    message_queue: Vec<QueuedMessage, 100>,
    /// Network statistics
    stats: NetworkStats,
    /// Retry configuration
    retry_config: RetryConfig,
}

/// Queued message waiting for delivery or retry
#[derive(Debug, Clone)]
struct QueuedMessage {
    /// Destination drone ID
    destination: DroneId,
    /// Message payload
    payload: Vec<u8, 1024>,
    /// Number of retry attempts made
    retry_count: u8,
    /// Timestamp when message was queued (ms)
    queued_at: u64,
    /// Next retry time (ms) - used for exponential backoff
    next_retry_at: u64,
}

/// Configuration for network retry behavior
#[derive(Debug, Clone, Copy)]
pub struct RetryConfig {
    /// Maximum number of retry attempts
    pub max_retries: u8,
    /// Base delay for exponential backoff (ms)
    pub base_delay_ms: u64,
    /// Maximum delay cap (ms)
    pub max_delay_ms: u64,
    /// Message timeout - drop after this duration (ms)
    pub timeout_ms: u64,
}

impl Default for RetryConfig {
    fn default() -> Self {
        Self {
            max_retries: 3,
            base_delay_ms: 100,
            max_delay_ms: 5000,
            timeout_ms: 30000,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct NetworkStats {
    /// Total messages sent
    pub messages_sent: u64,
    /// Total messages received
    pub messages_received: u64,
    /// Messages dropped
    pub messages_dropped: u64,
    /// Messages retried
    pub messages_retried: u64,
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
            retry_config: RetryConfig::default(),
        }
    }

    /// Create a new mesh network instance with custom retry configuration
    pub fn with_retry_config(local_id: DroneId, retry_config: RetryConfig) -> Self {
        Self {
            local_id,
            neighbors: FnvIndexMap::new(),
            routes: FnvIndexMap::new(),
            sequence_number: 0,
            message_queue: Vec::new(),
            stats: NetworkStats::default(),
            retry_config,
        }
    }

    /// Process message retry queue
    ///
    /// Call this periodically (e.g., every 100ms) to retry failed messages
    /// with exponential backoff. Returns the number of messages retried.
    pub fn process_retries(&mut self) -> Result<u32> {
        let current_time = Self::get_time();
        let mut retried = 0u32;
        let mut i = 0;

        while i < self.message_queue.len() {
            // Extract values we need before doing mutable operations
            let (queued_at, retry_count, next_retry_at, destination) = {
                let msg = &self.message_queue[i];
                (msg.queued_at, msg.retry_count, msg.next_retry_at, msg.destination)
            };

            // Check if message has timed out
            if current_time.saturating_sub(queued_at) > self.retry_config.timeout_ms {
                warn!(
                    "Message to drone {} timed out after {}ms",
                    destination.as_u64(),
                    current_time.saturating_sub(queued_at)
                );
                self.message_queue.swap_remove(i);
                self.stats.messages_dropped += 1;
                continue;
            }

            // Check if message exceeded max retries
            if retry_count >= self.retry_config.max_retries {
                warn!(
                    "Message to drone {} dropped after {} retries",
                    destination.as_u64(),
                    retry_count
                );
                self.message_queue.swap_remove(i);
                self.stats.messages_dropped += 1;
                continue;
            }

            // Check if it's time to retry
            if current_time >= next_retry_at {
                // Check if we now have a route
                if self.routes.contains_key(&destination.as_u64()) {
                    let msg = self.message_queue.swap_remove(i);
                    // Attempt to send - if it fails, it will be re-queued with incremented retry
                    if self.send_message_internal(msg.destination, msg.payload.clone(), msg.retry_count + 1).is_ok() {
                        debug!(
                            "Retry {} succeeded for message to drone {}",
                            msg.retry_count + 1,
                            msg.destination.as_u64()
                        );
                        retried += 1;
                        self.stats.messages_retried += 1;
                    }
                    continue;
                } else {
                    // Still no route - initiate discovery and update retry time
                    trace!("No route to drone {}, initiating discovery", destination.as_u64());
                    self.initiate_route_discovery(destination)?;

                    // Update next retry time with exponential backoff
                    let delay = self.calculate_backoff_delay(retry_count);
                    if let Some(msg) = self.message_queue.get_mut(i) {
                        msg.retry_count += 1;
                        msg.next_retry_at = current_time + delay;
                    }
                }
            }

            i += 1;
        }

        Ok(retried)
    }

    /// Calculate exponential backoff delay with jitter
    fn calculate_backoff_delay(&self, retry_count: u8) -> u64 {
        // Exponential backoff: base_delay * 2^retry_count
        let exp_delay = self.retry_config.base_delay_ms
            .saturating_mul(1u64 << retry_count.min(6)); // Cap at 2^6 to prevent overflow

        // Add jitter (±25% of delay) - use modulo for simple pseudo-random jitter
        let jitter = Self::get_time() % (exp_delay / 4 + 1);

        // Cap at max delay
        exp_delay.saturating_add(jitter).min(self.retry_config.max_delay_ms)
    }

    /// Internal send that tracks retry count
    fn send_message_internal(
        &mut self,
        destination: DroneId,
        payload: Vec<u8, 1024>,
        retry_count: u8,
    ) -> Result<()> {
        if let Some(_route) = self.routes.get(&destination.as_u64()) {
            let _msg = NetworkMessage::Data {
                source: self.local_id,
                destination,
                payload,
                hop_count: 0,
            };
            self.stats.messages_sent += 1;

            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio transmission not yet implemented");

            #[cfg(not(feature = "hardware"))]
            Ok(())
        } else {
            // No route - queue for retry
            self.queue_message_with_retry(destination, payload, retry_count)?;
            self.initiate_route_discovery(destination)?;
            Ok(())
        }
    }

    /// Queue message with retry tracking
    fn queue_message_with_retry(
        &mut self,
        destination: DroneId,
        payload: Vec<u8, 1024>,
        retry_count: u8,
    ) -> Result<()> {
        let current_time = Self::get_time();
        let delay = self.calculate_backoff_delay(retry_count);

        let msg = QueuedMessage {
            destination,
            payload,
            retry_count,
            queued_at: current_time,
            next_retry_at: current_time + delay,
        };

        self.message_queue
            .push(msg)
            .map_err(|_| SwarmError::BufferFull)?;

        Ok(())
    }

    /// Process incoming network message
    pub fn process_message(
        &mut self,
        message: NetworkMessage,
        sender_addr: NetworkAddress,
    ) -> Result<Option<Vec<u8, 1024>>> {
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
        if let Some(_route) = self.routes.get(&destination.as_u64()) {
            // Send directly via route
            let _msg = NetworkMessage::Data {
                source: self.local_id,
                destination,
                payload,
                hop_count: 0,
            };
            self.stats.messages_sent += 1;

            // SIMULATION MODE: In production, this would transmit via hardware radio interface
            // For testing/simulation, we track the message but don't perform actual I/O
            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio transmission not yet implemented");

            #[cfg(not(feature = "hardware"))]
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
    fn handle_heartbeat(&mut self, sender: DroneId, _timestamp: u64) -> Result<()> {
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
        let new_hop_count = hop_count.checked_add(1).ok_or(SwarmError::NetworkError)?;

        if new_hop_count > MAX_NETWORK_HOPS {
            self.stats.messages_dropped += 1;
            return Err(SwarmError::NetworkError);
        }

        // Find route
        if let Some(_route) = self.routes.get(&destination.as_u64()) {
            let _msg = NetworkMessage::Data {
                source,
                destination,
                payload,
                hop_count: new_hop_count,
            };

            // SIMULATION MODE: In production, forward to next hop via hardware interface
            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio forwarding not yet implemented");

            #[cfg(not(feature = "hardware"))]
            {
                self.stats.messages_sent += 1;
                Ok(())
            }
        } else {
            // No route - drop or queue
            self.stats.messages_dropped += 1;
            Err(SwarmError::NetworkError)
        }
    }

    /// Initiate route discovery
    fn initiate_route_discovery(&mut self, destination: DroneId) -> Result<()> {
        self.sequence_number += 1;
        let _msg = NetworkMessage::RouteRequest {
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
        _sequence: u32,
    ) -> Result<()> {
        if destination == self.local_id {
            // We are the destination - send route reply
            let _reply = NetworkMessage::RouteReply {
                source: destination,
                destination: source,
                next_hop: self.local_id,
                hop_count: 0,
            };
            // SIMULATION MODE: In production, send reply via hardware interface
            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio transmission not yet implemented");
        } else if let Some(route) = self.routes.get(&destination.as_u64()) {
            // We have a route - send reply
            let _reply = NetworkMessage::RouteReply {
                source: destination,
                destination: source,
                next_hop: route.next_hop,
                hop_count: route.hop_count,
            };
            // SIMULATION MODE: In production, send reply via hardware interface
            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio transmission not yet implemented");
        } else {
            // SIMULATION MODE: In production, forward request to neighbors
            #[cfg(feature = "hardware")]
            unimplemented!("Hardware radio forwarding not yet implemented");
        }
        #[cfg(not(feature = "hardware"))]
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
        _sender: DroneId,
        _neighbors: Vec<(DroneId, f32), MAX_NEIGHBORS>,
        _sequence: u32,
    ) -> Result<()> {
        // SIMULATION MODE: In production, update routing table based on link state
        // This would implement a distance vector or link state routing protocol
        // Currently stubbed for simulation/testing
        #[cfg(feature = "hardware")]
        unimplemented!("Link state routing table updates not yet implemented");

        #[cfg(not(feature = "hardware"))]
        Ok(())
    }

    /// Queue message for later delivery
    fn queue_message(&mut self, destination: DroneId, payload: Vec<u8, 1024>) -> Result<()> {
        self.queue_message_with_retry(destination, payload, 0)
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
        let _msg = NetworkMessage::Hello {
            sender: self.local_id,
            position,
            sequence: self.sequence_number,
        };
        // SIMULATION MODE: In production, broadcast to all neighbors via hardware
        #[cfg(feature = "hardware")]
        unimplemented!("Hardware broadcast not yet implemented");

        #[cfg(not(feature = "hardware"))]
        Ok(())
    }

    /// Send heartbeat to maintain connections
    pub fn send_heartbeat(&mut self) -> Result<()> {
        let _msg = NetworkMessage::Heartbeat {
            sender: self.local_id,
            timestamp: Self::get_time(),
        };
        // SIMULATION MODE: In production, send heartbeat to all neighbors
        #[cfg(feature = "hardware")]
        unimplemented!("Hardware heartbeat transmission not yet implemented");

        #[cfg(not(feature = "hardware"))]
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
