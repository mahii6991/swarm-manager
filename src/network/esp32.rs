//! ESP32 WiFi Mesh Network Module
//!
//! This module implements mesh networking for ESP32-based drone swarms.
//! It provides:
//! - Automatic node discovery
//! - Multi-hop message routing
//! - Position synchronization
//! - Command distribution
//!
//! # Features
//! - `std` - Desktop simulation mode (for testing without hardware)
//! - `esp32` - Real ESP32 hardware mode
//!
//! # Example (Desktop Simulation)
//! ```rust,ignore
//! use drone_swarm_system::esp32_mesh::*;
//!
//! let mut node = MeshNode::new(MeshNodeId::new(1));
//! node.set_position([10.0, 20.0, 30.0]);
//! node.broadcast_heartbeat(current_time_ms);
//! ```

use crate::network::mesh::*;
use crate::types::*;
use heapless::Vec;

/// Maximum neighbors to track
pub const MAX_NEIGHBORS: usize = 32;

/// Maximum pending messages in queue
pub const MAX_PENDING_MESSAGES: usize = 16;

/// Maximum routes in routing table
pub const MAX_ROUTES: usize = 64;

/// Mesh node state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeState {
    /// Node is initializing
    Initializing,
    /// Node is scanning for mesh
    Scanning,
    /// Node is connected to mesh
    Connected,
    /// Node is the root (leader)
    Root,
    /// Node is disconnected
    Disconnected,
    /// Node encountered an error
    Error,
}

/// Mesh node configuration
#[derive(Debug, Clone)]
pub struct MeshConfig {
    /// Node ID (must be unique in swarm)
    pub node_id: MeshNodeId,
    /// Mesh network ID (all nodes must match)
    pub mesh_id: [u8; 6],
    /// WiFi channel (1-13)
    pub channel: u8,
    /// Maximum layer depth
    pub max_layer: u8,
    /// Heartbeat interval (ms)
    pub heartbeat_interval_ms: u64,
    /// Node timeout (ms)
    pub node_timeout_ms: u64,
    /// Enable encryption
    pub encryption_enabled: bool,
}

impl Default for MeshConfig {
    fn default() -> Self {
        Self {
            node_id: MeshNodeId::new(0),
            mesh_id: [0x44, 0x52, 0x4F, 0x4E, 0x45, 0x53], // "DRONES"
            channel: 6,
            max_layer: 6,
            heartbeat_interval_ms: HEARTBEAT_INTERVAL_MS,
            node_timeout_ms: NODE_TIMEOUT_MS,
            encryption_enabled: true,
        }
    }
}

/// Mesh network node
pub struct MeshNode {
    /// Configuration
    config: MeshConfig,
    /// Current state
    state: NodeState,
    /// Current position [x, y, z]
    position: [f32; 3],
    /// Current velocity [vx, vy, vz]
    velocity: [f32; 3],
    /// Battery percentage
    battery_percent: u8,
    /// Known neighbors
    neighbors: Vec<MeshNeighbor, MAX_NEIGHBORS>,
    /// Routing table (reserved for future multi-hop routing)
    #[allow(dead_code)]
    routes: Vec<RouteEntry, MAX_ROUTES>,
    /// Outgoing message queue
    tx_queue: Vec<MeshMessage, MAX_PENDING_MESSAGES>,
    /// Incoming message queue (reserved for async processing)
    #[allow(dead_code)]
    rx_queue: Vec<MeshMessage, MAX_PENDING_MESSAGES>,
    /// Message sequence counter
    msg_sequence: u32,
    /// Last heartbeat time (ms)
    last_heartbeat_ms: u64,
    /// Statistics
    stats: MeshStats,
}

/// Mesh network statistics
#[derive(Debug, Clone, Default)]
pub struct MeshStats {
    /// Messages sent
    pub tx_count: u32,
    /// Messages received
    pub rx_count: u32,
    /// Messages forwarded
    pub forward_count: u32,
    /// Messages dropped (TTL, buffer full, etc.)
    pub drop_count: u32,
    /// Heartbeats sent
    pub heartbeat_count: u32,
    /// Current neighbor count
    pub neighbor_count: u8,
    /// Active neighbor count
    pub active_neighbors: u8,
}

impl MeshNode {
    /// Create a new mesh node
    pub fn new(node_id: MeshNodeId) -> Self {
        let config = MeshConfig {
            node_id,
            ..MeshConfig::default()
        };

        Self {
            config,
            state: NodeState::Initializing,
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            battery_percent: 100,
            neighbors: Vec::new(),
            routes: Vec::new(),
            tx_queue: Vec::new(),
            rx_queue: Vec::new(),
            msg_sequence: 0,
            last_heartbeat_ms: 0,
            stats: MeshStats::default(),
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: MeshConfig) -> Self {
        Self {
            config,
            state: NodeState::Initializing,
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            battery_percent: 100,
            neighbors: Vec::new(),
            routes: Vec::new(),
            tx_queue: Vec::new(),
            rx_queue: Vec::new(),
            msg_sequence: 0,
            last_heartbeat_ms: 0,
            stats: MeshStats::default(),
        }
    }

    /// Initialize the mesh node
    pub fn init(&mut self) -> Result<()> {
        self.state = NodeState::Scanning;
        Ok(())
    }

    /// Start the mesh network
    pub fn start(&mut self) -> Result<()> {
        self.state = NodeState::Connected;
        Ok(())
    }

    /// Stop the mesh network
    pub fn stop(&mut self) -> Result<()> {
        self.state = NodeState::Disconnected;
        Ok(())
    }

    /// Get current state
    pub fn state(&self) -> NodeState {
        self.state
    }

    /// Get node ID
    pub fn node_id(&self) -> MeshNodeId {
        self.config.node_id
    }

    /// Set current position
    pub fn set_position(&mut self, position: [f32; 3]) {
        self.position = position;
    }

    /// Get current position
    pub fn position(&self) -> [f32; 3] {
        self.position
    }

    /// Set current velocity
    pub fn set_velocity(&mut self, velocity: [f32; 3]) {
        self.velocity = velocity;
    }

    /// Set battery percentage
    pub fn set_battery(&mut self, percent: u8) {
        self.battery_percent = percent.min(100);
    }

    /// Get statistics
    pub fn stats(&self) -> &MeshStats {
        &self.stats
    }

    /// Get neighbor count
    pub fn neighbor_count(&self) -> usize {
        self.neighbors.iter().filter(|n| n.is_active).count()
    }

    /// Get all active neighbors
    pub fn get_neighbors(&self) -> impl Iterator<Item = &MeshNeighbor> {
        self.neighbors.iter().filter(|n| n.is_active)
    }

    /// Get neighbor by ID
    pub fn get_neighbor(&self, node_id: MeshNodeId) -> Option<&MeshNeighbor> {
        self.neighbors.iter().find(|n| n.node_id == node_id)
    }

    /// Broadcast heartbeat message
    pub fn broadcast_heartbeat(&mut self, current_time_ms: u64) -> Result<()> {
        let msg = MeshMessage::heartbeat(
            self.config.node_id,
            self.position,
            self.battery_percent,
            self.neighbor_count() as u8,
            current_time_ms,
        );

        self.queue_message(msg)?;
        self.last_heartbeat_ms = current_time_ms;
        self.stats.heartbeat_count += 1;

        Ok(())
    }

    /// Broadcast position update
    pub fn broadcast_position(&mut self, heading: f32, current_time_ms: u64) -> Result<()> {
        let msg = MeshMessage::position_update(
            self.config.node_id,
            self.position,
            self.velocity,
            heading,
            current_time_ms,
        );

        self.queue_message(msg)
    }

    /// Send command to target
    pub fn send_command(
        &mut self,
        target: CommandTarget,
        action: CommandAction,
        current_time_ms: u64,
    ) -> Result<()> {
        let msg = MeshMessage::command(self.config.node_id, target, action, current_time_ms);
        self.queue_message(msg)
    }

    /// Broadcast emergency
    pub fn broadcast_emergency(
        &mut self,
        emergency_type: EmergencyType,
        current_time_ms: u64,
    ) -> Result<()> {
        let msg = MeshMessage::emergency(
            self.config.node_id,
            emergency_type,
            self.position,
            current_time_ms,
        );

        // Emergency messages go to front of queue
        if self.tx_queue.len() >= MAX_PENDING_MESSAGES {
            // Remove lowest priority message
            if let Some(idx) = self
                .tx_queue
                .iter()
                .position(|m| m.priority == MessagePriority::Low)
            {
                self.tx_queue.swap_remove(idx);
            }
        }

        self.tx_queue
            .push(msg)
            .map_err(|_| SwarmError::BufferFull)?;
        Ok(())
    }

    /// Queue a message for transmission
    fn queue_message(&mut self, mut msg: MeshMessage) -> Result<()> {
        msg.msg_id = self.next_sequence();

        if self.tx_queue.len() >= MAX_PENDING_MESSAGES {
            self.stats.drop_count += 1;
            return Err(SwarmError::BufferFull);
        }

        self.tx_queue
            .push(msg)
            .map_err(|_| SwarmError::BufferFull)?;
        Ok(())
    }

    /// Get next message to transmit
    pub fn get_next_tx_message(&mut self) -> Option<MeshMessage> {
        if self.tx_queue.is_empty() {
            return None;
        }

        // Find highest priority message
        let mut best_idx = 0;
        let mut best_priority = self.tx_queue[0].priority as u8;

        for (i, msg) in self.tx_queue.iter().enumerate().skip(1) {
            if (msg.priority as u8) > best_priority {
                best_priority = msg.priority as u8;
                best_idx = i;
            }
        }

        let msg = self.tx_queue.swap_remove(best_idx);
        self.stats.tx_count += 1;
        Some(msg)
    }

    /// Process received message
    pub fn process_message(
        &mut self,
        msg: MeshMessage,
        rssi: i8,
        current_time_ms: u64,
    ) -> Result<ProcessResult> {
        // Verify checksum
        if !msg.verify_checksum() {
            self.stats.drop_count += 1;
            return Ok(ProcessResult::Dropped);
        }

        self.stats.rx_count += 1;

        // Check if message is for us (broadcast or addressed to us)
        let _is_for_us = msg.destination.is_none() || msg.destination == Some(self.config.node_id);
        let is_from_us = msg.source == self.config.node_id;

        // Don't process our own messages
        if is_from_us {
            return Ok(ProcessResult::Ignored);
        }

        // Process message
        let result = match &msg.payload {
            MeshMessageType::Heartbeat {
                node_id,
                position,
                battery_percent,
                neighbors_count: _,
            } => {
                self.update_neighbor(*node_id, *position, *battery_percent, rssi, current_time_ms);
                ProcessResult::Processed
            }

            MeshMessageType::PositionUpdate {
                node_id,
                position,
                velocity,
                heading: _,
                timestamp_ms: _,
            } => {
                if let Some(neighbor) = self.neighbors.iter_mut().find(|n| n.node_id == *node_id) {
                    neighbor.update_position(*position, *velocity, current_time_ms);
                }
                ProcessResult::Processed
            }

            MeshMessageType::Command {
                command_id: _,
                target,
                action,
            } => {
                let applies_to_us = match target {
                    CommandTarget::Broadcast => true,
                    CommandTarget::Node(id) => *id == self.config.node_id,
                    CommandTarget::Group(_) => false, // TODO: implement groups
                };

                if applies_to_us {
                    ProcessResult::Command(action.clone())
                } else {
                    ProcessResult::Processed
                }
            }

            MeshMessageType::Emergency {
                node_id,
                emergency_type,
                position,
            } => ProcessResult::Emergency {
                node_id: *node_id,
                emergency_type: *emergency_type,
                position: *position,
            },

            MeshMessageType::Telemetry { .. } => ProcessResult::Processed,
            MeshMessageType::FormationUpdate { .. } => ProcessResult::Processed,
            MeshMessageType::Ack { .. } => ProcessResult::Processed,
            MeshMessageType::RouteRequest { .. } => ProcessResult::Processed,
            MeshMessageType::RouteResponse { .. } => ProcessResult::Processed,
            MeshMessageType::Custom { .. } => ProcessResult::Processed,
        };

        // Forward if needed (broadcast or not for us)
        if msg.should_forward(self.config.node_id) {
            let mut forward_msg = msg.clone();
            if forward_msg.decrement_ttl() {
                // Try to queue for forwarding; if queue is full, increment drop count
                if self.tx_queue.push(forward_msg).is_err() {
                    self.stats.drop_count += 1;
                    // Note: We don't return error here to avoid blocking message processing
                    // when forward queue is full. The message was already processed locally.
                } else {
                    self.stats.forward_count += 1;
                }
            }
        }

        Ok(result)
    }

    /// Update neighbor information
    fn update_neighbor(
        &mut self,
        node_id: MeshNodeId,
        position: [f32; 3],
        battery_percent: u8,
        rssi: i8,
        current_time_ms: u64,
    ) {
        // Find existing neighbor
        if let Some(neighbor) = self.neighbors.iter_mut().find(|n| n.node_id == node_id) {
            neighbor.update_from_heartbeat(position, battery_percent, rssi, current_time_ms);
        } else {
            // Add new neighbor
            let mut neighbor = MeshNeighbor::new(node_id);
            neighbor.update_from_heartbeat(position, battery_percent, rssi, current_time_ms);

            if self.neighbors.push(neighbor).is_err() {
                // Remove oldest inactive neighbor to make room
                if let Some(idx) = self.neighbors.iter().position(|n| !n.is_active) {
                    self.neighbors.swap_remove(idx);
                    // Try again with fresh neighbor
                    let mut new_neighbor = MeshNeighbor::new(node_id);
                    new_neighbor.update_from_heartbeat(
                        position,
                        battery_percent,
                        rssi,
                        current_time_ms,
                    );
                    // If this still fails, log it in stats (neighbor table is completely full of active nodes)
                    if self.neighbors.push(new_neighbor).is_err() {
                        // All neighbors are active, can't add new one - this is not an error,
                        // just means we're at capacity with all active neighbors
                        self.stats.drop_count += 1;
                    }
                }
                // If no inactive neighbor found, the table is full of active neighbors
                // This is acceptable - we just can't track more neighbors
            }
        }

        self.update_neighbor_stats();
    }

    /// Update periodic tasks (call in main loop)
    pub fn update(&mut self, current_time_ms: u64) -> Result<()> {
        // Check for neighbor timeouts
        for neighbor in &mut self.neighbors {
            neighbor.check_timeout(current_time_ms);
        }

        self.update_neighbor_stats();

        // Auto-send heartbeat if needed
        if current_time_ms - self.last_heartbeat_ms >= self.config.heartbeat_interval_ms {
            self.broadcast_heartbeat(current_time_ms)?;
        }

        Ok(())
    }

    /// Update neighbor statistics
    fn update_neighbor_stats(&mut self) {
        self.stats.neighbor_count = self.neighbors.len() as u8;
        self.stats.active_neighbors = self.neighbors.iter().filter(|n| n.is_active).count() as u8;
    }

    /// Get next message sequence number
    fn next_sequence(&mut self) -> u32 {
        self.msg_sequence = self.msg_sequence.wrapping_add(1);
        self.msg_sequence
    }

    /// Check if heartbeat is due
    pub fn is_heartbeat_due(&self, current_time_ms: u64) -> bool {
        current_time_ms - self.last_heartbeat_ms >= self.config.heartbeat_interval_ms
    }

    /// Get closest neighbor
    pub fn get_closest_neighbor(&self) -> Option<&MeshNeighbor> {
        self.neighbors
            .iter()
            .filter(|n| n.is_active)
            .min_by(|a, b| {
                let dist_a = a.distance_to(&self.position);
                let dist_b = b.distance_to(&self.position);
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
    }

    /// Get neighbor positions for swarm algorithms
    ///
    /// Returns positions of all active neighbors. If the output buffer is full,
    /// remaining positions are silently dropped (this is acceptable as the buffer
    /// matches MAX_NEIGHBORS and we can't have more active neighbors than that).
    pub fn get_neighbor_positions(&self) -> Vec<[f32; 3], MAX_NEIGHBORS> {
        let mut positions = Vec::new();
        for neighbor in self.neighbors.iter().filter(|n| n.is_active) {
            // This should never fail since we can't have more neighbors than MAX_NEIGHBORS
            // but we handle it gracefully by stopping early
            if positions.push(neighbor.position).is_err() {
                break;
            }
        }
        positions
    }

    /// Calculate swarm center (average of all known positions)
    pub fn calculate_swarm_center(&self) -> [f32; 3] {
        let active_count = self.neighbors.iter().filter(|n| n.is_active).count();
        if active_count == 0 {
            return self.position;
        }

        let mut center = [0.0, 0.0, 0.0];

        // Include self
        center[0] += self.position[0];
        center[1] += self.position[1];
        center[2] += self.position[2];

        // Include neighbors
        for neighbor in self.neighbors.iter().filter(|n| n.is_active) {
            center[0] += neighbor.position[0];
            center[1] += neighbor.position[1];
            center[2] += neighbor.position[2];
        }

        let total = (active_count + 1) as f32;
        center[0] /= total;
        center[1] /= total;
        center[2] /= total;

        center
    }
}

/// Result of processing a message
#[derive(Debug, Clone)]
pub enum ProcessResult {
    /// Message was processed normally
    Processed,
    /// Message was ignored (e.g., from self)
    Ignored,
    /// Message was dropped (invalid checksum, etc.)
    Dropped,
    /// Command that should be executed
    Command(CommandAction),
    /// Emergency that should be handled
    Emergency {
        node_id: MeshNodeId,
        emergency_type: EmergencyType,
        position: [f32; 3],
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_node_creation() {
        let node = MeshNode::new(MeshNodeId::new(1));
        assert_eq!(node.node_id().0, 1);
        assert_eq!(node.state(), NodeState::Initializing);
    }

    #[test]
    fn test_mesh_node_init() {
        let mut node = MeshNode::new(MeshNodeId::new(1));
        assert!(node.init().is_ok());
        assert_eq!(node.state(), NodeState::Scanning);

        assert!(node.start().is_ok());
        assert_eq!(node.state(), NodeState::Connected);
    }

    #[test]
    fn test_position_update() {
        let mut node = MeshNode::new(MeshNodeId::new(1));
        node.set_position([10.0, 20.0, 30.0]);
        assert_eq!(node.position(), [10.0, 20.0, 30.0]);
    }

    #[test]
    fn test_heartbeat_broadcast() {
        let mut node = MeshNode::new(MeshNodeId::new(1));
        node.set_position([10.0, 20.0, 30.0]);
        node.set_battery(85);

        assert!(node.broadcast_heartbeat(1000).is_ok());
        assert_eq!(node.stats().heartbeat_count, 1);

        // Should have message in queue
        let msg = node.get_next_tx_message();
        assert!(msg.is_some());

        let msg = msg.unwrap();
        assert_eq!(msg.source.0, 1);
        assert!(msg.destination.is_none()); // Broadcast
    }

    #[test]
    fn test_neighbor_discovery() {
        let mut node = MeshNode::new(MeshNodeId::new(1));

        // Create heartbeat from another node
        let heartbeat = MeshMessage::heartbeat(MeshNodeId::new(2), [50.0, 60.0, 10.0], 90, 0, 1000);

        // Process heartbeat
        let result = node.process_message(heartbeat, -50, 1000);
        assert!(result.is_ok());

        // Should have neighbor now
        assert_eq!(node.neighbor_count(), 1);

        let neighbor = node.get_neighbor(MeshNodeId::new(2));
        assert!(neighbor.is_some());
        assert_eq!(neighbor.unwrap().position, [50.0, 60.0, 10.0]);
    }

    #[test]
    fn test_command_processing() {
        let mut node = MeshNode::new(MeshNodeId::new(1));

        let cmd = MeshMessage::command(
            MeshNodeId::new(0),
            CommandTarget::Broadcast,
            CommandAction::Arm,
            1000,
        );

        let result = node.process_message(cmd, -40, 1000).unwrap();

        match result {
            ProcessResult::Command(CommandAction::Arm) => {}
            _ => panic!("Expected Arm command"),
        }
    }

    #[test]
    fn test_swarm_center_calculation() {
        let mut node = MeshNode::new(MeshNodeId::new(1));
        node.set_position([0.0, 0.0, 10.0]);

        // Add neighbors
        let hb1 = MeshMessage::heartbeat(MeshNodeId::new(2), [10.0, 0.0, 10.0], 100, 0, 1000);
        let hb2 = MeshMessage::heartbeat(MeshNodeId::new(3), [0.0, 10.0, 10.0], 100, 0, 1000);

        node.process_message(hb1, -40, 1000).ok();
        node.process_message(hb2, -40, 1000).ok();

        let center = node.calculate_swarm_center();

        // Center should be average: (0+10+0)/3, (0+0+10)/3, (10+10+10)/3
        assert!((center[0] - 3.33).abs() < 0.1);
        assert!((center[1] - 3.33).abs() < 0.1);
        assert_eq!(center[2], 10.0);
    }
}
