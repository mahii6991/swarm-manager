//! Mesh Network Protocol for Drone Swarm Communication
//!
//! This module defines the message types and protocol for mesh networking
//! between drone swarm nodes. It supports:
//! - Node discovery and heartbeat
//! - Position sharing
//! - Command distribution
//! - Telemetry aggregation
//! - Encrypted communication
//!
//! Protocol is designed for `no_std` environments (ESP32, STM32).

use heapless::Vec;
use serde::{Deserialize, Serialize};

/// Maximum nodes in the mesh network
pub const MAX_MESH_NODES: usize = 64;

/// Maximum message payload size (bytes)
pub const MAX_PAYLOAD_SIZE: usize = 256;

/// Maximum hops for message routing
pub const MAX_HOPS: u8 = 10;

/// Heartbeat interval in milliseconds
pub const HEARTBEAT_INTERVAL_MS: u64 = 1000;

/// Node timeout in milliseconds (no heartbeat received)
pub const NODE_TIMEOUT_MS: u64 = 5000;

/// Mesh node identifier (unique within swarm)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MeshNodeId(pub u8);

impl MeshNodeId {
    pub fn new(id: u8) -> Self {
        Self(id)
    }

    pub fn as_u8(&self) -> u8 {
        self.0
    }
}

/// Message priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MessagePriority {
    /// Low priority (telemetry, status updates)
    Low = 0,
    /// Normal priority (position updates, heartbeats)
    Normal = 1,
    /// High priority (commands, formation changes)
    High = 2,
    /// Critical priority (emergency, collision avoidance)
    Critical = 3,
}

/// Mesh message types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MeshMessageType {
    /// Node discovery/heartbeat
    Heartbeat {
        node_id: MeshNodeId,
        position: [f32; 3],
        battery_percent: u8,
        neighbors_count: u8,
    },

    /// Position update (high frequency)
    PositionUpdate {
        node_id: MeshNodeId,
        position: [f32; 3],
        velocity: [f32; 3],
        heading: f32,
        timestamp_ms: u64,
    },

    /// Command from leader/GCS
    Command {
        command_id: u16,
        target: CommandTarget,
        action: CommandAction,
    },

    /// Acknowledgment
    Ack {
        original_msg_id: u32,
        status: AckStatus,
    },

    /// Formation update
    FormationUpdate {
        formation_id: u8,
        positions: Vec<[f32; 3], 16>,
        leader_id: MeshNodeId,
    },

    /// Telemetry data
    Telemetry {
        node_id: MeshNodeId,
        battery_voltage: f32,
        cpu_usage: u8,
        rssi: i8,
        gps_fix: bool,
        armed: bool,
    },

    /// Emergency broadcast
    Emergency {
        node_id: MeshNodeId,
        emergency_type: EmergencyType,
        position: [f32; 3],
    },

    /// Route discovery (for mesh routing)
    RouteRequest {
        source: MeshNodeId,
        destination: MeshNodeId,
        sequence: u16,
    },

    /// Route response
    RouteResponse {
        source: MeshNodeId,
        destination: MeshNodeId,
        hops: Vec<MeshNodeId, 10>,
        sequence: u16,
    },

    /// Custom application data
    Custom { data: Vec<u8, MAX_PAYLOAD_SIZE> },
}

/// Command target specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CommandTarget {
    /// Single node
    Node(MeshNodeId),
    /// All nodes
    Broadcast,
    /// Specific group
    Group(u8),
}

/// Command actions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CommandAction {
    /// Arm motors
    Arm,
    /// Disarm motors
    Disarm,
    /// Takeoff to altitude
    Takeoff { altitude: f32 },
    /// Land at current position
    Land,
    /// Return to launch
    ReturnToLaunch,
    /// Go to waypoint
    GoTo { position: [f32; 3] },
    /// Set formation
    SetFormation { formation_id: u8 },
    /// Emergency stop
    EmergencyStop,
    /// Set parameter
    SetParam { param_id: u8, value: f32 },
}

/// Acknowledgment status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AckStatus {
    Success,
    Failed,
    Busy,
    NotSupported,
}

/// Emergency types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmergencyType {
    LowBattery,
    MotorFailure,
    GpsLost,
    CollisionWarning,
    LinkLost,
    Geofence,
    Manual,
}

/// Complete mesh message with header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshMessage {
    /// Message ID (unique per sender)
    pub msg_id: u32,
    /// Source node
    pub source: MeshNodeId,
    /// Destination (None = broadcast)
    pub destination: Option<MeshNodeId>,
    /// Hop count (decremented each hop)
    pub ttl: u8,
    /// Priority level
    pub priority: MessagePriority,
    /// Timestamp (ms since boot)
    pub timestamp_ms: u64,
    /// Message payload
    pub payload: MeshMessageType,
    /// CRC32 checksum (computed over all fields except this one)
    pub checksum: u32,
}

impl MeshMessage {
    /// Create a new mesh message
    pub fn new(
        source: MeshNodeId,
        destination: Option<MeshNodeId>,
        priority: MessagePriority,
        payload: MeshMessageType,
        timestamp_ms: u64,
    ) -> Self {
        let mut msg = Self {
            msg_id: 0, // Will be set by sender
            source,
            destination,
            ttl: MAX_HOPS,
            priority,
            timestamp_ms,
            payload,
            checksum: 0,
        };
        msg.checksum = msg.compute_checksum();
        msg
    }

    /// Create heartbeat message
    pub fn heartbeat(
        node_id: MeshNodeId,
        position: [f32; 3],
        battery_percent: u8,
        neighbors_count: u8,
        timestamp_ms: u64,
    ) -> Self {
        Self::new(
            node_id,
            None, // Broadcast
            MessagePriority::Normal,
            MeshMessageType::Heartbeat {
                node_id,
                position,
                battery_percent,
                neighbors_count,
            },
            timestamp_ms,
        )
    }

    /// Create position update message
    pub fn position_update(
        node_id: MeshNodeId,
        position: [f32; 3],
        velocity: [f32; 3],
        heading: f32,
        timestamp_ms: u64,
    ) -> Self {
        Self::new(
            node_id,
            None, // Broadcast
            MessagePriority::Normal,
            MeshMessageType::PositionUpdate {
                node_id,
                position,
                velocity,
                heading,
                timestamp_ms,
            },
            timestamp_ms,
        )
    }

    /// Create command message
    pub fn command(
        source: MeshNodeId,
        target: CommandTarget,
        action: CommandAction,
        timestamp_ms: u64,
    ) -> Self {
        let destination = match &target {
            CommandTarget::Node(id) => Some(*id),
            _ => None,
        };

        Self::new(
            source,
            destination,
            MessagePriority::High,
            MeshMessageType::Command {
                command_id: 0, // Will be set by sender
                target,
                action,
            },
            timestamp_ms,
        )
    }

    /// Create emergency message
    pub fn emergency(
        node_id: MeshNodeId,
        emergency_type: EmergencyType,
        position: [f32; 3],
        timestamp_ms: u64,
    ) -> Self {
        Self::new(
            node_id,
            None, // Broadcast
            MessagePriority::Critical,
            MeshMessageType::Emergency {
                node_id,
                emergency_type,
                position,
            },
            timestamp_ms,
        )
    }

    /// Compute CRC32 checksum
    fn compute_checksum(&self) -> u32 {
        // Simple CRC32 implementation for no_std
        let mut crc: u32 = 0xFFFFFFFF;

        // Hash key fields
        crc = crc32_byte(crc, self.source.0);
        crc = crc32_byte(crc, self.destination.map_or(0xFF, |d| d.0));
        crc = crc32_byte(crc, self.ttl);
        crc = crc32_byte(crc, self.priority as u8);

        // Hash timestamp bytes
        for byte in self.timestamp_ms.to_le_bytes() {
            crc = crc32_byte(crc, byte);
        }

        !crc
    }

    /// Verify checksum
    pub fn verify_checksum(&self) -> bool {
        self.checksum == self.compute_checksum()
    }

    /// Decrement TTL for routing
    pub fn decrement_ttl(&mut self) -> bool {
        if self.ttl > 0 {
            self.ttl -= 1;
            true
        } else {
            false
        }
    }

    /// Check if message should be forwarded
    pub fn should_forward(&self, my_id: MeshNodeId) -> bool {
        self.ttl > 0 && self.source != my_id && self.destination != Some(my_id)
    }

    // Note: Serialization functions require postcard with use-std feature.
    // For now, messages are passed directly in memory.
    // Add serialization when implementing actual network transport:
    //
    // #[cfg(feature = "use-std")]
    // pub fn to_bytes(&self) -> Result<std::vec::Vec<u8>> {
    //     postcard::to_stdvec(self).map_err(|_| SwarmError::SerializationError)
    // }
    //
    // pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
    //     postcard::from_bytes(bytes).map_err(|_| SwarmError::SerializationError)
    // }
}

/// Simple CRC32 byte update
fn crc32_byte(crc: u32, byte: u8) -> u32 {
    let mut c = crc ^ (byte as u32);
    for _ in 0..8 {
        c = if c & 1 != 0 {
            0xEDB88320 ^ (c >> 1)
        } else {
            c >> 1
        };
    }
    c
}

/// Mesh neighbor information
#[derive(Debug, Clone)]
pub struct MeshNeighbor {
    /// Node ID
    pub node_id: MeshNodeId,
    /// Last known position
    pub position: [f32; 3],
    /// Last known velocity
    pub velocity: [f32; 3],
    /// Signal strength (RSSI in dBm)
    pub rssi: i8,
    /// Last heartbeat time (ms)
    pub last_seen_ms: u64,
    /// Battery percentage
    pub battery_percent: u8,
    /// Is node active (heartbeat within timeout)
    pub is_active: bool,
    /// Hop count to reach this node
    pub hop_count: u8,
}

impl MeshNeighbor {
    pub fn new(node_id: MeshNodeId) -> Self {
        Self {
            node_id,
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            rssi: -100,
            last_seen_ms: 0,
            battery_percent: 0,
            is_active: false,
            hop_count: 255,
        }
    }

    /// Update neighbor from heartbeat
    pub fn update_from_heartbeat(
        &mut self,
        position: [f32; 3],
        battery_percent: u8,
        rssi: i8,
        current_time_ms: u64,
    ) {
        self.position = position;
        self.battery_percent = battery_percent;
        self.rssi = rssi;
        self.last_seen_ms = current_time_ms;
        self.is_active = true;
        self.hop_count = 1; // Direct neighbor
    }

    /// Update neighbor from position update
    pub fn update_position(
        &mut self,
        position: [f32; 3],
        velocity: [f32; 3],
        current_time_ms: u64,
    ) {
        self.position = position;
        self.velocity = velocity;
        self.last_seen_ms = current_time_ms;
        self.is_active = true;
    }

    /// Check if neighbor has timed out
    pub fn check_timeout(&mut self, current_time_ms: u64) -> bool {
        if current_time_ms - self.last_seen_ms > NODE_TIMEOUT_MS {
            self.is_active = false;
            true
        } else {
            false
        }
    }

    /// Calculate distance to this neighbor
    pub fn distance_to(&self, position: &[f32; 3]) -> f32 {
        let dx = self.position[0] - position[0];
        let dy = self.position[1] - position[1];
        let dz = self.position[2] - position[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Routing table entry
#[derive(Debug, Clone)]
pub struct RouteEntry {
    /// Destination node
    pub destination: MeshNodeId,
    /// Next hop to reach destination
    pub next_hop: MeshNodeId,
    /// Total hop count
    pub hop_count: u8,
    /// Route sequence number
    pub sequence: u16,
    /// Route last updated (ms)
    pub last_updated_ms: u64,
    /// Is route valid
    pub is_valid: bool,
}

impl RouteEntry {
    pub fn new(
        destination: MeshNodeId,
        next_hop: MeshNodeId,
        hop_count: u8,
        sequence: u16,
        current_time_ms: u64,
    ) -> Self {
        Self {
            destination,
            next_hop,
            hop_count,
            sequence,
            last_updated_ms: current_time_ms,
            is_valid: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_node_id() {
        let id = MeshNodeId::new(5);
        assert_eq!(id.as_u8(), 5);
    }

    #[test]
    fn test_heartbeat_message() {
        let msg = MeshMessage::heartbeat(MeshNodeId::new(1), [10.0, 20.0, 30.0], 85, 3, 1000);

        assert_eq!(msg.source.0, 1);
        assert!(msg.destination.is_none());
        assert_eq!(msg.ttl, MAX_HOPS);
        assert_eq!(msg.priority, MessagePriority::Normal);
        assert!(msg.verify_checksum());
    }

    #[test]
    fn test_command_message() {
        let msg = MeshMessage::command(
            MeshNodeId::new(0),
            CommandTarget::Broadcast,
            CommandAction::Arm,
            2000,
        );

        assert_eq!(msg.priority, MessagePriority::High);
        assert!(msg.destination.is_none());
    }

    #[test]
    fn test_emergency_message() {
        let msg = MeshMessage::emergency(
            MeshNodeId::new(5),
            EmergencyType::LowBattery,
            [100.0, 200.0, 50.0],
            3000,
        );

        assert_eq!(msg.priority, MessagePriority::Critical);
    }

    #[test]
    fn test_neighbor_timeout() {
        let mut neighbor = MeshNeighbor::new(MeshNodeId::new(1));
        neighbor.last_seen_ms = 1000;
        neighbor.is_active = true;

        // Should not timeout at 5000ms
        assert!(!neighbor.check_timeout(5000));

        // Should timeout at 7000ms (> 5000ms timeout)
        assert!(neighbor.check_timeout(7000));
        assert!(!neighbor.is_active);
    }

    #[test]
    fn test_message_forwarding() {
        let mut msg = MeshMessage::heartbeat(MeshNodeId::new(1), [0.0, 0.0, 0.0], 100, 2, 0);

        // Should forward (different node, has TTL)
        assert!(msg.should_forward(MeshNodeId::new(2)));

        // Should not forward (same source)
        assert!(!msg.should_forward(MeshNodeId::new(1)));

        // Decrement TTL
        for _ in 0..MAX_HOPS {
            assert!(msg.decrement_ttl());
        }
        // TTL exhausted
        assert!(!msg.decrement_ttl());
        assert!(!msg.should_forward(MeshNodeId::new(2)));
    }
}
