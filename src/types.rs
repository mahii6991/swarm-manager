//! Core type definitions for the drone swarm system

use core::fmt;
use heapless::Vec;
use serde::{Deserialize, Serialize};

/// Result type for swarm operations
pub type Result<T> = core::result::Result<T, SwarmError>;

/// Unique identifier for each drone in the swarm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DroneId(pub u64);

impl DroneId {
    /// Create a new DroneId from a u64
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Get the inner u64 value
    pub const fn as_u64(&self) -> u64 {
        self.0
    }
}

impl fmt::Display for DroneId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Drone-{:016X}", self.0)
    }
}

/// 3D position vector
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    /// X coordinate (meters)
    pub x: f32,
    /// Y coordinate (meters)
    pub y: f32,
    /// Z coordinate (altitude in meters)
    pub z: f32,
}

impl Position {
    /// Calculate Euclidean distance to another position
    pub fn distance_to(&self, other: &Position) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        libm::sqrtf(dx * dx + dy * dy + dz * dz)
    }
}

/// Velocity vector
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Velocity {
    /// X velocity (m/s)
    pub vx: f32,
    /// Y velocity (m/s)
    pub vy: f32,
    /// Z velocity (m/s)
    pub vz: f32,
}

/// Drone state information
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DroneState {
    /// Drone identifier
    pub id: DroneId,
    /// Current position
    pub position: Position,
    /// Current velocity
    pub velocity: Velocity,
    /// Battery level (0-100%)
    pub battery: u8,
    /// Mission status
    pub status: MissionStatus,
    /// Timestamp (milliseconds since epoch)
    pub timestamp: u64,
}

/// Mission status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MissionStatus {
    /// Idle and ready
    Idle,
    /// Active mission
    Active,
    /// Returning to base
    Returning,
    /// Emergency mode
    Emergency,
    /// System failure
    Failed,
}

/// Comprehensive error types for the swarm system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmError {
    /// Network communication error
    NetworkError,
    /// Cryptographic operation failed
    CryptoError,
    /// Invalid message format
    InvalidMessage,
    /// Authentication failed
    AuthenticationFailed,
    /// Consensus not reached
    ConsensusError,
    /// Buffer overflow
    BufferFull,
    /// Timeout occurred
    Timeout,
    /// Invalid drone ID
    InvalidDroneId,
    /// Operation not permitted
    PermissionDenied,
    /// Resource exhausted
    ResourceExhausted,
    /// Hardware fault detected
    HardwareFault,
    /// Configuration error
    ConfigError,
    /// Swarm size exceeded
    SwarmSizeExceeded,
    /// Invalid parameter provided
    InvalidParameter,
    /// Serialization/deserialization error
    SerializationError,
}

impl fmt::Display for SwarmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SwarmError::NetworkError => write!(f, "Network communication error"),
            SwarmError::CryptoError => write!(f, "Cryptographic operation failed"),
            SwarmError::InvalidMessage => write!(f, "Invalid message format"),
            SwarmError::AuthenticationFailed => write!(f, "Authentication failed"),
            SwarmError::ConsensusError => write!(f, "Consensus not reached"),
            SwarmError::BufferFull => write!(f, "Buffer overflow"),
            SwarmError::Timeout => write!(f, "Operation timeout"),
            SwarmError::InvalidDroneId => write!(f, "Invalid drone ID"),
            SwarmError::PermissionDenied => write!(f, "Operation not permitted"),
            SwarmError::ResourceExhausted => write!(f, "Resource exhausted"),
            SwarmError::HardwareFault => write!(f, "Hardware fault detected"),
            SwarmError::ConfigError => write!(f, "Configuration error"),
            SwarmError::SwarmSizeExceeded => write!(f, "Swarm size exceeded"),
            SwarmError::InvalidParameter => write!(f, "Invalid parameter"),
            SwarmError::SerializationError => write!(f, "Serialization error"),
        }
    }
}

/// Network address type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NetworkAddress {
    /// IPv6 address bytes
    pub addr: [u8; 16],
    /// UDP port
    pub port: u16,
}

impl NetworkAddress {
    /// Create a new network address
    pub const fn new(addr: [u8; 16], port: u16) -> Self {
        Self { addr, port }
    }
}

/// Security clearance levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum SecurityLevel {
    /// Public information
    Public = 0,
    /// Internal swarm communication
    Internal = 1,
    /// Sensitive mission data
    Sensitive = 2,
    /// Critical system operations
    Critical = 3,
}

/// Task priority for swarm coordination
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum TaskPriority {
    /// Low priority task
    Low = 0,
    /// Normal priority
    Normal = 1,
    /// High priority
    High = 2,
    /// Critical - safety related
    Critical = 3,
}

/// Swarm task definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmTask {
    /// Task identifier
    pub task_id: u64,
    /// Target position or area
    pub target: Position,
    /// Task priority
    pub priority: TaskPriority,
    /// Assigned drones
    pub assigned_drones: Vec<DroneId, 32>,
    /// Task status
    pub completed: bool,
}
