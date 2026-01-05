//! Configuration management for the drone swarm system

use crate::types::*;
use crate::types::{KEY_SIZE, MAX_SWARM_SIZE};

/// System configuration
#[derive(Debug, Clone)]
pub struct SwarmConfig {
    /// This drone's unique identifier
    pub drone_id: DroneId,
    /// Maximum number of neighbors in mesh network
    pub max_neighbors: usize,
    /// Communication range (meters)
    pub comm_range: f32,
    /// Enable encryption
    pub encryption_enabled: bool,
    /// Enable consensus protocol
    pub consensus_enabled: bool,
    /// Enable federated learning
    pub federated_learning_enabled: bool,
    /// Heartbeat interval (milliseconds)
    pub heartbeat_interval_ms: u32,
    /// Election timeout range (min, max) in milliseconds
    pub election_timeout_range: (u32, u32),
    /// Network retry attempts
    pub max_retries: u8,
    /// Fault detection threshold
    pub fault_threshold: u8,
}

impl SwarmConfig {
    /// Create a new configuration with secure defaults
    pub fn new(drone_id: DroneId) -> Self {
        Self {
            drone_id,
            max_neighbors: 10,
            comm_range: 1000.0, // 1km
            encryption_enabled: true,
            consensus_enabled: true,
            federated_learning_enabled: true,
            heartbeat_interval_ms: 50,
            election_timeout_range: (150, 300),
            max_retries: 3,
            fault_threshold: 3,
        }
    }

    /// Create a configuration for testing (less secure)
    pub fn test_config(drone_id: DroneId) -> Self {
        Self {
            drone_id,
            max_neighbors: 5,
            comm_range: 500.0,
            encryption_enabled: false, // Disabled for testing
            consensus_enabled: true,
            federated_learning_enabled: false,
            heartbeat_interval_ms: 100,
            election_timeout_range: (200, 400),
            max_retries: 2,
            fault_threshold: 2,
        }
    }

    /// Validate configuration
    pub fn validate(&self) -> Result<()> {
        if self.max_neighbors == 0 || self.max_neighbors > MAX_SWARM_SIZE {
            return Err(SwarmError::ConfigError);
        }
        if self.comm_range <= 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.heartbeat_interval_ms == 0 {
            return Err(SwarmError::ConfigError);
        }
        if self.election_timeout_range.0 >= self.election_timeout_range.1 {
            return Err(SwarmError::ConfigError);
        }
        Ok(())
    }
}

/// Cryptographic configuration
#[derive(Debug, Clone)]
pub struct CryptoConfig {
    /// Long-term identity key pair
    pub identity_key: [u8; KEY_SIZE],
    /// Enable post-quantum cryptography
    pub post_quantum_enabled: bool,
    /// Key rotation interval (seconds)
    pub key_rotation_interval: u32,
    /// Nonce counter for replay protection
    pub nonce_counter: u64,
}

impl CryptoConfig {
    /// Create a new crypto configuration
    /// Note: In production, keys should be generated from hardware RNG
    pub fn new(identity_key: [u8; KEY_SIZE]) -> Self {
        Self {
            identity_key,
            post_quantum_enabled: false, // Enable when PQC is standardized
            key_rotation_interval: 3600, // 1 hour
            nonce_counter: 0,
        }
    }

    /// Generate a new nonce for encryption
    pub fn next_nonce(&mut self) -> [u8; 12] {
        self.nonce_counter += 1;
        let mut nonce = [0u8; 12];
        nonce[..8].copy_from_slice(&self.nonce_counter.to_le_bytes());
        nonce
    }
}

/// Network configuration
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    /// Local network address
    pub local_address: NetworkAddress,
    /// Maximum transmission unit
    pub mtu: u16,
    /// Enable IPv6
    pub ipv6_enabled: bool,
    /// Mesh routing protocol
    pub routing_protocol: RoutingProtocol,
}

impl NetworkConfig {
    /// Create default network configuration
    pub fn new(local_address: NetworkAddress) -> Self {
        Self {
            local_address,
            mtu: 1280, // IPv6 minimum MTU
            ipv6_enabled: true,
            routing_protocol: RoutingProtocol::AdaptiveMesh,
        }
    }
}

/// Routing protocol selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoutingProtocol {
    /// Adaptive mesh with automatic route optimization
    AdaptiveMesh,
    /// Flooding for small networks
    Flooding,
    /// Gradient-based routing
    GradientBased,
}
