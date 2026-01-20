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

/// PSO (Particle Swarm Optimization) configuration
#[derive(Debug, Clone)]
pub struct PsoConfig {
    /// Spatial cell size for collision detection grid (meters)
    /// Default: 10.0
    pub spatial_cell_size: f32,
    /// Number of neighbors for ring topology (k)
    /// Default: 3
    pub ring_topology_k: usize,
    /// Average connections per node for random topology
    /// Default: 4
    pub random_topology_degree: usize,
    /// Inertia weight for velocity updates
    /// Default: 0.7
    pub inertia_weight: f32,
    /// Cognitive coefficient (personal best attraction)
    /// Default: 1.5
    pub cognitive_coefficient: f32,
    /// Social coefficient (global best attraction)
    /// Default: 1.5
    pub social_coefficient: f32,
    /// Maximum velocity as fraction of search space
    /// Default: 0.2
    pub max_velocity_fraction: f32,
}

impl Default for PsoConfig {
    fn default() -> Self {
        Self {
            spatial_cell_size: 10.0,
            ring_topology_k: 3,
            random_topology_degree: 4,
            inertia_weight: 0.7,
            cognitive_coefficient: 1.5,
            social_coefficient: 1.5,
            max_velocity_fraction: 0.2,
        }
    }
}

impl PsoConfig {
    /// Create new PSO configuration with defaults
    pub fn new() -> Self {
        Self::default()
    }

    /// Create configuration optimized for exploration
    pub fn exploration() -> Self {
        Self {
            inertia_weight: 0.9,
            cognitive_coefficient: 2.0,
            social_coefficient: 1.0,
            ..Self::default()
        }
    }

    /// Create configuration optimized for exploitation
    pub fn exploitation() -> Self {
        Self {
            inertia_weight: 0.4,
            cognitive_coefficient: 1.0,
            social_coefficient: 2.0,
            ..Self::default()
        }
    }

    /// Validate PSO configuration
    pub fn validate(&self) -> Result<()> {
        if self.spatial_cell_size <= 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.ring_topology_k == 0 {
            return Err(SwarmError::ConfigError);
        }
        if self.random_topology_degree == 0 {
            return Err(SwarmError::ConfigError);
        }
        if self.inertia_weight < 0.0 || self.inertia_weight > 1.0 {
            return Err(SwarmError::ConfigError);
        }
        Ok(())
    }
}

/// Collision avoidance configuration
#[derive(Debug, Clone)]
pub struct CollisionConfig {
    /// Minimum separation distance between drones (meters)
    /// Default: 3.0
    pub min_separation: f32,
    /// Safety margin added to obstacles (meters)
    /// Default: 1.0
    pub safety_margin: f32,
    /// Maximum velocity magnitude (m/s)
    /// Default: 10.0
    pub max_velocity: f32,
    /// Time horizon for collision prediction (seconds)
    /// Default: 5.0
    pub time_horizon: f32,
    /// Responsiveness factor (0.0-1.0)
    /// Default: 0.8
    pub responsiveness: f32,
    /// Enable geofencing
    /// Default: true
    pub geofence_enabled: bool,
    /// Geofence boundary buffer (meters)
    /// Default: 5.0
    pub geofence_buffer: f32,
    /// Altitude limits [min, max] in meters AGL
    /// Default: [2.0, 100.0]
    pub altitude_limits: [f32; 2],
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self {
            min_separation: 3.0,
            safety_margin: 1.0,
            max_velocity: 10.0,
            time_horizon: 5.0,
            responsiveness: 0.8,
            geofence_enabled: true,
            geofence_buffer: 5.0,
            altitude_limits: [2.0, 100.0],
        }
    }
}

impl CollisionConfig {
    /// Create new collision configuration with defaults
    pub fn new() -> Self {
        Self::default()
    }

    /// Create configuration for tight formations
    pub fn tight_formation() -> Self {
        Self {
            min_separation: 1.5,
            safety_margin: 0.5,
            responsiveness: 0.9,
            ..Self::default()
        }
    }

    /// Create configuration for safe/conservative operation
    pub fn conservative() -> Self {
        Self {
            min_separation: 5.0,
            safety_margin: 2.0,
            max_velocity: 5.0,
            responsiveness: 0.95,
            ..Self::default()
        }
    }

    /// Validate collision configuration
    pub fn validate(&self) -> Result<()> {
        if self.min_separation <= 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.safety_margin < 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.max_velocity <= 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.time_horizon <= 0.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.responsiveness < 0.0 || self.responsiveness > 1.0 {
            return Err(SwarmError::ConfigError);
        }
        if self.altitude_limits[0] >= self.altitude_limits[1] {
            return Err(SwarmError::ConfigError);
        }
        Ok(())
    }
}

/// Extended cryptographic configuration
#[derive(Debug, Clone)]
pub struct ExtendedCryptoConfig {
    /// Base crypto configuration
    pub base: CryptoConfig,
    /// Maximum safe encryptions before rekeying is required
    /// Default: 1_000_000_000 (1 billion)
    pub max_safe_encryptions: u64,
    /// Enable automatic key rotation
    /// Default: true
    pub auto_key_rotation: bool,
    /// Minimum key rotation interval (seconds)
    /// Default: 3600 (1 hour)
    pub min_rotation_interval: u32,
    /// Maximum key rotation interval (seconds)
    /// Default: 86400 (24 hours)
    pub max_rotation_interval: u32,
    /// Enable replay protection
    /// Default: true
    pub replay_protection_enabled: bool,
    /// Replay window size (number of nonces to track)
    /// Default: 1024
    pub replay_window_size: usize,
}

impl ExtendedCryptoConfig {
    /// Create new extended crypto configuration
    pub fn new(identity_key: [u8; KEY_SIZE]) -> Self {
        Self {
            base: CryptoConfig::new(identity_key),
            max_safe_encryptions: 1_000_000_000,
            auto_key_rotation: true,
            min_rotation_interval: 3600,
            max_rotation_interval: 86400,
            replay_protection_enabled: true,
            replay_window_size: 1024,
        }
    }

    /// Create high-security configuration
    pub fn high_security(identity_key: [u8; KEY_SIZE]) -> Self {
        Self {
            base: CryptoConfig {
                identity_key,
                post_quantum_enabled: true,
                key_rotation_interval: 1800, // 30 minutes
                nonce_counter: 0,
            },
            max_safe_encryptions: 100_000_000, // 100 million
            auto_key_rotation: true,
            min_rotation_interval: 1800,
            max_rotation_interval: 7200,
            replay_protection_enabled: true,
            replay_window_size: 4096,
        }
    }

    /// Validate extended crypto configuration
    pub fn validate(&self) -> Result<()> {
        if self.max_safe_encryptions == 0 {
            return Err(SwarmError::ConfigError);
        }
        if self.min_rotation_interval >= self.max_rotation_interval {
            return Err(SwarmError::ConfigError);
        }
        if self.replay_window_size == 0 {
            return Err(SwarmError::ConfigError);
        }
        Ok(())
    }
}

/// Complete system configuration combining all sub-configurations
#[derive(Debug, Clone)]
pub struct SystemConfig {
    /// Core swarm configuration
    pub swarm: SwarmConfig,
    /// PSO algorithm configuration
    pub pso: PsoConfig,
    /// Collision avoidance configuration
    pub collision: CollisionConfig,
    /// Extended crypto configuration (optional)
    pub crypto: Option<ExtendedCryptoConfig>,
    /// Network configuration (optional)
    pub network: Option<NetworkConfig>,
}

impl SystemConfig {
    /// Create new system configuration with all defaults
    pub fn new(drone_id: DroneId) -> Self {
        Self {
            swarm: SwarmConfig::new(drone_id),
            pso: PsoConfig::default(),
            collision: CollisionConfig::default(),
            crypto: None,
            network: None,
        }
    }

    /// Create system configuration for testing
    pub fn test_config(drone_id: DroneId) -> Self {
        Self {
            swarm: SwarmConfig::test_config(drone_id),
            pso: PsoConfig::default(),
            collision: CollisionConfig::default(),
            crypto: None,
            network: None,
        }
    }

    /// Create high-performance configuration
    pub fn high_performance(drone_id: DroneId) -> Self {
        Self {
            swarm: SwarmConfig::new(drone_id),
            pso: PsoConfig::exploitation(),
            collision: CollisionConfig::tight_formation(),
            crypto: None,
            network: None,
        }
    }

    /// Create high-security configuration
    pub fn high_security(drone_id: DroneId, identity_key: [u8; KEY_SIZE]) -> Self {
        Self {
            swarm: SwarmConfig::new(drone_id),
            pso: PsoConfig::default(),
            collision: CollisionConfig::conservative(),
            crypto: Some(ExtendedCryptoConfig::high_security(identity_key)),
            network: None,
        }
    }

    /// Validate entire system configuration
    pub fn validate(&self) -> Result<()> {
        self.swarm.validate()?;
        self.pso.validate()?;
        self.collision.validate()?;
        if let Some(ref crypto) = self.crypto {
            crypto.validate()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pso_config_defaults() {
        let config = PsoConfig::default();
        assert_eq!(config.spatial_cell_size, 10.0);
        assert_eq!(config.ring_topology_k, 3);
        assert_eq!(config.random_topology_degree, 4);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_collision_config_defaults() {
        let config = CollisionConfig::default();
        assert_eq!(config.min_separation, 3.0);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_system_config_validation() {
        let config = SystemConfig::new(DroneId::new(1));
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_invalid_pso_config() {
        let mut config = PsoConfig::default();
        config.spatial_cell_size = -1.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_invalid_collision_config() {
        let mut config = CollisionConfig::default();
        config.altitude_limits = [100.0, 50.0]; // Invalid: min > max
        assert!(config.validate().is_err());
    }
}
