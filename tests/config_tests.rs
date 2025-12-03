//! Comprehensive tests for configuration management
//!
//! These tests ensure all configuration logic is correct and validated

use drone_swarm_system::*;
use drone_swarm_system::config::*;

#[cfg(test)]
mod swarm_config_tests {
    use super::*;

    #[test]
    fn test_new_config_has_secure_defaults() {
        let drone_id = DroneId::new(1);
        let config = SwarmConfig::new(drone_id);

        assert_eq!(config.drone_id, drone_id);
        assert_eq!(config.max_neighbors, 10);
        assert_eq!(config.comm_range, 1000.0);
        assert!(config.encryption_enabled, "Encryption should be enabled by default");
        assert!(config.consensus_enabled, "Consensus should be enabled by default");
        assert!(config.federated_learning_enabled, "FL should be enabled by default");
        assert_eq!(config.heartbeat_interval_ms, 50);
        assert_eq!(config.election_timeout_range, (150, 300));
        assert_eq!(config.max_retries, 3);
        assert_eq!(config.fault_threshold, 3);
    }

    #[test]
    fn test_test_config_has_less_secure_defaults() {
        let drone_id = DroneId::new(2);
        let config = SwarmConfig::test_config(drone_id);

        assert_eq!(config.drone_id, drone_id);
        assert_eq!(config.max_neighbors, 5);
        assert_eq!(config.comm_range, 500.0);
        assert!(!config.encryption_enabled, "Encryption disabled for testing");
        assert!(config.consensus_enabled);
        assert!(!config.federated_learning_enabled, "FL disabled for testing");
        assert_eq!(config.heartbeat_interval_ms, 100);
        assert_eq!(config.election_timeout_range, (200, 400));
        assert_eq!(config.max_retries, 2);
        assert_eq!(config.fault_threshold, 2);
    }

    #[test]
    fn test_validate_accepts_valid_config() {
        let config = SwarmConfig::new(DroneId::new(1));
        assert!(config.validate().is_ok(), "Valid config should pass validation");
    }

    #[test]
    fn test_validate_rejects_zero_neighbors() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.max_neighbors = 0;
        assert!(config.validate().is_err(), "Zero neighbors should be rejected");
    }

    #[test]
    fn test_validate_rejects_too_many_neighbors() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.max_neighbors = MAX_SWARM_SIZE + 1;
        assert!(config.validate().is_err(), "Too many neighbors should be rejected");
    }

    #[test]
    fn test_validate_rejects_negative_comm_range() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.comm_range = -1.0;
        assert!(config.validate().is_err(), "Negative range should be rejected");
    }

    #[test]
    fn test_validate_rejects_zero_comm_range() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.comm_range = 0.0;
        assert!(config.validate().is_err(), "Zero range should be rejected");
    }

    #[test]
    fn test_validate_rejects_zero_heartbeat() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.heartbeat_interval_ms = 0;
        assert!(config.validate().is_err(), "Zero heartbeat should be rejected");
    }

    #[test]
    fn test_validate_rejects_invalid_election_timeout() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.election_timeout_range = (300, 150); // min > max
        assert!(config.validate().is_err(), "Invalid timeout range should be rejected");
    }

    #[test]
    fn test_validate_rejects_equal_election_timeouts() {
        let mut config = SwarmConfig::new(DroneId::new(1));
        config.election_timeout_range = (200, 200); // min == max
        assert!(config.validate().is_err(), "Equal timeouts should be rejected");
    }

    #[test]
    fn test_config_can_be_cloned() {
        let config = SwarmConfig::new(DroneId::new(1));
        let cloned = config.clone();

        assert_eq!(config.drone_id, cloned.drone_id);
        assert_eq!(config.max_neighbors, cloned.max_neighbors);
        assert_eq!(config.comm_range, cloned.comm_range);
    }
}

#[cfg(test)]
mod crypto_config_tests {
    use super::*;

    #[test]
    fn test_crypto_config_new() {
        let key = [42u8; 32];
        let config = CryptoConfig::new(key);

        assert_eq!(config.identity_key, key);
        assert!(!config.post_quantum_enabled, "PQC should be disabled by default");
        assert_eq!(config.key_rotation_interval, 3600);
        assert_eq!(config.nonce_counter, 0);
    }

    #[test]
    fn test_next_nonce_increments_counter() {
        let key = [0u8; 32];
        let mut config = CryptoConfig::new(key);

        let nonce1 = config.next_nonce();
        assert_eq!(config.nonce_counter, 1);

        let nonce2 = config.next_nonce();
        assert_eq!(config.nonce_counter, 2);

        // Nonces should be different
        assert_ne!(nonce1, nonce2);
    }

    #[test]
    fn test_next_nonce_produces_12_bytes() {
        let key = [0u8; 32];
        let mut config = CryptoConfig::new(key);

        let nonce = config.next_nonce();
        assert_eq!(nonce.len(), 12);
    }

    #[test]
    fn test_next_nonce_encodes_counter_in_first_8_bytes() {
        let key = [0u8; 32];
        let mut config = CryptoConfig::new(key);

        let nonce = config.next_nonce();

        // First 8 bytes should be counter (1 in little-endian)
        let counter_bytes = &nonce[..8];
        let decoded_counter = u64::from_le_bytes(counter_bytes.try_into().unwrap());
        assert_eq!(decoded_counter, 1);
    }

    #[test]
    fn test_nonce_uniqueness() {
        let key = [0u8; 32];
        let mut config = CryptoConfig::new(key);

        let mut nonces = Vec::new();
        for _ in 0..100 {
            nonces.push(config.next_nonce());
        }

        // All nonces should be unique
        let mut sorted = nonces.clone();
        sorted.sort();
        sorted.dedup();

        assert_eq!(sorted.len(), nonces.len(), "All nonces should be unique");
    }

    #[test]
    fn test_crypto_config_can_be_cloned() {
        let key = [1u8; 32];
        let config = CryptoConfig::new(key);
        let cloned = config.clone();

        assert_eq!(config.identity_key, cloned.identity_key);
        assert_eq!(config.post_quantum_enabled, cloned.post_quantum_enabled);
        assert_eq!(config.nonce_counter, cloned.nonce_counter);
    }
}

#[cfg(test)]
mod network_config_tests {
    use super::*;

    #[test]
    fn test_network_config_new() {
        let addr = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 1], 8080);
        let config = NetworkConfig::new(addr);

        assert_eq!(config.local_address, addr);
        assert_eq!(config.mtu, 1280, "Should use IPv6 minimum MTU");
        assert!(config.ipv6_enabled, "IPv6 should be enabled by default");
        assert_eq!(config.routing_protocol, RoutingProtocol::AdaptiveMesh);
    }

    #[test]
    fn test_network_config_can_be_cloned() {
        let addr = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 168, 1, 1], 9000);
        let config = NetworkConfig::new(addr);
        let cloned = config.clone();

        assert_eq!(config.local_address, cloned.local_address);
        assert_eq!(config.mtu, cloned.mtu);
        assert_eq!(config.ipv6_enabled, cloned.ipv6_enabled);
    }

    #[test]
    fn test_routing_protocol_equality() {
        assert_eq!(RoutingProtocol::AdaptiveMesh, RoutingProtocol::AdaptiveMesh);
        assert_eq!(RoutingProtocol::Flooding, RoutingProtocol::Flooding);
        assert_eq!(RoutingProtocol::GradientBased, RoutingProtocol::GradientBased);

        assert_ne!(RoutingProtocol::AdaptiveMesh, RoutingProtocol::Flooding);
        assert_ne!(RoutingProtocol::Flooding, RoutingProtocol::GradientBased);
    }

    #[test]
    fn test_routing_protocol_can_be_copied() {
        let proto = RoutingProtocol::AdaptiveMesh;
        let copied = proto;

        assert_eq!(proto, copied);
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_full_system_configuration() {
        let drone_id = DroneId::new(42);
        let swarm_config = SwarmConfig::new(drone_id);
        let crypto_config = CryptoConfig::new([0u8; 32]);
        let addr = NetworkAddress::new([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 42], 8080);
        let network_config = NetworkConfig::new(addr);

        // Validate all configs
        assert!(swarm_config.validate().is_ok());

        // Ensure configs are consistent
        assert_eq!(swarm_config.drone_id, drone_id);
        assert_eq!(network_config.local_address.port, 8080);
        assert_eq!(crypto_config.nonce_counter, 0);
    }

    #[test]
    fn test_test_environment_setup() {
        let drone_id = DroneId::new(1);
        let config = SwarmConfig::test_config(drone_id);

        // Test config should be valid
        assert!(config.validate().is_ok());

        // But less secure
        assert!(!config.encryption_enabled);
        assert!(!config.federated_learning_enabled);
    }
}
