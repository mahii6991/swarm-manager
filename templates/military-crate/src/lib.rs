//! Swarm Manager Military Extensions
//!
//! This crate provides military-grade implementations of the extension traits
//! defined in the public swarm-manager crate.
//!
//! # Security Classification
//!
//! This crate contains RESTRICTED/CLASSIFIED implementations.
//! Ensure proper handling in accordance with applicable security policies.
//!
//! # Features
//!
//! - `post-quantum`: Enable post-quantum cryptography (Kyber, Dilithium)
//! - `hsm`: Enable hardware security module support
//! - `tactical-advanced`: Enable advanced tactical algorithms

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

// Re-export extension traits from public crate for convenience
pub use drone_swarm_system::extensions::{
    CryptoProvider, CryptoError,
    TacticalProvider, TacticalError,
    SecureCommProvider, CommError,
    TacticalObjective, ObjectiveType,
    FormationPlan, ThreatAssessment, ThreatType,
    ThreatResponse, ThreatAction, EvasionManeuver,
    SwarmSnapshot, MessagePriority, SecureMessage,
};

pub use drone_swarm_system::types::{DroneId, Position, Velocity};

// Military-specific modules
pub mod crypto;
pub mod tactical;
pub mod comm;

// Re-export main implementations
pub use crypto::PostQuantumCrypto;
pub use tactical::AdvancedTactical;
pub use comm::MilitarySecureComm;

/// Initialize military extensions with default configuration.
///
/// Call this function at application startup to register military
/// implementations with the extension registry.
pub fn init_military_extensions(
    registry: &mut drone_swarm_system::ExtensionRegistry,
) {
    registry.set_crypto_provider(alloc::boxed::Box::new(PostQuantumCrypto::new()));
    registry.set_tactical_provider(alloc::boxed::Box::new(AdvancedTactical::new()));
}

/// Military extension version information.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Security classification level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Classification {
    /// Unclassified but restricted.
    Restricted,
    /// Confidential.
    Confidential,
    /// Secret.
    Secret,
    /// Top Secret.
    TopSecret,
}

/// Get the classification level of this crate.
pub fn classification() -> Classification {
    // Adjust based on actual classification
    Classification::Restricted
}
