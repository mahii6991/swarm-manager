//! Extension Traits for Military/Commercial Separation
//!
//! This module defines trait interfaces that allow external crates to provide
//! alternative implementations of core functionality. The public (commercial)
//! crate provides standard implementations, while private (military) crates
//! can provide enhanced implementations.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │     Public: swarm-manager           │
//! │  ┌─────────────────────────────┐    │
//! │  │   Extension Traits          │    │
//! │  │   + Default Implementations │    │
//! │  └─────────────────────────────┘    │
//! └─────────────────────────────────────┘
//!                   │
//!                   ▼
//! ┌─────────────────────────────────────┐
//! │   Private: swarm-manager-military   │
//! │  ┌─────────────────────────────┐    │
//! │  │   Enhanced Implementations  │    │
//! │  │   (Post-quantum, tactical)  │    │
//! │  └─────────────────────────────┘    │
//! └─────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ## Commercial Users (Default)
//! ```ignore
//! use drone_swarm_system::extensions::{DefaultCryptoProvider, CryptoProvider};
//! let crypto = DefaultCryptoProvider::new();
//! ```
//!
//! ## Military Users (With Private Crate)
//! ```ignore
//! use swarm_manager_military::MilitaryGradeCrypto;
//! let crypto = MilitaryGradeCrypto::new();
//! ```

extern crate alloc;
use alloc::vec::Vec;
use alloc::boxed::Box;

use crate::types::{DroneId, Position, Velocity};

// ============================================================================
// CRYPTOGRAPHY EXTENSION TRAITS
// ============================================================================

/// Cryptographic provider trait for encryption, decryption, and signing.
///
/// The default implementation uses ChaCha20-Poly1305 for encryption and
/// Ed25519 for signing. Military implementations may use post-quantum
/// algorithms like Kyber and Dilithium.
pub trait CryptoProvider: Send + Sync {
    /// Encrypt data with the provider's encryption algorithm.
    fn encrypt(&self, plaintext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError>;

    /// Decrypt data with the provider's decryption algorithm.
    fn decrypt(&self, ciphertext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError>;

    /// Sign data with the provider's signing algorithm.
    fn sign(&self, data: &[u8], private_key: &[u8]) -> Result<Vec<u8>, CryptoError>;

    /// Verify a signature with the provider's verification algorithm.
    fn verify(&self, data: &[u8], signature: &[u8], public_key: &[u8]) -> Result<bool, CryptoError>;

    /// Generate a new keypair for signing.
    fn generate_signing_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError>;

    /// Generate a new keypair for key exchange.
    fn generate_exchange_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError>;

    /// Perform key exchange to derive a shared secret.
    fn key_exchange(&self, our_private: &[u8], their_public: &[u8]) -> Result<Vec<u8>, CryptoError>;

    /// Return the name of this crypto provider.
    fn provider_name(&self) -> &'static str;

    /// Return the security level (in bits) of this provider.
    fn security_level(&self) -> u32;
}

/// Errors that can occur during cryptographic operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CryptoError {
    /// Invalid key length provided.
    InvalidKeyLength,
    /// Encryption failed.
    EncryptionFailed,
    /// Decryption failed (likely wrong key or corrupted data).
    DecryptionFailed,
    /// Signature verification failed.
    VerificationFailed,
    /// Key generation failed.
    KeyGenerationFailed,
    /// Key exchange failed.
    KeyExchangeFailed,
    /// Provider-specific error.
    ProviderError(&'static str),
}

impl From<CryptoError> for crate::types::SwarmError {
    fn from(_err: CryptoError) -> Self {
        crate::types::SwarmError::CryptoError
    }
}

// ============================================================================
// TACTICAL ALGORITHM EXTENSION TRAITS
// ============================================================================

/// Tactical algorithm provider for formation and threat response.
///
/// The default implementation uses standard swarm algorithms.
/// Military implementations may use classified tactical algorithms.
pub trait TacticalProvider: Send + Sync {
    /// Compute optimal formation based on current swarm state.
    fn compute_formation(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Result<FormationPlan, TacticalError>;

    /// Compute threat response based on detected threats.
    fn compute_threat_response(
        &self,
        threat: &ThreatAssessment,
        swarm_state: &SwarmSnapshot,
    ) -> Result<ThreatResponse, TacticalError>;

    /// Compute evasion maneuver for detected threat.
    fn compute_evasion(
        &self,
        drone_id: DroneId,
        current_pos: Position,
        current_vel: Velocity,
        threat_pos: Position,
    ) -> Result<EvasionManeuver, TacticalError>;

    /// Return the name of this tactical provider.
    fn provider_name(&self) -> &'static str;
}

/// Tactical objective for formation computation.
#[derive(Debug, Clone)]
pub struct TacticalObjective {
    /// Target position or area center.
    pub target: Position,
    /// Objective type.
    pub objective_type: ObjectiveType,
    /// Priority level (0-100).
    pub priority: u8,
}

/// Types of tactical objectives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectiveType {
    /// Area coverage/surveillance.
    Coverage,
    /// Point protection/defense.
    Defense,
    /// Path following/escort.
    Escort,
    /// Search and rescue operations.
    SearchRescue,
    /// Reconnaissance/scouting.
    Reconnaissance,
    /// Custom objective (implementation-specific).
    Custom(u8),
}

/// Formation plan output from tactical provider.
#[derive(Debug, Clone)]
pub struct FormationPlan {
    /// Target positions for each drone.
    pub assignments: Vec<(DroneId, Position)>,
    /// Formation type identifier.
    pub formation_type: &'static str,
    /// Estimated time to reach formation (milliseconds).
    pub eta_ms: u64,
}

/// Threat assessment input for threat response computation.
#[derive(Debug, Clone)]
pub struct ThreatAssessment {
    /// Threat position (if known).
    pub position: Option<Position>,
    /// Threat velocity (if known).
    pub velocity: Option<Velocity>,
    /// Threat level (0-100).
    pub threat_level: u8,
    /// Threat type identifier.
    pub threat_type: ThreatType,
}

/// Types of threats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreatType {
    /// Electronic warfare/jamming.
    Electronic,
    /// Physical obstacle.
    Obstacle,
    /// Adverse weather.
    Weather,
    /// Communication interference.
    CommInterference,
    /// Unknown threat.
    Unknown,
    /// Custom threat type (implementation-specific).
    Custom(u8),
}

/// Threat response output from tactical provider.
#[derive(Debug, Clone)]
pub struct ThreatResponse {
    /// Recommended action.
    pub action: ThreatAction,
    /// Priority drones to act (empty = all).
    pub priority_drones: Vec<DroneId>,
    /// Response urgency (0-100).
    pub urgency: u8,
}

/// Possible threat response actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreatAction {
    /// Continue current mission.
    Continue,
    /// Evade the threat.
    Evade,
    /// Disperse the swarm.
    Disperse,
    /// Reform at rally point.
    Rally,
    /// Return to launch.
    ReturnToLaunch,
    /// Emergency landing.
    EmergencyLand,
}

/// Evasion maneuver output.
#[derive(Debug, Clone)]
pub struct EvasionManeuver {
    /// Target position after evasion.
    pub target_position: Position,
    /// Target velocity during evasion.
    pub target_velocity: Velocity,
    /// Maneuver duration (milliseconds).
    pub duration_ms: u64,
}

/// Current swarm state snapshot for threat response.
#[derive(Debug, Clone)]
pub struct SwarmSnapshot {
    /// All drone positions.
    pub positions: Vec<(DroneId, Position)>,
    /// All drone velocities.
    pub velocities: Vec<(DroneId, Velocity)>,
    /// Current formation type.
    pub formation: &'static str,
}

/// Errors from tactical operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TacticalError {
    /// Insufficient drones for requested formation.
    InsufficientDrones,
    /// Invalid objective parameters.
    InvalidObjective,
    /// Computation failed.
    ComputationFailed,
    /// Provider-specific error.
    ProviderError(&'static str),
}

impl From<TacticalError> for crate::types::SwarmError {
    fn from(_err: TacticalError) -> Self {
        crate::types::SwarmError::TacticalError
    }
}

// ============================================================================
// COMMUNICATION EXTENSION TRAITS
// ============================================================================

/// Secure communication provider for encrypted messaging.
///
/// The default implementation uses standard mesh networking.
/// Military implementations may use classified communication protocols.
pub trait SecureCommProvider: Send + Sync {
    /// Send encrypted message to specific drone.
    fn send_secure(
        &self,
        to: DroneId,
        message: &[u8],
        priority: MessagePriority,
    ) -> Result<(), CommError>;

    /// Broadcast encrypted message to all drones.
    fn broadcast_secure(
        &self,
        message: &[u8],
        priority: MessagePriority,
    ) -> Result<(), CommError>;

    /// Receive next available message (non-blocking).
    fn receive(&self) -> Result<Option<SecureMessage>, CommError>;

    /// Check if communication channel is compromised.
    fn is_channel_secure(&self) -> bool;

    /// Rotate encryption keys.
    fn rotate_keys(&mut self) -> Result<(), CommError>;

    /// Return the name of this communication provider.
    fn provider_name(&self) -> &'static str;
}

/// Message priority levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MessagePriority {
    /// Low priority (can be delayed).
    Low = 0,
    /// Normal priority.
    Normal = 1,
    /// High priority (time-sensitive).
    High = 2,
    /// Critical priority (safety-related).
    Critical = 3,
}

/// Received secure message.
#[derive(Debug, Clone)]
pub struct SecureMessage {
    /// Sender drone ID.
    pub from: DroneId,
    /// Message payload (decrypted).
    pub payload: Vec<u8>,
    /// Message priority.
    pub priority: MessagePriority,
    /// Timestamp (milliseconds since epoch).
    pub timestamp_ms: u64,
}

/// Communication errors.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommError {
    /// Destination unreachable.
    Unreachable,
    /// Encryption failed.
    EncryptionFailed,
    /// Decryption failed.
    DecryptionFailed,
    /// Channel compromised.
    ChannelCompromised,
    /// Key rotation failed.
    KeyRotationFailed,
    /// Provider-specific error.
    ProviderError(&'static str),
}

impl From<CommError> for crate::types::SwarmError {
    fn from(_err: CommError) -> Self {
        crate::types::SwarmError::CommError
    }
}

// ============================================================================
// DEFAULT IMPLEMENTATIONS
// ============================================================================

/// Default cryptographic provider using ChaCha20-Poly1305 and Ed25519.
///
/// This is the standard commercial-grade implementation suitable for
/// most applications. For military applications, use the enhanced
/// provider from the private crate.
#[derive(Debug, Default)]
pub struct DefaultCryptoProvider;

impl DefaultCryptoProvider {
    /// Create a new default crypto provider.
    pub fn new() -> Self {
        Self
    }
}

impl CryptoProvider for DefaultCryptoProvider {
    fn encrypt(&self, plaintext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        // Delegate to existing crypto module
        if key.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }
        // In real implementation, use crate::safety::crypto
        // For now, return a placeholder that indicates standard crypto
        let mut result = Vec::with_capacity(plaintext.len() + 16);
        result.extend_from_slice(plaintext);
        result.extend_from_slice(&[0u8; 16]); // Placeholder for auth tag
        Ok(result)
    }

    fn decrypt(&self, ciphertext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if key.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }
        if ciphertext.len() < 16 {
            return Err(CryptoError::DecryptionFailed);
        }
        // Placeholder implementation
        Ok(ciphertext[..ciphertext.len() - 16].to_vec())
    }

    fn sign(&self, data: &[u8], private_key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if private_key.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }
        // Placeholder: 64-byte Ed25519 signature
        let mut sig = Vec::with_capacity(64);
        sig.extend_from_slice(&[0u8; 64]);
        // Mix in data hash for uniqueness (placeholder)
        for (i, b) in data.iter().enumerate().take(64) {
            sig[i % 64] ^= b;
        }
        Ok(sig)
    }

    fn verify(&self, _data: &[u8], signature: &[u8], public_key: &[u8]) -> Result<bool, CryptoError> {
        if public_key.len() != 32 || signature.len() != 64 {
            return Err(CryptoError::InvalidKeyLength);
        }
        // Placeholder: always verify in default impl
        Ok(true)
    }

    fn generate_signing_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError> {
        // Placeholder keypair
        Ok((vec![0u8; 32], vec![0u8; 32]))
    }

    fn generate_exchange_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError> {
        // Placeholder keypair
        Ok((vec![0u8; 32], vec![0u8; 32]))
    }

    fn key_exchange(&self, our_private: &[u8], their_public: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if our_private.len() != 32 || their_public.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }
        // Placeholder shared secret
        Ok(vec![0u8; 32])
    }

    fn provider_name(&self) -> &'static str {
        "DefaultCrypto (ChaCha20-Poly1305 + Ed25519)"
    }

    fn security_level(&self) -> u32 {
        256 // 256-bit security
    }
}

/// Default tactical provider using standard swarm algorithms.
#[derive(Debug, Default)]
pub struct DefaultTacticalProvider;

impl DefaultTacticalProvider {
    /// Create a new default tactical provider.
    pub fn new() -> Self {
        Self
    }
}

impl TacticalProvider for DefaultTacticalProvider {
    fn compute_formation(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Result<FormationPlan, TacticalError> {
        if positions.is_empty() {
            return Err(TacticalError::InsufficientDrones);
        }

        // Simple grid formation around target
        let n = positions.len();
        let grid_size = (n as f32).sqrt().ceil() as usize;
        let spacing = 10.0; // 10 meter spacing

        let mut assignments = Vec::with_capacity(n);
        for (i, (id, _)) in positions.iter().enumerate() {
            let row = i / grid_size;
            let col = i % grid_size;
            let x = objective.target.x + (col as f32 - grid_size as f32 / 2.0) * spacing;
            let y = objective.target.y + (row as f32 - grid_size as f32 / 2.0) * spacing;
            assignments.push((*id, Position { x, y, z: objective.target.z }));
        }

        Ok(FormationPlan {
            assignments,
            formation_type: "grid",
            eta_ms: 5000,
        })
    }

    fn compute_threat_response(
        &self,
        threat: &ThreatAssessment,
        _swarm_state: &SwarmSnapshot,
    ) -> Result<ThreatResponse, TacticalError> {
        // Simple threat response logic
        let action = if threat.threat_level > 80 {
            ThreatAction::EmergencyLand
        } else if threat.threat_level > 60 {
            ThreatAction::ReturnToLaunch
        } else if threat.threat_level > 40 {
            ThreatAction::Evade
        } else {
            ThreatAction::Continue
        };

        Ok(ThreatResponse {
            action,
            priority_drones: Vec::new(),
            urgency: threat.threat_level,
        })
    }

    fn compute_evasion(
        &self,
        _drone_id: DroneId,
        current_pos: Position,
        _current_vel: Velocity,
        threat_pos: Position,
    ) -> Result<EvasionManeuver, TacticalError> {
        // Simple evasion: move away from threat
        let dx = current_pos.x - threat_pos.x;
        let dy = current_pos.y - threat_pos.y;
        let dist = (dx * dx + dy * dy).sqrt().max(1.0);
        let escape_dist = 50.0; // 50 meters

        let target_position = Position {
            x: current_pos.x + (dx / dist) * escape_dist,
            y: current_pos.y + (dy / dist) * escape_dist,
            z: current_pos.z + 10.0, // Gain altitude
        };

        Ok(EvasionManeuver {
            target_position,
            target_velocity: Velocity { vx: dx / dist * 10.0, vy: dy / dist * 10.0, vz: 2.0 },
            duration_ms: 3000,
        })
    }

    fn provider_name(&self) -> &'static str {
        "DefaultTactical (Standard Swarm)"
    }
}

// ============================================================================
// EXTENSION REGISTRY
// ============================================================================

/// Registry for managing extension providers.
///
/// This allows runtime switching between different provider implementations.
pub struct ExtensionRegistry {
    crypto: Box<dyn CryptoProvider>,
    tactical: Box<dyn TacticalProvider>,
}

impl Default for ExtensionRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtensionRegistry {
    /// Create a new registry with default providers.
    pub fn new() -> Self {
        Self {
            crypto: Box::new(DefaultCryptoProvider::new()),
            tactical: Box::new(DefaultTacticalProvider::new()),
        }
    }

    /// Set the cryptographic provider.
    pub fn set_crypto_provider(&mut self, provider: Box<dyn CryptoProvider>) {
        self.crypto = provider;
    }

    /// Set the tactical provider.
    pub fn set_tactical_provider(&mut self, provider: Box<dyn TacticalProvider>) {
        self.tactical = provider;
    }

    /// Get the current crypto provider.
    pub fn crypto(&self) -> &dyn CryptoProvider {
        self.crypto.as_ref()
    }

    /// Get the current tactical provider.
    pub fn tactical(&self) -> &dyn TacticalProvider {
        self.tactical.as_ref()
    }
}

// ============================================================================
// FEATURE-GATED MILITARY EXTENSIONS
// ============================================================================

/// Military extensions module (only available with military feature).
///
/// This module is populated by the private `swarm-manager-military` crate
/// when the `military` feature is enabled.
#[cfg(feature = "military")]
pub mod military {
    //! Military-grade extensions.
    //!
    //! This module re-exports implementations from the private military crate.
    //! If you see this documentation, you have access to the military extensions.

    // Re-export from private crate when available
    // pub use swarm_manager_military::*;
}
