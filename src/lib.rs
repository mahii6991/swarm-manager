//! # Drone Swarm Communication System
//!
//! Ultra-secure, safety-critical drone swarm communication system
//! implementing mesh networking, consensus algorithms, and federated learning.
//!
//! ## Features
//! - Memory-safe Rust implementation for embedded systems
//! - Post-quantum cryptography ready
//! - Decentralized mesh networking
//! - Raft consensus protocol for swarm coordination
//! - Federated learning with blockchain verification
//! - Multi-layer security with defense in depth
//! - Fault tolerance and self-healing
//!
//! ## Safety Guarantees
//! - No heap allocation (suitable for constrained microcontrollers)
//! - Zero-copy message passing
//! - Compile-time memory safety
//! - Formal verification compatible

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(warnings)]
#![allow(missing_docs)] // Gradually adding docs - see Week 2-3 goals
// Standard clippy allows
#![allow(clippy::manual_range_contains)]
#![allow(clippy::needless_borrow)]
#![allow(clippy::excessive_precision)]
#![allow(clippy::manual_find)]
#![allow(clippy::manual_is_multiple_of)]
#![allow(clippy::should_implement_trait)]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::default_trait_access)]
#![allow(clippy::manual_div_ceil)]
// Pedantic clippy allows (style preferences, not bugs)
#![allow(clippy::must_use_candidate)] // Many pure functions don't need #[must_use]
#![allow(clippy::missing_errors_doc)] // Docs will be added in Phase 1 Week 3
#![allow(clippy::missing_panics_doc)] // Docs will be added in Phase 1 Week 3
#![allow(clippy::missing_const_for_fn)] // Const fn not always beneficial
#![allow(clippy::use_self)] // Self vs TypeName is a style preference
#![allow(clippy::wildcard_imports)] // Used for internal module re-exports
#![allow(clippy::cast_precision_loss)] // Intentional f32 casts for embedded
#![allow(clippy::cast_possible_truncation)] // Checked at runtime where needed
#![allow(clippy::cast_sign_loss)] // Checked at runtime where needed
#![allow(clippy::cast_lossless)] // Style preference
#![allow(clippy::suboptimal_flops)] // mul_add not always faster on embedded
#![allow(clippy::items_after_statements)] // Sometimes clearer to define inline
#![allow(clippy::doc_markdown)] // Docs will be improved in Phase 1 Week 3
#![allow(clippy::unused_self)] // Sometimes needed for API consistency
#![allow(clippy::unnecessary_wraps)] // Sometimes needed for API consistency
#![allow(clippy::match_wildcard_for_single_variants)] // Style preference
#![allow(clippy::unreadable_literal)] // Some constants are clearer without separators

/// Ant Colony Optimization (ACO) for path planning and resource allocation
pub mod aco;
/// System configuration and parameter management
pub mod config;
/// Raft consensus protocol implementation for distributed coordination
pub mod consensus;
/// Cryptographic primitives (ChaCha20Poly1305, Ed25519, key management)
pub mod crypto;
/// Fault detection, isolation, and recovery mechanisms
pub mod fault_tolerance;
/// Federated learning with differential privacy and blockchain verification
pub mod federated;
/// Grey Wolf Optimizer (GWO) for multi-objective optimization
pub mod gwo;
/// Mesh networking, routing, and message passing
pub mod network;
/// Particle Swarm Optimization (PSO) for formation control
pub mod pso;
/// Advanced PSO variants with adaptive parameters
pub mod pso_advanced;
/// Cryptographically secure random number generation
pub mod rng;
/// Multi-layer security framework and intrusion detection
pub mod security;
/// High-level swarm coordination and behavior management
pub mod swarm;
/// Platform-agnostic time abstraction for embedded systems
pub mod time_abstraction;
/// Core types (Position, Velocity, DroneId, NetworkAddress, etc.)
pub mod types;

// Re-export configuration types for convenience
pub use config::*;
// Re-export time functions for easy access
pub use time_abstraction::{delay_ms, get_time_ms, get_time_us, init_time_source};
// Re-export core types for convenience
pub use types::*;

/// Maximum number of drones in a swarm.
///
/// This limit ensures bounded memory usage in embedded systems.
/// Increasing this value requires more heap/stack space for neighbor tables,
/// routing information, and consensus state.
pub const MAX_SWARM_SIZE: usize = 100;

/// Maximum message size in bytes.
///
/// Network messages are limited to this size to:
/// - Prevent buffer overflows
/// - Ensure real-time delivery (smaller packets = lower latency)
/// - Work within typical RF module constraints (e.g., LoRa 256-byte payloads)
pub const MAX_MESSAGE_SIZE: usize = 1024;

/// Cryptographic key size in bytes (256-bit).
///
/// Used for:
/// - ChaCha20Poly1305 encryption keys (256-bit)
/// - Ed25519 signing keys (256-bit seed)
/// - HMAC-SHA256 authentication
///
/// 256-bit keys provide ~128-bit security (post-quantum: 64-bit)
pub const KEY_SIZE: usize = 32;

/// Network operation timeout in milliseconds.
///
/// Maximum time to wait for:
/// - Message acknowledgments
/// - Route discoveries
/// - Connection establishment
///
/// Chosen to balance responsiveness vs. packet loss tolerance.
pub const NETWORK_TIMEOUT_MS: u32 = 5000;

/// Raft consensus election timeout in milliseconds.
///
/// If a follower doesn't receive a heartbeat within this time,
/// it starts a new election. Must be >> HEARTBEAT_INTERVAL_MS
/// to avoid spurious elections due to network jitter.
///
/// Typical range: 150-300ms (randomized per node)
pub const ELECTION_TIMEOUT_MS: u32 = 150;

/// Raft leader heartbeat interval in milliseconds.
///
/// Leaders send heartbeats at this interval to maintain authority
/// and prevent unnecessary elections. Must be << ELECTION_TIMEOUT_MS.
///
/// 50ms provides good balance between overhead and responsiveness.
pub const HEARTBEAT_INTERVAL_MS: u32 = 50;

// ═══════════════════════════════════════════════════════════════════════════
// BUG-008 FIX: Platform-Specific Time Abstraction
// ═══════════════════════════════════════════════════════════════════════════
//
// Time functions are now provided by the time_abstraction module.
// See src/time_abstraction.rs for platform-specific implementations:
// - ARM Cortex-M (STM32, nRF52, etc.) via DWT + SysTick
// - ESP32 via esp_timer
// - RISC-V via cycle counter
// - std for development/testing
//
// Initialize at system startup:
//   #[cfg(all(target_arch = "arm", not(feature = "std")))]
//   init_time_source(168_000_000); // CPU frequency in Hz
//
// ═══════════════════════════════════════════════════════════════════════════
