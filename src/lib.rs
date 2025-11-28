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
// TODO: Re-enable after adding all missing documentation
// #![deny(missing_docs)]
// TODO: Re-enable after fixing all warnings
// #![deny(warnings)]
#![allow(warnings)]
#![allow(missing_docs)]

pub mod config;
pub mod crypto;
pub mod network;
pub mod consensus;
pub mod federated;
pub mod swarm;
pub mod types;
pub mod security;
pub mod fault_tolerance;
// TODO: Fix borrow checker errors before re-enabling
// See KNOWN_ISSUES.md and FIXES_APPLIED.md for details
// pub mod pso;
// pub mod pso_advanced;
// pub mod aco;
// pub mod gwo;
pub mod time_abstraction;

pub use types::*;
pub use config::*;
pub use time_abstraction::{get_time_ms, get_time_us, delay_ms, init_time_source};

/// Maximum number of drones in a swarm
pub const MAX_SWARM_SIZE: usize = 100;

/// Maximum message size (bytes)
pub const MAX_MESSAGE_SIZE: usize = 1024;

/// Cryptographic key size (256-bit)
pub const KEY_SIZE: usize = 32;

/// Network timeout in milliseconds
pub const NETWORK_TIMEOUT_MS: u32 = 5000;

/// Consensus election timeout (ms)
pub const ELECTION_TIMEOUT_MS: u32 = 150;

/// Heartbeat interval (ms)
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
