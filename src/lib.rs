#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]

//! Drone Swarm System Core Library
//!
//! A comprehensive library for autonomous drone swarm coordination, communication,
//! and control.

extern crate alloc;

// Core types are available at the top level
pub mod types;
pub use types::*;

// New module structure
pub mod algorithms;
pub mod consensus;
pub mod network;
pub mod control;
pub mod safety;
pub mod ml;
pub mod system;
pub mod extensions;
pub mod hardware;

// Re-exports for convenience (optional, but helps with backward compatibility)
// We might want to be selective here to encourage using the new structure.
// For now, let's expose the main components.

// Algorithms
pub use algorithms::{
    aco, gwo, woa, hybrid, meta_heuristic, selector, rng, pso,
};

// Consensus
pub use consensus::{
    raft, pbft, hierarchical, merkle,
};

// Network
pub use network::{
    core as network_core, mesh, mavlink, esp32, routing,
};

// Control
pub use control::{
    swarm, collision, mission, task, prediction, coordinator,
};

// Safety
pub use safety::{
    security, crypto, failsafe, fault_tolerance, chaos_monkey, injector, scorer,
};

// ML
pub use ml::federated;

// System
pub use system::{
    config, telemetry, time, clustering, hierarchy,
};

// Hardware
pub use hardware::{HardwareRng, RngSource};

// Extensions (commercial/military separation)
pub use extensions::{
    CryptoProvider, TacticalProvider, SecureCommProvider,
    DefaultCryptoProvider, DefaultTacticalProvider,
    ExtensionRegistry,
};

// Feature-gated modules
#[cfg(feature = "std")]
pub mod visualization {
    // Visualization specific code
}

// Global helper functions
pub fn get_time_ms() -> u64 {
    system::time::get_time_ms()
}

pub fn get_time_us() -> u64 {
    system::time::get_time_us()
}

pub fn delay_ms(ms: u32) {
    system::time::delay_ms(ms)
}

pub fn init_time_source(freq: u32) {
    system::time::init_time_source(freq)
}