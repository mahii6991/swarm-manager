//! Hardware Abstraction Layer
//!
//! Platform-specific hardware interfaces for STM32 and ESP32 microcontrollers.
//!
//! # Modules
//!
//! - `rng`: Hardware Random Number Generator
//! - `motor`: Motor controller interface (coming soon)

pub mod rng;

pub use rng::{HardwareRng, RngSource};
