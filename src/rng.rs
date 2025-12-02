//! Cryptographically Secure Random Number Generation
//!
//! Provides platform-specific secure RNG for:
//! - Cryptographic operations (nonces, keys)
//! - Optimization algorithms (PSO, ACO, GWO)
//! - Simulation randomness
//!
//! Uses hardware RNG when available, falls back to getrandom crate.

use crate::types::*;

/// Secure random number generator
pub struct SecureRng {
    // Platform-specific state if needed
    _marker: core::marker::PhantomData<()>,
}

impl SecureRng {
    /// Create a new secure RNG instance
    pub fn new() -> Result<Self> {
        // Verify that getrandom is available
        let mut test_buf = [0u8; 1];
        getrandom::getrandom(&mut test_buf)
            .map_err(|_| SwarmError::CryptoError)?;

        Ok(Self {
            _marker: core::marker::PhantomData,
        })
    }

    /// Generate a random u32
    pub fn next_u32(&mut self) -> Result<u32> {
        let mut buf = [0u8; 4];
        getrandom::getrandom(&mut buf)
            .map_err(|_| SwarmError::CryptoError)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Generate a random u64
    pub fn next_u64(&mut self) -> Result<u64> {
        let mut buf = [0u8; 8];
        getrandom::getrandom(&mut buf)
            .map_err(|_| SwarmError::CryptoError)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Generate a random f32 in range [0.0, 1.0)
    pub fn next_f32(&mut self) -> Result<f32> {
        let val = self.next_u32()?;
        // Use upper 24 bits for mantissa
        Ok((val >> 8) as f32 * (1.0 / 16777216.0))
    }

    /// Generate a random f32 in range [min, max)
    pub fn next_f32_range(&mut self, min: f32, max: f32) -> Result<f32> {
        let t = self.next_f32()?;
        Ok(min + t * (max - min))
    }

    /// Fill buffer with random bytes
    pub fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<()> {
        getrandom::getrandom(dest)
            .map_err(|_| SwarmError::CryptoError)
    }
}

impl Default for SecureRng {
    fn default() -> Self {
        Self::new().expect("Failed to initialize secure RNG")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rng_creation() {
        let rng = SecureRng::new();
        assert!(rng.is_ok());
    }

    #[test]
    fn test_random_u32() {
        let mut rng = SecureRng::new().unwrap();
        let val1 = rng.next_u32().unwrap();
        let val2 = rng.next_u32().unwrap();
        // Very unlikely to be equal
        assert_ne!(val1, val2);
    }

    #[test]
    fn test_random_f32_range() {
        let mut rng = SecureRng::new().unwrap();
        for _ in 0..100 {
            let val = rng.next_f32().unwrap();
            assert!(val >= 0.0 && val < 1.0);
        }
    }

    #[test]
    fn test_random_f32_custom_range() {
        let mut rng = SecureRng::new().unwrap();
        for _ in 0..100 {
            let val = rng.next_f32_range(-10.0, 10.0).unwrap();
            assert!(val >= -10.0 && val < 10.0);
        }
    }

    #[test]
    fn test_fill_bytes() {
        let mut rng = SecureRng::new().unwrap();
        let mut buf1 = [0u8; 32];
        let mut buf2 = [0u8; 32];
        rng.fill_bytes(&mut buf1).unwrap();
        rng.fill_bytes(&mut buf2).unwrap();
        // Very unlikely to be equal
        assert_ne!(buf1, buf2);
    }
}
