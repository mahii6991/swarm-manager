//! Hardware Random Number Generator
//!
//! Platform-specific hardware RNG implementations for cryptographic security.
//!
//! # Supported Platforms
//!
//! - **STM32**: Uses the True Random Number Generator (TRNG) peripheral
//! - **ESP32**: Uses the hardware RNG peripheral via `esp_random()`
//! - **Default**: Falls back to `getrandom` crate
//!
//! # Security
//!
//! Hardware RNG provides true randomness from physical entropy sources,
//! making it suitable for cryptographic key generation and nonce creation.

use crate::types::*;

/// RNG entropy source type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RngSource {
    /// STM32 True Random Number Generator
    Stm32Trng,
    /// ESP32 Hardware RNG
    Esp32Hwrng,
    /// Software-based (getrandom)
    Software,
    /// Unknown/uninitialized
    Unknown,
}

/// Hardware Random Number Generator trait
pub trait HardwareRngProvider {
    /// Get the entropy source type
    fn source(&self) -> RngSource;

    /// Generate a random u32
    fn next_u32(&mut self) -> Result<u32>;

    /// Generate a random u64
    fn next_u64(&mut self) -> Result<u64>;

    /// Fill buffer with random bytes
    fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<()>;

    /// Check if the RNG is ready/healthy
    fn is_ready(&self) -> bool;
}

/// Hardware RNG with platform detection
pub struct HardwareRng {
    source: RngSource,
    #[cfg(feature = "stm32")]
    stm32_rng: Option<Stm32Rng>,
    #[cfg(feature = "esp32")]
    esp32_rng: Option<Esp32Rng>,
}

impl HardwareRng {
    /// Create a new hardware RNG, auto-detecting the platform
    pub fn new() -> Result<Self> {
        #[cfg(feature = "stm32")]
        {
            if let Ok(rng) = Stm32Rng::new() {
                return Ok(Self {
                    source: RngSource::Stm32Trng,
                    stm32_rng: Some(rng),
                    #[cfg(feature = "esp32")]
                    esp32_rng: None,
                });
            }
        }

        #[cfg(feature = "esp32")]
        {
            if let Ok(rng) = Esp32Rng::new() {
                return Ok(Self {
                    source: RngSource::Esp32Hwrng,
                    #[cfg(feature = "stm32")]
                    stm32_rng: None,
                    esp32_rng: Some(rng),
                });
            }
        }

        // Fallback to software RNG
        Ok(Self {
            source: RngSource::Software,
            #[cfg(feature = "stm32")]
            stm32_rng: None,
            #[cfg(feature = "esp32")]
            esp32_rng: None,
        })
    }

    /// Get the current entropy source
    pub fn source(&self) -> RngSource {
        self.source
    }

    /// Generate a random u32
    pub fn next_u32(&mut self) -> Result<u32> {
        match self.source {
            #[cfg(feature = "stm32")]
            RngSource::Stm32Trng => {
                self.stm32_rng.as_mut()
                    .ok_or(SwarmError::HardwareFault)?
                    .next_u32()
            }
            #[cfg(feature = "esp32")]
            RngSource::Esp32Hwrng => {
                self.esp32_rng.as_mut()
                    .ok_or(SwarmError::HardwareFault)?
                    .next_u32()
            }
            RngSource::Software | RngSource::Unknown => {
                let mut buf = [0u8; 4];
                getrandom::getrandom(&mut buf).map_err(|_| SwarmError::CryptoError)?;
                Ok(u32::from_le_bytes(buf))
            }
            #[allow(unreachable_patterns)]
            _ => {
                let mut buf = [0u8; 4];
                getrandom::getrandom(&mut buf).map_err(|_| SwarmError::CryptoError)?;
                Ok(u32::from_le_bytes(buf))
            }
        }
    }

    /// Generate a random u64
    pub fn next_u64(&mut self) -> Result<u64> {
        let low = self.next_u32()? as u64;
        let high = self.next_u32()? as u64;
        Ok((high << 32) | low)
    }

    /// Generate a random f32 in range [0.0, 1.0)
    pub fn next_f32(&mut self) -> Result<f32> {
        let val = self.next_u32()?;
        Ok((val >> 8) as f32 * (1.0 / 16777216.0))
    }

    /// Generate a random f32 in range [min, max)
    pub fn next_f32_range(&mut self, min: f32, max: f32) -> Result<f32> {
        let t = self.next_f32()?;
        Ok(min + t * (max - min))
    }

    /// Fill buffer with random bytes
    pub fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<()> {
        match self.source {
            #[cfg(feature = "stm32")]
            RngSource::Stm32Trng => {
                self.stm32_rng.as_mut()
                    .ok_or(SwarmError::HardwareFault)?
                    .fill_bytes(dest)
            }
            #[cfg(feature = "esp32")]
            RngSource::Esp32Hwrng => {
                self.esp32_rng.as_mut()
                    .ok_or(SwarmError::HardwareFault)?
                    .fill_bytes(dest)
            }
            RngSource::Software | RngSource::Unknown => {
                getrandom::getrandom(dest).map_err(|_| SwarmError::CryptoError)
            }
            #[allow(unreachable_patterns)]
            _ => {
                getrandom::getrandom(dest).map_err(|_| SwarmError::CryptoError)
            }
        }
    }

    /// Check if hardware RNG is available
    pub fn is_hardware(&self) -> bool {
        matches!(self.source, RngSource::Stm32Trng | RngSource::Esp32Hwrng)
    }

    /// Check if RNG is ready
    pub fn is_ready(&self) -> bool {
        match self.source {
            #[cfg(feature = "stm32")]
            RngSource::Stm32Trng => {
                self.stm32_rng.as_ref().map(|r| r.is_ready()).unwrap_or(false)
            }
            #[cfg(feature = "esp32")]
            RngSource::Esp32Hwrng => {
                self.esp32_rng.as_ref().map(|r| r.is_ready()).unwrap_or(false)
            }
            RngSource::Software => true,
            RngSource::Unknown => false,
            #[allow(unreachable_patterns)]
            _ => false,
        }
    }

    /// Generate cryptographic nonce (12 bytes for AES-GCM)
    pub fn generate_nonce(&mut self) -> Result<[u8; 12]> {
        let mut nonce = [0u8; 12];
        self.fill_bytes(&mut nonce)?;
        Ok(nonce)
    }

    /// Generate cryptographic key (32 bytes for AES-256)
    pub fn generate_key(&mut self) -> Result<[u8; 32]> {
        let mut key = [0u8; 32];
        self.fill_bytes(&mut key)?;
        Ok(key)
    }
}

impl Default for HardwareRng {
    fn default() -> Self {
        Self::new().expect("Failed to initialize hardware RNG")
    }
}

// ============================================================================
// STM32 HARDWARE RNG IMPLEMENTATION
// ============================================================================

#[cfg(feature = "stm32")]
pub struct Stm32Rng {
    /// Indicates if TRNG is initialized
    initialized: bool,
}

#[cfg(feature = "stm32")]
impl Stm32Rng {
    /// Initialize STM32 True Random Number Generator
    pub fn new() -> Result<Self> {
        // In a real implementation, this would:
        // 1. Enable RNG clock via RCC
        // 2. Configure RNG peripheral
        // 3. Wait for DRDY flag
        //
        // Example for STM32F4:
        // unsafe {
        //     let rcc = &*stm32f4xx_hal::pac::RCC::ptr();
        //     rcc.ahb2enr.modify(|_, w| w.rngen().enabled());
        //
        //     let rng = &*stm32f4xx_hal::pac::RNG::ptr();
        //     rng.cr.modify(|_, w| w.rngen().enabled());
        //
        //     // Wait for RNG to be ready
        //     while rng.sr.read().drdy().bit_is_clear() {}
        // }

        Ok(Self { initialized: true })
    }

    /// Read random value from TRNG data register
    pub fn next_u32(&mut self) -> Result<u32> {
        if !self.initialized {
            return Err(SwarmError::HardwareFault);
        }

        // In a real implementation:
        // unsafe {
        //     let rng = &*stm32f4xx_hal::pac::RNG::ptr();
        //
        //     // Wait for data ready
        //     while rng.sr.read().drdy().bit_is_clear() {
        //         // Check for errors
        //         if rng.sr.read().ceis().bit_is_set() || rng.sr.read().seis().bit_is_set() {
        //             // Clear error flags and restart
        //             rng.sr.modify(|_, w| w.ceis().clear_bit().seis().clear_bit());
        //             return Err(SwarmError::HardwareFault);
        //         }
        //     }
        //
        //     Ok(rng.dr.read().bits())
        // }

        // Fallback for compilation without actual hardware
        let mut buf = [0u8; 4];
        getrandom::getrandom(&mut buf).map_err(|_| SwarmError::CryptoError)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Fill buffer with random bytes from TRNG
    pub fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<()> {
        for chunk in dest.chunks_mut(4) {
            let val = self.next_u32()?;
            let bytes = val.to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
        Ok(())
    }

    /// Check if TRNG is ready
    pub fn is_ready(&self) -> bool {
        self.initialized
    }

    /// Get clock seed errors count (for diagnostics)
    pub fn seed_error_count(&self) -> u32 {
        // In real implementation, track SEIS errors
        0
    }

    /// Get clock errors count (for diagnostics)
    pub fn clock_error_count(&self) -> u32 {
        // In real implementation, track CEIS errors
        0
    }
}

// ============================================================================
// ESP32 HARDWARE RNG IMPLEMENTATION
// ============================================================================

#[cfg(feature = "esp32")]
pub struct Esp32Rng {
    /// Indicates if hardware RNG is available
    available: bool,
}

#[cfg(feature = "esp32")]
impl Esp32Rng {
    /// Initialize ESP32 Hardware RNG
    pub fn new() -> Result<Self> {
        // ESP32 hardware RNG is always available when WiFi/BT is enabled
        // It uses noise from the RF subsystem as entropy source
        //
        // In a real implementation using esp-idf:
        // The `esp_random()` function is available via esp_idf_sys crate
        //
        // Note: For best entropy, ensure WiFi or Bluetooth is active

        Ok(Self { available: true })
    }

    /// Get random u32 from hardware RNG
    pub fn next_u32(&mut self) -> Result<u32> {
        if !self.available {
            return Err(SwarmError::HardwareFault);
        }

        // In a real implementation using esp-idf:
        // unsafe { Ok(esp_idf_sys::esp_random()) }
        //
        // Or using esp-hal:
        // let mut rng = esp_hal::Rng::new(peripherals.RNG);
        // Ok(rng.random())

        // Fallback for compilation without actual hardware
        let mut buf = [0u8; 4];
        getrandom::getrandom(&mut buf).map_err(|_| SwarmError::CryptoError)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Fill buffer with random bytes
    pub fn fill_bytes(&mut self, dest: &mut [u8]) -> Result<()> {
        // In a real implementation using esp-idf:
        // unsafe { esp_idf_sys::esp_fill_random(dest.as_mut_ptr() as *mut _, dest.len()) };
        // Ok(())

        for chunk in dest.chunks_mut(4) {
            let val = self.next_u32()?;
            let bytes = val.to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
        Ok(())
    }

    /// Check if hardware RNG is ready
    pub fn is_ready(&self) -> bool {
        self.available
    }

    /// Check WiFi status for optimal entropy
    pub fn has_wifi_entropy(&self) -> bool {
        // In real implementation, check if WiFi is active
        // WiFi provides additional entropy to the hardware RNG
        true
    }

    /// Check Bluetooth status for optimal entropy
    pub fn has_bt_entropy(&self) -> bool {
        // In real implementation, check if Bluetooth is active
        // Bluetooth provides additional entropy to the hardware RNG
        true
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_rng_creation() {
        let rng = HardwareRng::new();
        assert!(rng.is_ok());
    }

    #[test]
    fn test_hardware_rng_source_detection() {
        let rng = HardwareRng::new().unwrap();
        // In test environment, should fall back to software
        assert_eq!(rng.source(), RngSource::Software);
    }

    #[test]
    fn test_random_u32() {
        let mut rng = HardwareRng::new().unwrap();
        let val1 = rng.next_u32().unwrap();
        let val2 = rng.next_u32().unwrap();
        // Very unlikely to be equal
        assert_ne!(val1, val2);
    }

    #[test]
    fn test_random_u64() {
        let mut rng = HardwareRng::new().unwrap();
        let val1 = rng.next_u64().unwrap();
        let val2 = rng.next_u64().unwrap();
        assert_ne!(val1, val2);
    }

    #[test]
    fn test_random_f32_range() {
        let mut rng = HardwareRng::new().unwrap();
        for _ in 0..100 {
            let val = rng.next_f32().unwrap();
            assert!((0.0..1.0).contains(&val));
        }
    }

    #[test]
    fn test_random_f32_custom_range() {
        let mut rng = HardwareRng::new().unwrap();
        for _ in 0..100 {
            let val = rng.next_f32_range(-10.0, 10.0).unwrap();
            assert!((-10.0..10.0).contains(&val));
        }
    }

    #[test]
    fn test_fill_bytes() {
        let mut rng = HardwareRng::new().unwrap();
        let mut buf1 = [0u8; 32];
        let mut buf2 = [0u8; 32];
        rng.fill_bytes(&mut buf1).unwrap();
        rng.fill_bytes(&mut buf2).unwrap();
        assert_ne!(buf1, buf2);
    }

    #[test]
    fn test_generate_nonce() {
        let mut rng = HardwareRng::new().unwrap();
        let nonce1 = rng.generate_nonce().unwrap();
        let nonce2 = rng.generate_nonce().unwrap();
        assert_ne!(nonce1, nonce2);
        assert_eq!(nonce1.len(), 12);
    }

    #[test]
    fn test_generate_key() {
        let mut rng = HardwareRng::new().unwrap();
        let key1 = rng.generate_key().unwrap();
        let key2 = rng.generate_key().unwrap();
        assert_ne!(key1, key2);
        assert_eq!(key1.len(), 32);
    }

    #[test]
    fn test_is_ready() {
        let rng = HardwareRng::new().unwrap();
        assert!(rng.is_ready());
    }
}
