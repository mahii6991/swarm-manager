//! Platform-specific time abstraction layer
//!
//! Provides real hardware timer integration for production embedded systems.
//! Supports multiple platforms:
//! - ARM Cortex-M (STM32, nRF52, etc.) via DWT cycle counter
//! - ESP32 via esp_timer
//! - Standard library (Linux/macOS/Windows) for development/testing
//!
//! # Usage
//!
//! ```rust,ignore
//! // At system startup (before any swarm operations)
//! #[cfg(all(target_arch = "arm", not(feature = "std")))]
//! drone_swarm_system::time_abstraction::init_time_source(168_000_000); // 168 MHz CPU
//!
//! // Use anywhere in the system
//! let current_time = drone_swarm_system::get_time_ms();
//! ```
//!
//! # Platform Support
//!
//! ## ARM Cortex-M (STM32, nRF52, etc.)
//! Uses Data Watchpoint and Trace (DWT) cycle counter with SysTick interrupt.
//! Requires CPU frequency to be specified at initialization.
//!
//! ## ESP32
//! Uses native esp_timer_get_time() function (microsecond precision).
//!
//! ## Standard Library (std)
//! Uses SystemTime for development and testing on desktop platforms.

use crate::types::*;
use core::sync::atomic::{AtomicU64, Ordering};

/// Global time counter (milliseconds since initialization)
/// Updated by platform-specific interrupt handlers or direct reads
static GLOBAL_TIME_MS: AtomicU64 = AtomicU64::new(0);

/// Time source trait for dependency injection and testing
pub trait TimeSource: Send + Sync {
    /// Get current time in milliseconds
    fn get_time_ms(&self) -> u64;

    /// Get high-precision timestamp in microseconds
    fn get_time_us(&self) -> u64;

    /// Reset the time counter to zero
    fn reset(&self);
}

// ═══════════════════════════════════════════════════════════════════════════
// ARM Cortex-M Implementation (STM32, nRF52, SAM, etc.)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(all(target_arch = "arm", not(feature = "std")))]
pub struct ArmCortexMTimeSource {
    /// CPU frequency in Hz (e.g., 168_000_000 for 168 MHz)
    cpu_freq_hz: u32,
    /// Cycles per millisecond
    cycles_per_ms: u32,
    /// Cycles per microsecond
    cycles_per_us: u32,
}

#[cfg(all(target_arch = "arm", not(feature = "std")))]
impl ArmCortexMTimeSource {
    /// Create and initialize ARM Cortex-M time source
    ///
    /// # Arguments
    /// * `cpu_freq_hz` - CPU core frequency in Hz
    ///   - STM32F4: 168_000_000 (168 MHz)
    ///   - STM32F7: 216_000_000 (216 MHz)
    ///   - nRF52: 64_000_000 (64 MHz)
    ///
    /// # Safety
    /// This function accesses hardware registers directly. It should be called
    /// once at system initialization before any interrupts are enabled.
    pub fn new(cpu_freq_hz: u32) -> Self {
        unsafe {
            // Memory-mapped register addresses (ARM Cortex-M standard)
            const DWT_CTRL: *mut u32 = 0xE0001000 as *mut u32;
            const DWT_CYCCNT: *mut u32 = 0xE0001004 as *mut u32;
            const SCB_DEMCR: *mut u32 = 0xE000EDFC as *mut u32;
            const SYST_CSR: *mut u32 = 0xE000E010 as *mut u32;
            const SYST_RVR: *mut u32 = 0xE000E014 as *mut u32;
            const SYST_CVR: *mut u32 = 0xE000E018 as *mut u32;

            // Enable trace system (required for DWT)
            let demcr = core::ptr::read_volatile(SCB_DEMCR);
            core::ptr::write_volatile(SCB_DEMCR, demcr | (1 << 24)); // TRCENA bit

            // Enable DWT cycle counter
            let dwt_ctrl = core::ptr::read_volatile(DWT_CTRL);
            core::ptr::write_volatile(DWT_CTRL, dwt_ctrl | 1); // CYCCNTENA bit

            // Reset cycle counter
            core::ptr::write_volatile(DWT_CYCCNT, 0);

            // Configure SysTick for 1ms interrupts
            let reload_value = cpu_freq_hz / 1000 - 1; // 1ms period
            core::ptr::write_volatile(SYST_RVR, reload_value);
            core::ptr::write_volatile(SYST_CVR, 0); // Clear current value

            // Enable SysTick: enable counter + enable interrupt + use processor clock
            core::ptr::write_volatile(SYST_CSR, 0x7);
        }

        Self {
            cpu_freq_hz,
            cycles_per_ms: cpu_freq_hz / 1000,
            cycles_per_us: cpu_freq_hz / 1_000_000,
        }
    }

    /// Read the current DWT cycle counter value
    #[inline(always)]
    fn read_cycle_count(&self) -> u32 {
        unsafe {
            const DWT_CYCCNT: *const u32 = 0xE0001004 as *const u32;
            core::ptr::read_volatile(DWT_CYCCNT)
        }
    }
}

#[cfg(all(target_arch = "arm", not(feature = "std")))]
impl TimeSource for ArmCortexMTimeSource {
    fn get_time_ms(&self) -> u64 {
        // Return the interrupt-driven counter for milliseconds
        // This is updated by SysTick_Handler
        GLOBAL_TIME_MS.load(Ordering::Relaxed)
    }

    fn get_time_us(&self) -> u64 {
        // High precision: combine interrupt counter with cycle counter
        let ms = GLOBAL_TIME_MS.load(Ordering::Relaxed);
        let cycles = self.read_cycle_count();
        let us_in_current_ms = cycles / self.cycles_per_us;

        (ms * 1000) + (us_in_current_ms as u64)
    }

    fn reset(&self) {
        GLOBAL_TIME_MS.store(0, Ordering::Relaxed);
        unsafe {
            const DWT_CYCCNT: *mut u32 = 0xE0001004 as *mut u32;
            core::ptr::write_volatile(DWT_CYCCNT, 0);
        }
    }
}

/// SysTick interrupt handler for ARM Cortex-M
///
/// This must be included in your interrupt vector table.
/// It increments the global millisecond counter every 1ms.
///
/// # Example (cortex-m-rt)
/// ```rust,ignore
/// #[interrupt]
/// fn SysTick() {
///     drone_swarm_system::time_abstraction::systick_handler();
/// }
/// ```
#[cfg(all(target_arch = "arm", not(feature = "std")))]
pub fn systick_handler() {
    GLOBAL_TIME_MS.fetch_add(1, Ordering::Relaxed);
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP32 Implementation (Xtensa architecture)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(target_arch = "xtensa")]
pub struct Esp32TimeSource;

#[cfg(target_arch = "xtensa")]
impl Esp32TimeSource {
    /// Create ESP32 time source
    ///
    /// No initialization needed - uses native esp_timer
    pub fn new() -> Self {
        Self
    }
}

#[cfg(target_arch = "xtensa")]
impl TimeSource for Esp32TimeSource {
    fn get_time_ms(&self) -> u64 {
        unsafe {
            // ESP-IDF provides esp_timer_get_time() which returns microseconds
            extern "C" {
                fn esp_timer_get_time() -> i64;
            }
            (esp_timer_get_time() / 1000) as u64
        }
    }

    fn get_time_us(&self) -> u64 {
        unsafe {
            extern "C" {
                fn esp_timer_get_time() -> i64;
            }
            esp_timer_get_time() as u64
        }
    }

    fn reset(&self) {
        // ESP32 timer cannot be reset - it's a system-wide counter
        // Store offset instead
        let current = self.get_time_ms();
        GLOBAL_TIME_MS.store(current, Ordering::Relaxed);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RISC-V Implementation (for future support)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(all(target_arch = "riscv32", not(feature = "std")))]
pub struct RiscVTimeSource {
    /// Timer frequency in Hz
    timer_freq_hz: u32,
}

#[cfg(all(target_arch = "riscv32", not(feature = "std")))]
impl RiscVTimeSource {
    /// Create RISC-V time source
    pub fn new(timer_freq_hz: u32) -> Self {
        Self { timer_freq_hz }
    }

    /// Read RISC-V cycle counter (CSR)
    #[inline(always)]
    fn read_cycles(&self) -> u64 {
        let cycles: u64;
        unsafe {
            core::arch::asm!(
                "rdcycle {0}",
                out(reg) cycles,
            );
        }
        cycles
    }
}

#[cfg(all(target_arch = "riscv32", not(feature = "std")))]
impl TimeSource for RiscVTimeSource {
    fn get_time_ms(&self) -> u64 {
        let cycles = self.read_cycles();
        (cycles * 1000) / (self.timer_freq_hz as u64)
    }

    fn get_time_us(&self) -> u64 {
        let cycles = self.read_cycles();
        (cycles * 1_000_000) / (self.timer_freq_hz as u64)
    }

    fn reset(&self) {
        // RISC-V cycle counter cannot be reset
        GLOBAL_TIME_MS.store(self.get_time_ms(), Ordering::Relaxed);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Standard Library Implementation (for development/testing)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(feature = "std")]
pub struct StdTimeSource {
    /// Start time (for relative timing)
    start_time: std::time::Instant,
}

#[cfg(feature = "std")]
impl StdTimeSource {
    /// Create standard library time source
    pub fn new() -> Self {
        Self {
            start_time: std::time::Instant::now(),
        }
    }
}

#[cfg(feature = "std")]
impl Default for StdTimeSource {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "std")]
impl TimeSource for StdTimeSource {
    fn get_time_ms(&self) -> u64 {
        self.start_time.elapsed().as_millis() as u64
    }

    fn get_time_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    fn reset(&self) {
        // For std, we use Instant which is always relative
        // Store current time as "zero point"
        GLOBAL_TIME_MS.store(0, Ordering::Relaxed);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Public API - Platform-independent functions
// ═══════════════════════════════════════════════════════════════════════════

/// Initialize the global time source
///
/// # Platform-specific initialization
///
/// ## ARM Cortex-M
/// ```rust,ignore
/// // Call this at startup with your CPU frequency
/// init_time_source(168_000_000); // 168 MHz STM32F4
/// ```
///
/// ## ESP32
/// ```rust,ignore
/// // No initialization needed, but call it for consistency
/// init_time_source(0); // Parameter ignored
/// ```
///
/// ## Standard Library (testing)
/// ```rust,ignore
/// // Automatically initialized, call for consistency
/// init_time_source(0);
/// ```
///
/// # Safety
/// Must be called once before any swarm operations.
/// On ARM Cortex-M, this configures hardware timers.
#[cfg(all(target_arch = "arm", not(feature = "std")))]
pub fn init_time_source(cpu_freq_hz: u32) {
    // Create time source (initializes hardware)
    let _ = ArmCortexMTimeSource::new(cpu_freq_hz);

    // Reset global counter
    GLOBAL_TIME_MS.store(0, Ordering::Relaxed);
}

#[cfg(target_arch = "xtensa")]
pub fn init_time_source(_: u32) {
    // ESP32 time source needs no initialization
    // Timer is always running
}

#[cfg(all(target_arch = "riscv32", not(feature = "std")))]
pub fn init_time_source(timer_freq_hz: u32) {
    // Store timer frequency for later use
    // In a real implementation, this would be stored globally
    GLOBAL_TIME_MS.store(0, Ordering::Relaxed);
}

#[cfg(feature = "std")]
pub fn init_time_source(_: u32) {
    // Standard library time source needs no special initialization
    GLOBAL_TIME_MS.store(0, Ordering::Relaxed);
}

/// Get current time in milliseconds (platform-independent)
///
/// This is the main function used throughout the drone swarm system.
///
/// # Returns
/// Milliseconds since system initialization
///
/// # Examples
/// ```rust
/// use drone_swarm_system::get_time_ms;
///
/// let start = get_time_ms();
/// // ... do some work ...
/// let elapsed = get_time_ms() - start;
/// println!("Operation took {} ms", elapsed);
/// ```
#[inline]
pub fn get_time_ms() -> u64 {
    #[cfg(feature = "std")]
    {
        // For testing/development
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64
    }

    #[cfg(all(target_arch = "arm", not(feature = "std")))]
    {
        // ARM Cortex-M: return interrupt-driven counter
        GLOBAL_TIME_MS.load(Ordering::Relaxed)
    }

    #[cfg(target_arch = "xtensa")]
    {
        // ESP32: read native timer
        unsafe {
            extern "C" {
                fn esp_timer_get_time() -> i64;
            }
            (esp_timer_get_time() / 1000) as u64
        }
    }

    #[cfg(all(target_arch = "riscv32", not(feature = "std")))]
    {
        // RISC-V: calculated from cycle counter
        // This is a simplified version - real implementation would cache timer_freq
        GLOBAL_TIME_MS.load(Ordering::Relaxed)
    }
}

/// Get current time in microseconds (high precision)
///
/// Use this for high-precision timing measurements.
#[inline]
pub fn get_time_us() -> u64 {
    #[cfg(feature = "std")]
    {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64
    }

    #[cfg(all(target_arch = "arm", not(feature = "std")))]
    {
        // Would need to create a static time source instance
        // For now, approximate from milliseconds
        get_time_ms() * 1000
    }

    #[cfg(target_arch = "xtensa")]
    {
        unsafe {
            extern "C" {
                fn esp_timer_get_time() -> i64;
            }
            esp_timer_get_time() as u64
        }
    }

    #[cfg(all(target_arch = "riscv32", not(feature = "std")))]
    {
        get_time_ms() * 1000
    }
}

/// Delay for specified milliseconds (blocking)
///
/// # Warning
/// This is a blocking delay. For non-blocking delays,
/// use the time functions with your async runtime.
#[inline]
pub fn delay_ms(ms: u32) {
    let start = get_time_ms();
    while get_time_ms() - start < ms as u64 {
        // Busy wait
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "std")]
    fn test_time_source_monotonic() {
        let t1 = get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let t2 = get_time_ms();

        assert!(t2 > t1, "Time should be monotonic");
        assert!(t2 - t1 >= 10, "Time should advance by at least 10ms");
    }

    #[test]
    #[cfg(feature = "std")]
    fn test_time_precision() {
        let t1 = get_time_us();
        std::thread::sleep(std::time::Duration::from_micros(100));
        let t2 = get_time_us();

        assert!(t2 > t1, "Microsecond time should be monotonic");
    }

    #[test]
    #[cfg(feature = "std")]
    fn test_delay() {
        let start = get_time_ms();
        delay_ms(50);
        let elapsed = get_time_ms() - start;

        assert!(elapsed >= 50, "Delay should wait at least 50ms");
        assert!(elapsed < 100, "Delay should not wait too long");
    }
}
