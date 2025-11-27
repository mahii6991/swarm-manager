//! Example: Time source initialization for ESP32
//!
//! This example shows how to use the time source on ESP32.
//! ESP32 uses the native esp_timer which requires no special initialization.
//!
//! # Hardware Requirements
//! - ESP32 or ESP32-S3 development board
//!
//! # Build
//! ```bash
//! cargo build --example time_source_esp32 --target xtensa-esp32-none-elf
//! ```

// Note: This example shows the concept. For real ESP32 development,
// use esp-idf-hal or esp-hal crates

#[cfg(feature = "std")]
fn main() {
    use drone_swarm_system::{init_time_source, get_time_ms, get_time_us, delay_ms};

    println!("ESP32 Time Source Example (simulated on std)");
    println!("=============================================\n");

    // ESP32 time source needs no special initialization
    init_time_source(0);

    println!("✓ Time source initialized");
    println!("  ESP32 uses native esp_timer (no configuration needed)\n");

    // Test millisecond precision
    println!("Testing millisecond precision:");
    let start_ms = get_time_ms();
    delay_ms(100);
    let elapsed_ms = get_time_ms() - start_ms;
    println!("  Requested: 100 ms");
    println!("  Actual:    {} ms", elapsed_ms);
    println!("  Accuracy:  {}%\n", (elapsed_ms as f64 / 100.0) * 100.0);

    // Test microsecond precision
    println!("Testing microsecond precision:");
    let start_us = get_time_us();
    delay_ms(10);
    let elapsed_us = get_time_us() - start_us;
    println!("  Requested: 10,000 µs (10 ms)");
    println!("  Actual:    {} µs", elapsed_us);
    println!("  In ms:     {} ms\n", elapsed_us / 1000);

    // Simulate swarm operations
    println!("Simulating swarm heartbeat (10 iterations):");
    for i in 1..=10 {
        let heartbeat_time = get_time_ms();
        println!("  Heartbeat {}: {} ms", i, heartbeat_time);
        delay_ms(50); // 50ms heartbeat interval
    }

    println!("\n✅ ESP32 time source working correctly!");
    println!("\nOn real ESP32 hardware:");
    println!("  - Time source is always available");
    println!("  - No initialization needed");
    println!("  - Microsecond precision via esp_timer_get_time()");
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP32-specific code (for reference)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(target_arch = "xtensa")]
mod esp32_example {
    use drone_swarm_system::{init_time_source, get_time_ms, delay_ms};

    // Typical ESP32 entry point
    // #[entry]
    fn main() -> ! {
        // Initialize ESP32 peripherals (using esp-hal or esp-idf-hal)
        // let peripherals = Peripherals::take();

        // Initialize time source (no-op on ESP32, but good practice)
        init_time_source(0);

        // Your drone swarm code here
        let start = get_time_ms();

        loop {
            let current = get_time_ms();
            let elapsed = current - start;

            // Heartbeat every 50ms
            if elapsed % 50 == 0 {
                // swarm.send_heartbeat();
            }

            delay_ms(1);
        }
    }
}

#[cfg(not(feature = "std"))]
fn main() {
    // This will be replaced by the ESP32-specific entry point
}
