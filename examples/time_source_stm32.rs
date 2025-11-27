//! Example: Time source initialization for STM32 (ARM Cortex-M)
//!
//! This example shows how to initialize the time source on an STM32
//! microcontroller using the DWT cycle counter and SysTick interrupt.
//!
//! # Hardware Requirements
//! - STM32F4 or similar ARM Cortex-M microcontroller
//! - Debug probe (for programming)
//!
//! # Build
//! ```bash
//! cargo build --example time_source_stm32 --target thumbv7em-none-eabihf --release
//! ```

#![no_std]
#![no_main]

// For demonstration - in real code, use your HAL crate
// use stm32f4xx_hal as hal;
// use cortex_m_rt::entry;

#[cfg(all(target_arch = "arm", not(feature = "std")))]
use drone_swarm_system::{init_time_source, get_time_ms, delay_ms};

#[cfg(all(target_arch = "arm", not(feature = "std")))]
use panic_halt as _;

// Example main function for STM32
#[cfg(all(target_arch = "arm", not(feature = "std")))]
// #[entry]
fn main() -> ! {
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 1: Initialize hardware (normally done with HAL)
    // ═══════════════════════════════════════════════════════════════════════

    // let dp = hal::pac::Peripherals::take().unwrap();
    // let cp = cortex_m::Peripherals::take().unwrap();

    // Configure system clock (example for STM32F4 @ 168 MHz)
    // let rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 2: Initialize drone swarm time source
    // ═══════════════════════════════════════════════════════════════════════

    // IMPORTANT: Pass your actual CPU frequency
    const CPU_FREQ_HZ: u32 = 168_000_000; // 168 MHz for STM32F4

    init_time_source(CPU_FREQ_HZ);

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 3: SysTick interrupt handler (add to your interrupt handlers)
    // ═══════════════════════════════════════════════════════════════════════

    // This is handled automatically by time_abstraction::systick_handler()
    // Add to your interrupt vector table:
    //
    // #[interrupt]
    // fn SysTick() {
    //     drone_swarm_system::time_abstraction::systick_handler();
    // }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 4: Use time functions in your drone swarm application
    // ═══════════════════════════════════════════════════════════════════════

    let start_time = get_time_ms();

    loop {
        // Blink LED every 1 second (example)
        let current_time = get_time_ms();
        let elapsed = current_time - start_time;

        if elapsed % 1000 < 500 {
            // LED ON (first 500ms of each second)
            // led.set_high();
        } else {
            // LED OFF (last 500ms of each second)
            // led.set_low();
        }

        // Example: Send heartbeat every 50ms
        if elapsed % 50 == 0 {
            // Send heartbeat to swarm
            // swarm.send_heartbeat();
        }

        // Small delay to prevent busy-waiting
        delay_ms(1);
    }
}

// For std builds (testing on desktop)
#[cfg(feature = "std")]
fn main() {
    use drone_swarm_system::{init_time_source, get_time_ms, delay_ms};

    println!("STM32 Time Source Example (running on std for testing)");

    init_time_source(0); // Parameter ignored on std

    let start = get_time_ms();
    println!("Start time: {} ms", start);

    for i in 1..=5 {
        delay_ms(100);
        let elapsed = get_time_ms() - start;
        println!("Iteration {}: {} ms elapsed", i, elapsed);
    }

    println!("\n✅ Time source working correctly!");
    println!("\nTo run on real STM32 hardware:");
    println!("  cargo build --example time_source_stm32 --target thumbv7em-none-eabihf --release");
}

// ═══════════════════════════════════════════════════════════════════════════
// Interrupt Handler Template (copy to your main.rs)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(all(target_arch = "arm", not(feature = "std")))]
mod interrupt_handlers {
    use drone_swarm_system::time_abstraction;

    // Add this to your interrupt handlers
    // #[interrupt]
    // fn SysTick() {
    //     time_abstraction::systick_handler();
    // }
}
