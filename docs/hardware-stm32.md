# STM32 Bare-Metal Deployment Guide

Complete guide to deploying the drone swarm system on STM32 microcontrollers.

## Supported Hardware

### Recommended Development Boards

| Board | MCU | Flash | RAM | Price | Best For |
|-------|-----|-------|-----|-------|----------|
| **STM32F746G-DISCO** | STM32F746 | 1 MB | 320 KB | $50 | Development & prototyping |
| **STM32F767ZI Nucleo** | STM32F767 | 2 MB | 512 KB | $24 | Best value, lots of RAM |
| **STM32H743ZI Nucleo** | STM32H743 | 2 MB | 1 MB | $26 | High performance |
| **STM32F407VG** | STM32F407 | 1 MB | 192 KB | $15 | Budget option |

### Minimum Requirements

- **CPU**: ARM Cortex-M4 @ 80 MHz (M7 @ 200 MHz+ recommended)
- **Flash**: 256 KB minimum, 512 KB recommended
- **RAM**: 64 KB minimum, 128 KB recommended
- **Peripherals**: SPI/UART for radio, I2C for sensors, USB for debugging

---

## Development Environment Setup

### 1. Install Rust Toolchain

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM Cortex-M target
rustup target add thumbv7em-none-eabihf  # For STM32F4/F7 (Cortex-M4/M7 with FPU)
rustup target add thumbv7m-none-eabi     # For STM32F1/F3 (Cortex-M3/M4 without FPU)

# Install cargo-binutils for flashing
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

### 2. Install Debugger and Flasher

**Option A: probe-rs (Recommended)**
```bash
# Install probe-rs
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh

# Verify installation
probe-rs list
```

**Option B: OpenOCD + GDB**
```bash
# Ubuntu/Debian
sudo apt install openocd gdb-multiarch

# macOS
brew install openocd arm-none-eabi-gdb

# Windows (use MSYS2)
pacman -S mingw-w64-x86_64-arm-none-eabi-gdb mingw-w64-x86_64-openocd
```

### 3. ST-Link Driver (Windows only)

Download and install from [STMicroelectronics](https://www.st.com/en/development-tools/stsw-link009.html)

---

## Project Setup

### Create Embedded Project

```bash
# Create new project
cargo new --bin stm32_drone_swarm
cd stm32_drone_swarm

# Add dependencies
cat >> Cargo.toml << 'EOF'

[dependencies]
drone-swarm-system = { version = "0.1", default-features = false, features = ["no_std"] }
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"
embedded-hal = "1.0"
stm32f7xx-hal = { version = "0.7", features = ["stm32f746"] }

[profile.release]
opt-level = "z"       # Optimize for size
lto = true            # Link-time optimization
codegen-units = 1     # Better optimization
debug = true          # Keep debug symbols for debugging
EOF
```

### Memory Layout Configuration

Create `.cargo/config.toml`:
```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F746ZGTx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
]
```

Create `memory.x` (adjust for your specific MCU):
```
MEMORY
{
  /* STM32F746ZG: 1MB Flash, 320KB RAM */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 320K
}
```

---

## Basic Swarm Controller Implementation

### Minimal Example (`src/main.rs`)

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f7xx_hal::{pac, prelude::*};

use drone_swarm_system::{
    DroneId, Position, SwarmController, Formation,
    init_time_source,
};

#[entry]
fn main() -> ! {
    // Initialize peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure clocks (216 MHz for STM32F7)
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .sysclk(216.MHz())
        .hclk(216.MHz())
        .freeze();

    // Initialize time source for drone swarm system
    init_time_source(clocks.sysclk().raw());

    // Configure GPIO for LED (status indicator)
    let gpioi = dp.GPIOI.split();
    let mut led = gpioi.pi1.into_push_pull_output();

    // Initialize swarm controller
    let drone_id = DroneId::new(1);
    let position = Position { x: 0.0, y: 0.0, z: 10.0 };
    let mut swarm = SwarmController::new(drone_id, position);

    // Set formation
    swarm.set_formation(Formation::Circle { radius: 50.0 });

    // Configure SysTick for 20 Hz control loop (50ms)
    let mut delay = cp.SYST.delay(&clocks);

    // Main control loop
    loop {
        // Compute control velocity
        let dt = 0.05; // 50ms = 0.05s
        let velocity = swarm.compute_control_velocity(dt);

        // TODO: Send velocity command to motor controller
        // send_to_motors(velocity);

        // Blink LED to show activity
        led.toggle();

        // 20 Hz control loop
        delay.delay_ms(50u32);
    }
}
```

### Build and Flash

```bash
# Build
cargo build --release

# Flash to board
cargo run --release

# Or manually with probe-rs
probe-rs run --chip STM32F746ZGTx target/thumbv7em-none-eabihf/release/stm32_drone_swarm
```

---

## Adding Network Communication

### WiFi Module Integration (ESP8266/ESP32 via UART)

```rust
use stm32f7xx_hal::serial::{Config, Serial};

// Initialize UART for ESP module
let gpioa = dp.GPIOA.split();
let tx = gpioa.pa9.into_alternate();
let rx = gpioa.pa10.into_alternate();

let mut serial = Serial::new(
    dp.USART1,
    (tx, rx),
    &clocks,
    Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none()
);

// AT command example
writeln!(serial, "AT+CWMODE=2\r\n").unwrap(); // AP mode
```

### LoRa Module Integration (RFM95W via SPI)

```rust
use stm32f7xx_hal::spi::{Spi, Mode, Phase, Polarity};

// Configure SPI for LoRa
let gpiob = dp.GPIOB.split();
let sck = gpiob.pb3.into_alternate();
let miso = gpiob.pb4.into_alternate();
let mosi = gpiob.pb5.into_alternate();
let cs = gpiob.pb6.into_push_pull_output();

let spi_mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

let mut spi = Spi::new(
    dp.SPI1,
    (sck, miso, mosi),
    spi_mode,
    1.MHz(),
    &clocks,
);

// Initialize LoRa (using rust-lora crate)
// let mut lora = sx127x_lora::LoRa::new(spi, cs, ...);
```

---

## Adding Sensors

### IMU Integration (MPU6050 via I2C)

```rust
use stm32f7xx_hal::i2c::I2c;
use mpu6050::Mpu6050;

// Configure I2C
let gpiob = dp.GPIOB.split();
let scl = gpiob.pb8.into_alternate_open_drain();
let sda = gpiob.pb9.into_alternate_open_drain();

let i2c = I2c::new(
    dp.I2C1,
    (scl, sda),
    400.kHz(),
    &clocks,
);

// Initialize IMU
let mut mpu = Mpu6050::new(i2c);
mpu.init(&mut delay).unwrap();

// Read accelerometer and gyroscope
loop {
    let acc = mpu.get_acc().unwrap();
    let gyro = mpu.get_gyro().unwrap();

    // Update position estimate
    // position.x += acc.x * dt * dt;

    delay.delay_ms(10u32);
}
```

### GPS Module (NMEA via UART)

```rust
use nmea::Nmea;

let mut nmea = Nmea::default();
let mut gps_buffer = [0u8; 128];

loop {
    if let Ok(byte) = serial.read() {
        if let Ok(sentence) = nmea.parse_for_fix(&[byte]) {
            if let Some(fix) = sentence {
                let lat = fix.latitude;
                let lon = fix.longitude;
                let alt = fix.altitude;

                // Update drone position
                position = gps_to_local(lat, lon, alt);
            }
        }
    }
}
```

---

## Memory Optimization

### Heap Allocation Configuration

Create `heap.rs`:
```rust
use core::alloc::Layout;
use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

const HEAP_SIZE: usize = 64 * 1024; // 64 KB heap
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

pub fn init_heap() {
    unsafe {
        ALLOCATOR.lock().init(HEAP.as_mut_ptr(), HEAP_SIZE);
    }
}

#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!("Allocation error: {:?}", layout);
}
```

Call `init_heap()` early in `main()`:
```rust
#[entry]
fn main() -> ! {
    heap::init_heap();
    // ... rest of initialization
}
```

### Reduce Binary Size

```toml
[profile.release]
opt-level = "z"           # Optimize for size
lto = true                # Link-time optimization
codegen-units = 1         # Single codegen unit
strip = true              # Strip symbols
panic = "abort"           # No unwinding

[profile.release.package."*"]
opt-level = "z"
```

**Size comparison**:
- Debug build: ~850 KB
- Release (opt-level=3): ~320 KB
- Release (opt-level=z + LTO): ~180 KB

---

## Debugging

### Using probe-rs

```bash
# Run with RTT (Real-Time Transfer) logging
probe-rs run --chip STM32F746ZGTx target/thumbv7em-none-eabihf/release/stm32_drone_swarm

# Attach debugger
probe-rs attach --chip STM32F746ZGTx

# Reset chip
probe-rs reset --chip STM32F746ZGTx
```

### Enable RTT Logging

Add to `Cargo.toml`:
```toml
[dependencies]
rtt-target = "0.5"
panic-rtt-target = { version = "0.1", features = ["cortex-m"] }
```

Replace `panic-halt` with RTT logging:
```rust
use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Drone swarm controller starting...");

    // ... rest of code

    loop {
        rprintln!("Control velocity: ({:.2}, {:.2}, {:.2})",
                  velocity.vx, velocity.vy, velocity.vz);
        delay.delay_ms(50u32);
    }
}
```

### GDB Debugging

```bash
# Terminal 1: Start OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg

# Terminal 2: Start GDB
arm-none-eabi-gdb target/thumbv7em-none-eabihf/release/stm32_drone_swarm
(gdb) target remote :3333
(gdb) load
(gdb) break main
(gdb) continue
```

---

## Performance Benchmarks

### STM32F746 (216 MHz)

| Operation | Time | Notes |
|-----------|------|-------|
| SwarmController update | 2.1 ms | Formation control |
| PSO iteration (30 particles) | 45 ms | Path planning |
| ChaCha20 encrypt (1KB) | 2.3 ms | With hardware RNG |
| Ed25519 sign | 1.8 ms | Digital signature |
| Network message send | 5.2 ms | Via UART to ESP32 |

### Memory Usage

| Component | Flash | RAM |
|-----------|-------|-----|
| Core system | 120 KB | 28 KB |
| Swarm controller | 24 KB | 12 KB |
| Cryptography | 18 KB | 8 KB |
| Network stack | 32 KB | 24 KB |
| **Total** | **~180 KB** | **~64 KB** |

Leaves plenty of room for application code on most STM32 devices.

---

## Power Management

### Low-Power Mode

```rust
use stm32f7xx_hal::pwr::{Pwr, PowerMode};

let mut pwr = Pwr::new(dp.PWR);

// Enter sleep mode when idle
if !swarm.has_active_mission() {
    pwr.enter_stop_mode(&mut cortex_m::asm::wfi);
}

// Wake on interrupt (UART, SPI, timer, etc.)
```

### Battery Monitoring

```rust
use stm32f7xx_hal::adc::Adc;

let mut adc = Adc::new(dp.ADC1, &clocks);
let battery_pin = gpioa.pa0.into_analog();

loop {
    let battery_voltage: u16 = adc.read(&mut battery_pin).unwrap();
    let voltage_mv = (battery_voltage as f32 * 3300.0 / 4096.0);

    if voltage_mv < 3300.0 {
        // Low battery - initiate landing
        swarm.emergency_land();
    }

    delay.delay_ms(1000u32);
}
```

---

## Production Checklist

- [ ] **Watchdog Timer**: Enable IWDG to recover from hangs
- [ ] **Brown-Out Detection**: Configure BOR for voltage monitoring
- [ ] **Flash Protection**: Enable read/write protection for firmware
- [ ] **Secure Boot**: Implement verified boot with signatures
- [ ] **Error Handling**: Replace `panic-halt` with logging + recovery
- [ ] **OTA Updates**: Implement bootloader for field updates
- [ ] **Testing**: Hardware-in-the-loop testing with real sensors
- [ ] **Calibration**: IMU, magnetometer, and ESC calibration
- [ ] **Failsafe**: Geofencing, return-to-home, emergency landing

---

## Example Projects

### Complete STM32F7 Drone Controller

See [`examples/stm32_deployment.rs`](https://github.com/mahii6991/drone-swarm-system/blob/main/examples/stm32_deployment.rs) for:
- Full initialization sequence
- Sensor fusion (IMU + GPS)
- Motor control (PWM output)
- Network communication (ESP32 WiFi)
- Formation flight logic

### Bootloader for OTA Updates

See [`examples/stm32_bootloader.rs`](https://github.com/mahii6991/drone-swarm-system/blob/main/examples/stm32_bootloader.rs)

---

## Troubleshooting

### "Error: Mass erase failed"

**Solution**: Disable flash protection
```bash
st-flash --reset --connect-under-reset erase
```

### Stack overflow / heap exhaustion

**Solution**: Increase stack size in `memory.x` or reduce heap allocations

### Hard fault on startup

**Solution**: Enable verbose panic messages with `panic-rtt-target`

---

## Next Steps

- [ESP32 WiFi Mesh Guide](hardware-esp32.md) - Cost-effective communication
- [PX4 Integration](hardware-px4.md) - Flight controller integration

---

**Hardware support questions?** [Ask on GitHub Discussions](https://github.com/mahii6991/drone-swarm-system/discussions)
