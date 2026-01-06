# PX4 and ArduPilot Integration Guide

Integrate the drone swarm system with popular flight controllers using MAVLink protocol.

## Overview

This guide shows how to use **PX4** or **ArduPilot** as the low-level flight controller while running swarm coordination on a companion computer (Raspberry Pi, Jetson Nano, etc.).

**Architecture**:
```
┌─────────────────────────────────────┐
│   Companion Computer (Linux/Rust)   │
│  ┌───────────────────────────────┐  │
│  │  Drone Swarm System Library   │  │
│  │  - Formation control          │  │
│  │  - Path planning (PSO/ACO)    │  │
│  │  - Mesh networking            │  │
│  │  - Federated learning         │  │
│  └──────────┬────────────────────┘  │
│             │ MAVLink (UART/UDP)    │
│  ┌──────────▼────────────────────┐  │
│  │     MAVLink Interface         │  │
│  └──────────┬────────────────────┘  │
└─────────────┼─────────────────────┘
              │ Serial/WiFi
┌─────────────▼─────────────────────┐
│   Flight Controller (PX4/ArduPilot)│
│   - Stabilization                  │
│   - Motor control                  │
│   - Sensor fusion (IMU, GPS)       │
│   - Failsafe                       │
└────────────────────────────────────┘
```

---

## Supported Hardware

### Flight Controllers

| Controller | Firmware | CPU | Price | Notes |
|------------|----------|-----|-------|-------|
| **Pixhawk 4** | PX4/ArduPilot | STM32F7 | $200 | Industry standard |
| **Pixhawk 6C** | PX4 | STM32H7 | $240 | Latest, most powerful |
| **Holybro Kakute H7** | ArduPilot | STM32H7 | $80 | Racing quad |
| **mRo Pixracer** | PX4/ArduPilot | STM32F4 | $120 | Lightweight |
| **CUAV X7+** | PX4 | STM32H7 | $280 | Triple redundancy |

All support MAVLink 2.0 over UART/USB.

### Companion Computers

| Computer | RAM | CPU | WiFi | Price | Best For |
|----------|-----|-----|------|-------|----------|
| **Raspberry Pi 4B** | 4 GB | Cortex-A72 | Yes | $55 | General purpose |
| **Jetson Nano** | 4 GB | Cortex-A57 + GPU | No | $99 | Computer vision |
| **Raspberry Pi Zero 2 W** | 512 MB | Cortex-A53 | Yes | $15 | Ultra-lightweight |
| **NVIDIA Jetson Xavier NX** | 8 GB | Carmel + GPU | No | $399 | High-performance AI |
| **Orange Pi 5** | 8 GB | RK3588S | Yes | $80 | Best value |

---

## MAVLink Protocol Basics

### What is MAVLink?

**MAVLink** (Micro Air Vehicle Link) is a lightweight messaging protocol for drones:
- Binary protocol (efficient for low-bandwidth links)
- Message-based (position, velocity, mission commands, etc.)
- Widely supported (PX4, ArduPilot, QGroundControl, Mission Planner)

### Key Message Types

| Message | ID | Purpose |
|---------|----|---------|
| `HEARTBEAT` | 0 | Presence/status announcement |
| `ATTITUDE` | 30 | Roll, pitch, yaw |
| `LOCAL_POSITION_NED` | 32 | Position in local frame (North-East-Down) |
| `GLOBAL_POSITION_INT` | 33 | GPS position (lat, lon, alt) |
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | Set velocity/position setpoint |
| `COMMAND_LONG` | 76 | Arm, takeoff, land, etc. |
| `MISSION_ITEM` | 39 | Waypoint upload |

---

## Setup: Companion Computer Connection

### Physical Connection

**Option 1: UART (Serial)**
```
Pixhawk TELEM2 (6-pin JST-GH):
Pin 1: +5V  ────────────┐
Pin 2: TX   ────────┐   │
Pin 3: RX   ──────┐ │   │
Pin 4: CTS  ─┐    │ │   │
Pin 5: RTS  ─┼─┐  │ │   │
Pin 6: GND  ─┼─┼──┼─┼───┴── Raspberry Pi
             │ │  │ │
             │ │  │ └────── GPIO 15 (RX)
             │ │  └──────── GPIO 14 (TX)
             │ └─────────── Optional (flow control)
             └──────────── Optional (flow control)
```

**Option 2: USB** (easier for development)
```
Pixhawk USB ──[USB cable]── Raspberry Pi USB port
```

Appears as `/dev/ttyACM0` or `/dev/ttyUSB0`.

**Option 3: WiFi/UDP** (wireless, for testing)
```
Pixhawk ─[Serial]─ ESP8266 ─[WiFi]─ Raspberry Pi
```

### Enable MAVLink on PX4

Connect via QGroundControl and set parameters:
```
MAV_1_CONFIG = TELEM 2
MAV_1_MODE = Normal
MAV_1_RATE = 921600 (baud rate)
MAV_1_FORWARD = Enabled
SER_TEL2_BAUD = 921600
```

Reboot flight controller.

### Test Connection

```bash
# Install MAVProxy for testing
pip3 install MAVProxy

# Connect to flight controller
mavproxy.py --master=/dev/ttyACM0 --baudrate=921600

# You should see heartbeat messages:
# HEARTBEAT {type : 2, autopilot : MAV_AUTOPILOT_PX4, ...}
```

---

## Rust MAVLink Integration

### Add Dependencies

```toml
[dependencies]
swarm-manager = "0.1"
mavlink = { version = "0.13", features = ["default", "common", "uavionix"] }
serialport = "4.0"
tokio = { version = "1", features = ["full"] }
anyhow = "1.0"
log = "0.4"
```

### Connect to Flight Controller

```rust
use mavlink::{self, MavConnection};
use std::sync::{Arc, Mutex};
use anyhow::Result;

fn main() -> Result<()> {
    // Connect to Pixhawk via serial
    let mut vehicle = mavlink::connect::<mavlink::common::MavMessage>(
        "serial:/dev/ttyACM0:921600"
    )?;

    println!("Connected to flight controller!");

    // Request data streams
    request_data_stream(&mut vehicle)?;

    // Receive loop
    loop {
        match vehicle.recv() {
            Ok((_header, msg)) => {
                match msg {
                    mavlink::common::MavMessage::HEARTBEAT(hb) => {
                        println!("Heartbeat: {:?}", hb);
                    }
                    mavlink::common::MavMessage::GLOBAL_POSITION_INT(pos) => {
                        println!("Position: lat={}, lon={}, alt={}",
                                 pos.lat as f64 / 1e7,
                                 pos.lon as f64 / 1e7,
                                 pos.alt);
                    }
                    _ => {}
                }
            }
            Err(e) => eprintln!("Receive error: {:?}", e),
        }
    }
}

fn request_data_stream(vehicle: &mut Box<dyn MavConnection + Send>) -> Result<()> {
    use mavlink::common::*;

    let msg = MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
        target_system: 1,
        target_component: 1,
        req_stream_id: 0, // All streams
        req_message_rate: 10, // 10 Hz
        start_stop: 1, // Start
    });

    vehicle.send_default(&msg)?;
    Ok(())
}
```

---

## Swarm Integration

### Bridge MAVLink Position to Swarm System

```rust
use drone_swarm_system::{DroneId, Position, SwarmController, Formation};
use mavlink::common::*;
use std::sync::{Arc, Mutex};

struct DroneSwarmBridge {
    vehicle: Box<dyn MavConnection + Send>,
    swarm: SwarmController,
    current_position: Arc<Mutex<Position>>,
}

impl DroneSwarmBridge {
    fn new(vehicle_url: &str, drone_id: DroneId) -> Result<Self> {
        let vehicle = mavlink::connect::<MavMessage>(vehicle_url)?;

        let position = Position { x: 0.0, y: 0.0, z: 0.0 };
        let swarm = SwarmController::new(drone_id, position);

        Ok(Self {
            vehicle,
            swarm,
            current_position: Arc::new(Mutex::new(position)),
        })
    }

    fn run(&mut self) -> Result<()> {
        // Request position updates at 20 Hz
        self.request_data_stream()?;

        // Spawn thread for swarm control loop
        let pos_clone = self.current_position.clone();
        let mut swarm_clone = self.swarm.clone();

        std::thread::spawn(move || {
            swarm_control_loop(&mut swarm_clone, pos_clone)
        });

        // Main MAVLink receive loop
        loop {
            match self.vehicle.recv() {
                Ok((_header, msg)) => self.handle_mavlink_message(msg)?,
                Err(e) => eprintln!("MAVLink error: {:?}", e),
            }
        }
    }

    fn handle_mavlink_message(&mut self, msg: MavMessage) -> Result<()> {
        match msg {
            MavMessage::LOCAL_POSITION_NED(pos) => {
                // Convert NED (North-East-Down) to ENU (East-North-Up) used by swarm system
                let position = Position {
                    x: pos.y,  // East
                    y: pos.x,  // North
                    z: -pos.z, // Up (negate Down)
                };

                *self.current_position.lock().unwrap() = position;
                self.swarm.update_position(position);
            }
            MavMessage::HEARTBEAT(_) => {
                // System is alive
            }
            _ => {}
        }
        Ok(())
    }

    fn request_data_stream(&mut self) -> Result<()> {
        let msg = MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
            target_system: 1,
            target_component: 1,
            req_stream_id: 6, // Position stream
            req_message_rate: 20, // 20 Hz
            start_stop: 1,
        });
        self.vehicle.send_default(&msg)?;
        Ok(())
    }
}

fn swarm_control_loop(
    swarm: &mut SwarmController,
    position: Arc<Mutex<Position>>,
) -> Result<()> {
    swarm.set_formation(Formation::Circle { radius: 50.0 });

    loop {
        // Update swarm with current position
        let pos = *position.lock().unwrap();
        swarm.update_position(pos);

        // Compute desired velocity
        let dt = 0.05; // 50ms = 20 Hz
        let velocity = swarm.compute_control_velocity(dt);

        // Send velocity command to Pixhawk
        send_velocity_setpoint(velocity)?;

        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}
```

### Send Velocity Commands to PX4

```rust
use mavlink::common::*;

fn send_velocity_setpoint(
    vehicle: &mut Box<dyn MavConnection + Send>,
    velocity: Velocity,
) -> Result<()> {
    let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(
        SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,
            target_system: 1,
            target_component: 1,
            coordinate_frame: MAV_FRAME_LOCAL_NED,
            type_mask: 0b0000_1111_1100_0111, // Ignore position, use velocity
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: velocity.vy,  // North (NED)
            vy: velocity.vx,  // East (NED)
            vz: -velocity.vz, // Down (NED)
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        }
    );

    vehicle.send_default(&msg)?;
    Ok(())
}
```

---

## High-Level Commands

### Arm and Takeoff

```rust
fn arm_and_takeoff(vehicle: &mut Box<dyn MavConnection + Send>, altitude: f32) -> Result<()> {
    use mavlink::common::*;

    // 1. Set mode to GUIDED (PX4) or GUIDED (ArduPilot)
    set_mode(vehicle, "OFFBOARD")?; // PX4
    // OR
    // set_mode(vehicle, "GUIDED")?; // ArduPilot

    std::thread::sleep(std::time::Duration::from_secs(1));

    // 2. Arm motors
    let arm_cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 1.0, // Arm
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    vehicle.send_default(&arm_cmd)?;

    println!("Armed!");
    std::thread::sleep(std::time::Duration::from_secs(2));

    // 3. Takeoff
    let takeoff_cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MAV_CMD_NAV_TAKEOFF,
        confirmation: 0,
        param1: 0.0, // Pitch
        param2: 0.0,
        param3: 0.0,
        param4: 0.0, // Yaw
        param5: 0.0, // Latitude (0 = current)
        param6: 0.0, // Longitude (0 = current)
        param7: altitude, // Altitude
    });
    vehicle.send_default(&takeoff_cmd)?;

    println!("Taking off to {} meters...", altitude);

    Ok(())
}

fn set_mode(vehicle: &mut Box<dyn MavConnection + Send>, mode: &str) -> Result<()> {
    use mavlink::common::*;

    let mode_id = match mode {
        "OFFBOARD" => 6,   // PX4 offboard mode
        "GUIDED" => 4,     // ArduPilot guided mode
        "AUTO" => 3,
        "STABILIZE" => 0,
        _ => return Err(anyhow::anyhow!("Unknown mode")),
    };

    let msg = MavMessage::SET_MODE(SET_MODE_DATA {
        target_system: 1,
        base_mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode: mode_id,
    });

    vehicle.send_default(&msg)?;
    Ok(())
}
```

### Land

```rust
fn land(vehicle: &mut Box<dyn MavConnection + Send>) -> Result<()> {
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MAV_CMD_NAV_LAND,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0, // Yaw
        param5: 0.0, // Latitude (0 = current)
        param6: 0.0, // Longitude (0 = current)
        param7: 0.0, // Altitude (0 = current)
    });

    vehicle.send_default(&msg)?;
    println!("Landing...");
    Ok(())
}
```

---

## Complete Example: Formation Flight

```rust
use drone_swarm_system::*;
use mavlink::common::*;
use anyhow::Result;

fn main() -> Result<()> {
    // Connect to Pixhawk
    let mut vehicle = mavlink::connect::<MavMessage>(
        "serial:/dev/ttyACM0:921600"
    )?;

    // Initialize swarm controller
    let drone_id = DroneId::new(1);
    let position = Position { x: 0.0, y: 0.0, z: 0.0 };
    let mut swarm = SwarmController::new(drone_id, position);

    // Set circle formation (50m radius)
    swarm.set_formation(Formation::Circle { radius: 50.0 });

    // Arm and takeoff to 10m
    arm_and_takeoff(&mut vehicle, 10.0)?;

    // Wait for takeoff to complete
    std::thread::sleep(std::time::Duration::from_secs(10));

    // Enable offboard mode (required for velocity control)
    set_mode(&mut vehicle, "OFFBOARD")?;

    // Main control loop
    let start_time = std::time::Instant::now();
    loop {
        // Receive position from Pixhawk
        if let Ok((_header, msg)) = vehicle.recv() {
            if let MavMessage::LOCAL_POSITION_NED(pos) = msg {
                // Update swarm position
                let position = Position {
                    x: pos.y,
                    y: pos.x,
                    z: -pos.z,
                };
                swarm.update_position(position);

                // Compute control velocity
                let velocity = swarm.compute_control_velocity(0.05);

                // Send to Pixhawk
                send_velocity_setpoint(&mut vehicle, velocity)?;

                println!("Velocity: ({:.2}, {:.2}, {:.2})",
                         velocity.vx, velocity.vy, velocity.vz);
            }
        }

        // Mission duration: 60 seconds
        if start_time.elapsed().as_secs() > 60 {
            break;
        }
    }

    // Land
    land(&mut vehicle)?;

    // Wait for landing
    std::thread::sleep(std::time::Duration::from_secs(10));

    // Disarm
    disarm(&mut vehicle)?;

    Ok(())
}
```

---

## PX4 vs ArduPilot Differences

| Feature | PX4 | ArduPilot |
|---------|-----|-----------|
| Offboard mode | OFFBOARD (ID 6) | GUIDED (ID 4) |
| Velocity control | SET_POSITION_TARGET_LOCAL_NED | Same |
| Coordinate frame | MAV_FRAME_LOCAL_NED | Same |
| Custom mode IDs | Different | Different |
| Configuration | Parameters (QGC) | Parameters (Mission Planner) |

**Tip**: Check `custom_mode` in HEARTBEAT message to detect firmware type.

---

## Simulation with SITL

### PX4 SITL (Software In The Loop)

```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Build and run SITL
make px4_sitl gazebo

# In another terminal, connect your Rust code
cargo run -- --mavlink udp://127.0.0.1:14540
```

### ArduPilot SITL

```bash
# Install ArduCopter SITL
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter

# Run SITL
./build/sitl/bin/arducopter --model quad

# Connect Rust code
cargo run -- --mavlink tcp://127.0.0.1:5760
```

---

## Multi-Drone SITL Testing

### Launch 10 Drones

```bash
#!/bin/bash
# start_swarm_sitl.sh

for i in {1..10}; do
    cd ~/PX4-Autopilot
    HEADLESS=1 PX4_SIM_MODEL=iris \
    PX4_SYS_AUTOSTART=4001 \
    PX4_SIM_SPEED_FACTOR=1 \
    ./build/px4_sitl_default/bin/px4 \
    -i $i \
    -d "instance_$i" &
done

wait
```

### Connect Swarm Controller to All Drones

```rust
use std::thread;

fn main() -> Result<()> {
    let mut handles = vec![];

    for i in 1..=10 {
        let handle = thread::spawn(move || {
            let url = format!("udp://127.0.0.1:{}", 14540 + i);
            let mut bridge = DroneSwarmBridge::new(&url, DroneId::new(i))?;
            bridge.run()
        });
        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap()?;
    }

    Ok(())
}
```

---

## Performance Considerations

### Message Rate Limits

PX4 has rate limits for MAVLink messages:
- Position updates: Max 50 Hz
- Velocity setpoints: Max 20 Hz (recommended)
- Mission items: Max 10 Hz

**Recommendation**: Send velocity commands at 10-20 Hz to match control loop.

### Latency

Typical latencies:
- Serial (921600 baud): ~2-5 ms
- UDP (WiFi): ~10-30 ms
- USB: ~1-3 ms

**Compensation**: Use predictive control or increase look-ahead time.

---

## Troubleshooting

### "Offboard mode rejected"

**Cause**: PX4 requires continuous velocity setpoints before accepting OFFBOARD mode.

**Solution**: Send setpoints for 0.5s before mode switch:
```rust
for _ in 0..10 {
    send_velocity_setpoint(&mut vehicle, Velocity { vx: 0.0, vy: 0.0, vz: 0.0 })?;
    std::thread::sleep(Duration::from_millis(50));
}
set_mode(&mut vehicle, "OFFBOARD")?;
```

### "Connection timeout"

**Check**:
- Correct serial port (`ls /dev/tty*`)
- Baud rate matches (921600 vs 57600)
- USB cable quality (try different cable)
- Permissions: `sudo usermod -a -G dialout $USER`

### "Invalid frame"

**Cause**: MAVLink frame mismatch (v1 vs v2).

**Solution**: Force MAVLink 2:
```toml
[dependencies]
mavlink = { version = "0.13", features = ["default", "common"], default-features = false }
```

---

## Production Deployment

### Systemd Service (Auto-start on Boot)

Create `/etc/systemd/system/drone-swarm.service`:
```ini
[Unit]
Description=Drone Swarm Controller
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/drone-swarm
ExecStart=/home/pi/drone-swarm/target/release/swarm_controller --mavlink serial:/dev/ttyACM0:921600
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable drone-swarm
sudo systemctl start drone-swarm
sudo systemctl status drone-swarm
```

### Logging

```rust
use env_logger;
use log::*;

fn main() -> Result<()> {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    info!("Drone swarm controller starting...");
    // ... rest of code
}
```

---

## Example Projects

### [`examples/px4_formation_flight.rs`](https://github.com/mahii6991/swarm-manager/tree/main/examples)
Complete formation flight with:
- MAVLink communication
- Position control
- Formation transitions
- Emergency landing

### [`examples/ardupilot_swarm.rs`](https://github.com/mahii6991/swarm-manager/tree/main/examples)
ArduPilot-specific example with mission upload.

---

## Next Steps

- [ESP32 WiFi Mesh](hardware-esp32.md) - Cost-effective communication
- [STM32 Deployment](hardware-stm32.md) - Lower power alternative

---

**Questions?** [Ask on GitHub Discussions](https://github.com/mahii6991/swarm-manager/discussions)
