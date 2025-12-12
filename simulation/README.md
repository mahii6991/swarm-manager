# PX4 SITL Simulation Setup

This directory contains tools for testing the drone swarm system using PX4 Software-In-The-Loop (SITL) simulation.

## Quick Start

### 1. Single Drone Simulation

```bash
# Start PX4 SITL (in one terminal)
./simulation/start_sitl.sh single

# Connect with Rust code (in another terminal)
cargo run --example sitl_connection --features simulation
```

### 2. Swarm Simulation (3+ drones)

```bash
# Start swarm SITL (requires Gazebo)
./simulation/start_sitl.sh swarm 3

# Connect with Rust swarm code
cargo run --example swarm_sitl --features "simulation tokio" -- 3
```

### 3. Headless Mode (CI/Testing)

```bash
# No GUI, just MAVLink
./simulation/start_sitl.sh headless
```

## Prerequisites

### Already Installed
- [x] PX4-Autopilot: `~/PX4-Autopilot`
- [x] QGroundControl: `/Applications/QGroundControl.app`
- [ ] Gazebo Harmonic: `brew install gz-harmonic` (in progress)

### MAVLink Ports

| Mode | Drone | MAVLink Port | Simulator Port |
|------|-------|--------------|----------------|
| Single | 0 | UDP 14540 | 4560 |
| Swarm | 0 | UDP 14540 | 4560 |
| Swarm | 1 | UDP 14541 | 4561 |
| Swarm | 2 | UDP 14542 | 4562 |
| Swarm | N | UDP 14540+N | 4560+N |

## Integration with Rust Code

### Enable simulation feature in Cargo.toml

```toml
[features]
simulation = ["std", "mavlink"]
```

### Basic Connection

```rust
use mavlink::common::MavMessage;

let address = "udpin:0.0.0.0:14540";
let connection = mavlink::connect::<MavMessage>(address)?;

// Receive messages
loop {
    let (_header, msg) = connection.recv()?;
    match msg {
        MavMessage::HEARTBEAT(hb) => {
            println!("Heartbeat: {:?}", hb);
        }
        MavMessage::LOCAL_POSITION_NED(pos) => {
            println!("Position: x={}, y={}, z={}", pos.x, pos.y, pos.z);
        }
        _ => {}
    }
}
```

### Sending Commands

```rust
use mavlink::common::{MavMessage, COMMAND_LONG_DATA};

// Arm the drone
let arm_cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
    target_system: 1,
    target_component: 1,
    command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation: 0,
    param1: 1.0,  // 1 = arm, 0 = disarm
    param2: 0.0,
    param3: 0.0,
    param4: 0.0,
    param5: 0.0,
    param6: 0.0,
    param7: 0.0,
});

connection.send(&mavlink::MavHeader::default(), &arm_cmd)?;
```

## Simulation Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Your Development Machine                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     UDP 14540     ┌──────────────────────┐   │
│  │  PX4 SITL    │ ◄───────────────► │  Your Rust Code      │   │
│  │  (Drone 0)   │     MAVLink       │  (sitl_connection)   │   │
│  └──────────────┘                   └──────────────────────┘   │
│         │                                     │                  │
│         │                                     │                  │
│  ┌──────────────┐                   ┌──────────────────────┐   │
│  │   Gazebo     │                   │   QGroundControl     │   │
│  │  (3D View)   │                   │   (Ground Station)   │   │
│  └──────────────┘                   └──────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Common Commands

### PX4 Console Commands (in SITL terminal)

```bash
# Check vehicle status
commander status

# Arm vehicle
commander arm

# Takeoff to 10m
commander takeoff

# Land
commander land

# Disarm
commander disarm
```

### QGroundControl

1. Open QGroundControl app
2. It auto-connects to SITL on UDP 14550
3. Use the UI to:
   - Arm/disarm
   - Set flight modes
   - Create waypoint missions
   - Monitor telemetry

## Troubleshooting

### "Connection refused" error

1. Make sure SITL is running first
2. Check the correct port (14540 for first drone)
3. Wait for "Ready for takeoff" message in SITL

### Gazebo not starting

1. Check if Gazebo is installed: `gz sim --version`
2. If not installed: `brew install gz-harmonic`
3. Use headless mode: `./start_sitl.sh headless`

### Multiple instances conflict

1. Kill existing instances: `pkill -f px4`
2. Restart simulation

## Next Steps

1. **Test basic connection**: `cargo run --example sitl_connection --features simulation`
2. **Implement swarm algorithms**: Connect your PSO/ACO/GWO to SITL drones
3. **Test formation control**: Use swarm_sitl example as starting point
4. **Add MAVLink commands**: Implement takeoff, waypoint navigation, landing
