<div align="center">

# Swarm Manager

### Industrial-Grade Autonomous Drone Swarm Communication & Intelligence

[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange?style=flat-square&logo=rust)](https://www.rust-lang.org/)
[![Build](https://img.shields.io/github/actions/workflow/status/mahii6991/drone-swarm-system/ci.yml?branch=main&style=flat-square)](https://github.com/mahii6991/swarm-manager/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue?style=flat-square)](LICENSE)
[![Docs](https://img.shields.io/badge/Docs-GitHub%20Pages-green?style=flat-square)](https://mahii6991.github.io/drone-swarm-system/)

---

**A safety-critical drone swarm system built in Rust with zero heap allocation, Byzantine fault tolerance, and real-time swarm intelligence algorithms.**

</div>

---

## Status

| Component | Status | Notes |
|:----------|:------:|:------|
| Swarm Algorithms (PSO, ACO, GWO, WOA) | :white_check_mark: Production-ready | Bio-inspired optimization suite |
| Cryptography (ChaCha20, Ed25519) | :white_check_mark: Production-ready | Military-grade encryption |
| SwarmRaft Consensus + Merkle Trees | :white_check_mark: Production-ready | Tamper-evident distributed consensus |
| ESP32 WiFi Mesh Networking | :white_check_mark: Production-ready | Self-healing mesh with auto-discovery |
| MAVLink Protocol | :white_check_mark: Production-ready | Flight controller communication |
| PX4 SITL Integration | :white_check_mark: Production-ready | Multi-drone simulation support |
| Collision Avoidance | :white_check_mark: Production-ready | VO, RVO, ORCA, APF algorithms |
| Mission Planning | :white_check_mark: Production-ready | Waypoints & survey patterns |
| Telemetry & Monitoring | :white_check_mark: Production-ready | Health alerts & status tracking |
| Failsafe Behaviors | :white_check_mark: Production-ready | RTL, land, geofence protection |
| Interactive Visualization | :white_check_mark: Production-ready | egui-based real-time GUI |

---

## Features

| | Feature | Description |
|:---:|:---|:---|
| :shield: | **Enterprise-Grade Security** | ChaCha20-Poly1305 encryption, Ed25519 signatures, Byzantine fault tolerance |
| :globe_with_meridians: | **Mesh Networking** | Adaptive multi-hop routing supporting 100+ drones with self-healing topology |
| :brain: | **Swarm Intelligence** | PSO, ACO, GWO, WOA bio-inspired optimization for collective decision making |
| :handshake: | **Distributed Consensus** | SwarmRaft protocol with Merkle tree verification for tamper-evident logging |
| :robot: | **Federated Learning** | Privacy-preserving decentralized model training across the swarm |
| :zap: | **Real-Time Performance** | Zero heap allocation, `no_std` compatible for embedded systems |
| :warning: | **Collision Avoidance** | VO, RVO, ORCA, APF algorithms with geofence protection |
| :world_map: | **Mission Planning** | Waypoint navigation with lawnmower, spiral, square survey patterns |
| :helicopter: | **Flight Control** | MAVLink integration with PX4/ArduPilot SITL support |
| :bar_chart: | **Telemetry & Failsafe** | Real-time health monitoring with automated RTL, land, hold behaviors |

---

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/mahii6991/swarm-manager.git
cd drone-swarm-system

# Build the project
cargo build --release

# Run tests
cargo test
```

### Run Examples

```bash
# Basic swarm simulation
cargo run --example simple_swarm

# PSO optimization demo
cargo run --example pso_optimization

# ACO path planning
cargo run --example aco_path_planning

# GWO swarm optimization
cargo run --example gwo_swarm_optimization
```

### Launch Visualization

```bash
# Start the interactive GUI
cargo run -p drone-swarm-visualization --release
```

> **Keyboard Shortcuts:** `Space` = Play/Pause | `R` = Reset | `D` = Demo Mode | `Esc` = Deselect

---

## Usage

```rust
use drone_swarm_system::{
    types::{DroneId, Position},
    control::swarm::{SwarmController, Formation},
    network::core::MeshNetwork,
};

// Initialize a drone
let drone_id = DroneId::new(1);
let position = Position { x: 0.0, y: 0.0, z: 10.0 };

// Create swarm controller
let mut swarm = SwarmController::new(drone_id, position);

// Set formation
swarm.set_formation(Formation::Circle { radius: 50 });

// Initialize mesh network
let mut network = MeshNetwork::new(drone_id);
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         DRONE SWARM SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   Security  │  │  Consensus  │  │   Network   │  │   Merkle    │    │
│  │  ChaCha20   │  │  SwarmRaft  │  │    Mesh     │  │    Tree     │    │
│  │   Ed25519   │  │   Leader    │  │   Routing   │  │  Logging    │    │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘    │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │     PSO     │  │     ACO     │  │     GWO     │  │     WOA     │    │
│  │  Particle   │  │  Ant Colony │  │  Grey Wolf  │  │    Whale    │    │
│  │   Swarm     │  │ Optimization│  │  Optimizer  │  │ Optimization│    │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘    │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      Federated Learning                          │   │
│  │              Privacy-Preserving ML Training Across Swarm         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Modules

The codebase is organized into logical module groups:

### `algorithms/` - Swarm Intelligence & Optimization
| Module | Description |
|:-------|:------------|
| `pso` | Particle Swarm Optimization (basic & advanced variants) |
| `aco` | Ant Colony Optimization for path planning |
| `gwo` | Grey Wolf Optimizer for swarm coordination |
| `woa` | Whale Optimization Algorithm |
| `hybrid` | Hybrid optimization combining multiple algorithms |
| `selector` | Deep RL-based algorithm selection |

### `consensus/` - Distributed Agreement
| Module | Description |
|:-------|:------------|
| `raft` | SwarmRaft distributed consensus protocol |
| `pbft` | Practical Byzantine Fault Tolerance |
| `hierarchical` | Hierarchical consensus for large swarms |
| `merkle` | Merkle tree for tamper-evident logging |

### `network/` - Communication
| Module | Description |
|:-------|:------------|
| `core` | Adaptive mesh routing with multi-hop support |
| `mesh` | ESP32 WiFi mesh protocol |
| `mavlink` | MAVLink flight controller interface |
| `esp32` | ESP32-specific networking |
| `routing` | Proactive routing & link prediction |

### `control/` - Flight Control & Planning
| Module | Description |
|:-------|:------------|
| `swarm` | Formation control & collective behavior |
| `collision` | VO, RVO, ORCA, APF collision avoidance |
| `mission` | Waypoint navigation & survey patterns |
| `coordinator` | Multi-drone SITL coordination |
| `task` | Task distribution & priority management |

### `safety/` - Security & Fault Tolerance
| Module | Description |
|:-------|:------------|
| `crypto` | ChaCha20-Poly1305 encryption & Ed25519 signatures |
| `security` | Intrusion detection & threat mitigation |
| `failsafe` | RTL, land, geofence protection |
| `fault_tolerance` | Self-healing & automatic failover |
| `chaos_monkey` | Fault injection for testing |

### `ml/` - Machine Learning
| Module | Description |
|:-------|:------------|
| `federated` | Privacy-preserving decentralized learning |

### `system/` - Infrastructure
| Module | Description |
|:-------|:------------|
| `config` | System configuration management |
| `telemetry` | Health monitoring, alerts & status |
| `time` | Hardware-agnostic time abstraction |
| `clustering` | Drone cluster management |

---

## Visualization

The interactive GUI provides real-time visualization of:

- **Drone Formations** - Circle, Grid, Line, V-Formation, Random
- **Algorithm Visualization** - PSO particles, ACO pheromone trails, GWO wolf hierarchy
- **Network Topology** - Link quality, routing paths, mesh connectivity
- **Metrics Dashboard** - Convergence graphs, FPS, drone statistics
- **Safety Panel** - Collision avoidance zones, geofence boundaries, failsafe status
- **Mission Planning** - Waypoints, paths, survey patterns (lawnmower, spiral, square)
- **Telemetry Monitoring** - Battery levels, health status, alerts

```bash
cargo run -p drone-swarm-visualization --release
```

---

## Supported Platforms

| Platform | Status | Notes |
|:---------|:------:|:------|
| **x86_64 Linux** | :white_check_mark: | Full support |
| **x86_64 macOS** | :white_check_mark: | Full support |
| **ARM64 (Apple Silicon)** | :white_check_mark: | Full support |
| **Windows** | :white_check_mark: | Full support |
| **STM32** | :white_check_mark: | `no_std` embedded |
| **ESP32** | :white_check_mark: | `no_std` embedded |
| **nRF52** | :white_check_mark: | `no_std` embedded |
| **RISC-V** | :white_check_mark: | `no_std` embedded |

---

## Documentation

- :book: [Getting Started](https://mahii6991.github.io/drone-swarm-system/getting-started)
- :gear: [API Reference](https://mahii6991.github.io/drone-swarm-system/api-reference)
- :building_construction: [Architecture](https://mahii6991.github.io/drone-swarm-system/architecture)
- :test_tube: [Examples](https://mahii6991.github.io/drone-swarm-system/examples)
- :lock: [Security](https://mahii6991.github.io/drone-swarm-system/security)

---

## Project Structure

```
drone-swarm-system/
├── src/
│   ├── algorithms/         # Optimization algorithms
│   │   ├── pso/            # Particle Swarm (basic + advanced)
│   │   ├── aco.rs          # Ant Colony Optimization
│   │   ├── gwo.rs          # Grey Wolf Optimizer
│   │   ├── woa.rs          # Whale Optimization
│   │   ├── hybrid.rs       # Hybrid optimizer
│   │   └── selector.rs     # Deep RL algorithm selection
│   ├── consensus/          # Distributed consensus
│   │   ├── raft.rs         # SwarmRaft protocol
│   │   ├── pbft.rs         # Byzantine fault tolerance
│   │   ├── hierarchical.rs # Hierarchical consensus
│   │   └── merkle.rs       # Merkle tree logging
│   ├── network/            # Communication layer
│   │   ├── core.rs         # Mesh networking
│   │   ├── mesh.rs         # ESP32 mesh protocol
│   │   ├── mavlink.rs      # MAVLink interface
│   │   └── routing/        # Proactive routing
│   ├── control/            # Flight control
│   │   ├── swarm.rs        # Formation control
│   │   ├── collision.rs    # Collision avoidance
│   │   ├── mission.rs      # Mission planning
│   │   └── coordinator.rs  # Multi-drone coordination
│   ├── safety/             # Security & fault tolerance
│   │   ├── crypto.rs       # Encryption & signatures
│   │   ├── security.rs     # Intrusion detection
│   │   ├── failsafe.rs     # Safety behaviors
│   │   └── fault_tolerance.rs
│   ├── ml/                 # Machine learning
│   │   └── federated.rs    # Federated learning
│   ├── system/             # Infrastructure
│   │   ├── config.rs       # Configuration
│   │   ├── telemetry.rs    # Health monitoring
│   │   └── time.rs         # Time abstraction
│   ├── lib.rs              # Library root
│   └── types.rs            # Core types
├── visualization/          # Interactive GUI
├── examples/               # Usage examples
├── tests/                  # Comprehensive tests
├── fuzz/                   # Security fuzzing
└── docs/                   # Documentation
```

---

## For Companies

<div align="center">

### Ready-to-Deploy Autonomous Drone Swarm Infrastructure

</div>

### What You Get

| Component | Description |
|:----------|:------------|
| **Rust Swarm Core** | Battle-tested, memory-safe foundation with 13,500+ lines of production code |
| **Mesh Networking** | Self-healing, multi-hop communication supporting 100+ drones |
| **Distributed Consensus** | SwarmRaft protocol with Byzantine fault tolerance |
| **Collision Avoidance** | 4 algorithms (VO, RVO, ORCA, APF) + geofencing |
| **PX4/SITL Integration** | Ready for hardware-in-the-loop testing and real deployments |
| **Interactive GUI** | Real-time visualization dashboard for monitoring and control |
| **Mission Planning** | Waypoint navigation with survey patterns (lawnmower, spiral, grid) |
| **Failsafe Systems** | RTL, emergency land, altitude hold with configurable triggers |
| **Cryptographic Security** | ChaCha20-Poly1305 + Ed25519 for secure communications |

### Who It's For

| Industry | Use Cases |
|:---------|:----------|
| **Drone Manufacturers** | Integrate swarm intelligence into your UAV product line |
| **Civilian & Enterprise** | Tactical reconnaissance, perimeter security, ISR missions |
| **Security & Surveillance** | Automated patrol, crowd monitoring, critical infrastructure |
| **Inspection & Mapping** | Solar farms, pipelines, power lines, agricultural surveys |
| **Research Labs** | Academic research, algorithm development, swarm behavior studies |
| **Search & Rescue** | Coordinated search patterns, area coverage optimization |

### Enterprise Features

- **Priority Support** - Direct engineering support with guaranteed response times
- **Custom Integration** - Tailored integration with your existing drone fleet
- **Hardware Optimization** - Embedded systems tuning for your specific platforms
- **Training & Onboarding** - Technical workshops for your engineering team
- **Source Code Access** - Full source with modification rights for enterprise
- **Compliance Assistance** - Help with aviation regulatory requirements

---

### Get Started

<div align="center">

**Ready to deploy autonomous drone swarms?**

[:email: **Contact for Commercial Licensing**](mailto:m.s.rajpoot20@gmail.com)

[:calendar: **Schedule a Demo**](mailto:m.s.rajpoot20@gmail.com?subject=Drone%20Swarm%20System%20Demo%20Request&body=Hi%2C%0A%0AI%27m%20interested%20in%20learning%20more%20about%20the%20Drone%20Swarm%20System%20for%20our%20organization.%0A%0ACompany%3A%20%0AUse%20Case%3A%20%0ATimeline%3A%20%0A%0APlease%20contact%20me%20to%20schedule%20a%20demo.%0A%0AThanks!)

</div>

---

## License

| Component | License |
|:----------|:--------|
| Core algorithms (PSO, ACO, GWO, WOA, etc.) | Apache 2.0 (Open Source) |
| Enterprise features | Commercial license available |

**Core Library:** Licensed under the [Apache License 2.0](LICENSE) - free for personal and commercial use.

**Enterprise Licensing:** For commercial deployments requiring dedicated support, custom integrations, or proprietary features, contact: **m.s.rajpoot20@gmail.com**

---

<div align="center">

**Built with :heart: in Rust**

[:star: Star this repo](https://github.com/mahii6991/swarm-manager) | [:book: Documentation](https://mahii6991.github.io/drone-swarm-system/) | [:bug: Report Issues](https://github.com/mahii6991/swarm-manager/issues)

</div>
