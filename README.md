<div align="center">

# Drone Swarm System

### Military-Grade Autonomous Drone Swarm Communication & Intelligence

[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange?style=flat-square&logo=rust)](https://www.rust-lang.org/)
[![Build](https://img.shields.io/github/actions/workflow/status/mahii6991/drone-swarm-system/ci.yml?branch=main&style=flat-square)](https://github.com/mahii6991/drone-swarm-system/actions)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue?style=flat-square)](LICENSE)
[![Docs](https://img.shields.io/badge/Docs-GitHub%20Pages-green?style=flat-square)](https://mahii6991.github.io/drone-swarm-system/)

---

**A safety-critical drone swarm system built in Rust with zero heap allocation, Byzantine fault tolerance, and real-time swarm intelligence algorithms.**

</div>

---

## Features

| | Feature | Description |
|:---:|:---|:---|
| :shield: | **Military-Grade Security** | ChaCha20-Poly1305 encryption, Ed25519 signatures, Byzantine fault tolerance |
| :globe_with_meridians: | **Mesh Networking** | Adaptive multi-hop routing supporting 100+ drones with self-healing topology |
| :brain: | **Swarm Intelligence** | PSO, ACO, GWO optimization algorithms for collective decision making |
| :handshake: | **Distributed Consensus** | Raft-based SwarmRaft protocol for leader election and state agreement |
| :robot: | **Federated Learning** | Privacy-preserving decentralized model training across the swarm |
| :zap: | **Real-Time Performance** | Zero heap allocation, `no_std` compatible for embedded systems |

---

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/mahii6991/drone-swarm-system.git
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
use drone_swarm_system::*;

// Initialize a drone
let drone_id = DroneId::new(1);
let position = Position { x: 0.0, y: 0.0, z: 10.0 };

// Create swarm controller
let mut swarm = SwarmController::new(drone_id, position);

// Set formation
swarm.set_formation(Formation::Circle { radius: 50 });

// Initialize mesh network
let mut network = MeshNetwork::new(drone_id);
network.update_position(position);
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      DRONE SWARM SYSTEM                         │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Security  │  │  Consensus  │  │   Network   │             │
│  │  ChaCha20   │  │  SwarmRaft  │  │    Mesh     │             │
│  │   Ed25519   │  │   Leader    │  │   Routing   │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │     PSO     │  │     ACO     │  │     GWO     │             │
│  │  Particle   │  │  Ant Colony │  │  Grey Wolf  │             │
│  │   Swarm     │  │ Optimization│  │  Optimizer  │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│  ┌─────────────────────────────────────────────────┐           │
│  │              Federated Learning                  │           │
│  │         Privacy-Preserving ML Training           │           │
│  └─────────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Modules

| Module | Description |
|:-------|:------------|
| `crypto` | ChaCha20-Poly1305 encryption & Ed25519 signatures |
| `network` | Adaptive mesh routing with multi-hop support |
| `consensus` | SwarmRaft distributed consensus protocol |
| `swarm` | Formation control & collective behavior |
| `pso` | Particle Swarm Optimization |
| `aco` | Ant Colony Optimization for path planning |
| `gwo` | Grey Wolf Optimizer for swarm coordination |
| `federated` | Decentralized federated learning |
| `fault_tolerance` | Self-healing & automatic failover |
| `security` | Intrusion detection & threat mitigation |

---

## Visualization

The interactive GUI provides real-time visualization of:

- **Drone Formations** - Circle, Grid, Line, V-Formation, Random
- **Algorithm Visualization** - PSO particles, ACO pheromone trails, GWO wolf hierarchy
- **Network Topology** - Link quality, routing paths, mesh connectivity
- **Metrics Dashboard** - Convergence graphs, FPS, drone statistics

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
├── src/                    # Core library
│   ├── crypto.rs           # Encryption & signatures
│   ├── network.rs          # Mesh networking
│   ├── consensus.rs        # SwarmRaft protocol
│   ├── swarm.rs            # Formation control
│   ├── pso.rs              # Particle Swarm Optimization
│   ├── aco.rs              # Ant Colony Optimization
│   ├── gwo.rs              # Grey Wolf Optimizer
│   └── ...
├── visualization/          # Interactive GUI
│   └── src/
│       ├── panels/         # UI panels
│       └── renderers/      # Algorithm visualizers
├── examples/               # Usage examples
├── tests/                  # Comprehensive tests
├── fuzz/                   # Security fuzzing
└── docs/                   # Documentation
```

---

## License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**Built with :heart: in Rust**

[:star: Star this repo](https://github.com/mahii6991/drone-swarm-system) | [:book: Documentation](https://mahii6991.github.io/drone-swarm-system/) | [:bug: Report Issues](https://github.com/mahii6991/drone-swarm-system/issues)

</div>
