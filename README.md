# Drone Swarm Communication System

[![Rust](https://img.shields.io/badge/rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![Build Status](https://img.shields.io/github/actions/workflow/status/mahii6991/drone-swarm-system/ci.yml?branch=main)](https://github.com/mahii6991/drone-swarm-system/actions)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

A safety-critical drone swarm communication system in Rust with military-grade security, distributed consensus, and swarm intelligence. Designed for embedded systems with zero heap allocation.

## Features

- **Security**: ChaCha20-Poly1305 encryption, Ed25519 signatures, Byzantine fault tolerance
- **Networking**: Adaptive mesh routing, multi-hop communication, 100+ drone support
- **Consensus**: Raft-based distributed agreement (SwarmRaft)
- **Swarm Intelligence**: PSO, ACO, GWO optimization algorithms
- **Federated Learning**: Decentralized model training
- **Fault Tolerance**: Self-healing with automatic failover

## Quick Start

```bash
git clone https://github.com/mahii6991/drone-swarm-system.git
cd drone-swarm-system
cargo build --release
cargo test
cargo run --example simple_swarm
```

## Usage

```rust
use drone_swarm_system::*;

let drone_id = DroneId::new(1);
let position = Position { x: 0.0, y: 0.0, z: 10.0 };
let swarm = SwarmController::new(drone_id, position);
swarm.set_formation(Formation::Circle { radius: 50 });
```

## Supported Platforms

STM32, ESP32, nRF52, RISC-V, x86/ARM64

## License

Apache 2.0 - see [LICENSE](LICENSE)
