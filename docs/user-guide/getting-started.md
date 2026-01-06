# Getting Started

This guide will help you get the Drone Swarm Communication System up and running on your machine.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Your First Swarm](#your-first-swarm)
- [Basic Configuration](#basic-configuration)
- [Running Examples](#running-examples)
- [Next Steps](#next-steps)

## Prerequisites

### Required

- **Rust**: Version 1.70 or higher
  ```bash
  # Install Rust via rustup
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

  # Verify installation
  rustc --version
  cargo --version
  ```

- **Git**: For cloning the repository
  ```bash
  git --version
  ```

### Optional (for embedded deployment)

- **ARM Toolchain**: For embedded targets
  ```bash
  rustup target add thumbv7em-none-eabihf  # Cortex-M4/M7
  rustup target add thumbv8m.main-none-eabihf  # Cortex-M33
  ```

- **probe-rs**: For flashing embedded devices
  ```bash
  cargo install probe-rs --features cli
  ```

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/mahii6991/swarm-manager.git
cd swarm-manager
```

### 2. Build the Project

```bash
# Debug build (faster compilation)
cargo build

# Release build (optimized)
cargo build --release
```

### 3. Run Tests

```bash
# Run all tests
cargo test

# Run tests with output
cargo test -- --nocapture

# Run specific test
cargo test test_encryption
```

### 4. Verify Installation

```bash
# Check formatting
cargo fmt -- --check

# Run linter
cargo clippy

# Generate documentation
cargo doc --open
```

## Your First Swarm

Let's create a simple 3-drone swarm that demonstrates the core features.

### Step 1: Create a New Example File

Create `examples/my_first_swarm.rs`:

```rust
use drone_swarm_system::{
    safety::crypto::CryptoContext,
    network::core::MeshNetwork,
    consensus::raft::ConsensusEngine,
    control::swarm::{SwarmController, Formation},
    types::{DroneId, Position},
};

fn main() {
    println!("🚁 Starting My First Drone Swarm!");

    // Initialize 3 drones
    let drones = vec![
        create_drone(1, Position { x: 0.0, y: 0.0, z: 10.0 }),
        create_drone(2, Position { x: 10.0, y: 0.0, z: 10.0 }),
        create_drone(3, Position { x: 5.0, y: 8.66, z: 10.0 }),
    ];

    println!("✅ Created {} drones", drones.len());

    // Set formation
    for (id, controller) in drones {
        println!("Drone {} ready at position: {:?}", id, controller.get_position());
    }

    println!("🎯 Swarm initialized successfully!");
}

fn create_drone(id: u32, position: Position) -> (u32, SwarmController) {
    let drone_id = DroneId::new(id);
    let controller = SwarmController::new(drone_id, position);
    (id, controller)
}
```

### Step 2: Run Your Swarm

```bash
cargo run --example my_first_swarm
```

Expected output:
```
🚁 Starting My First Drone Swarm!
✅ Created 3 drones
Drone 1 ready at position: Position { x: 0.0, y: 0.0, z: 10.0 }
Drone 2 ready at position: Position { x: 10.0, y: 0.0, z: 10.0 }
Drone 3 ready at position: Position { x: 5.0, y: 8.66, z: 10.0 }
🎯 Swarm initialized successfully!
```

## Basic Configuration

### Swarm Configuration

```rust
use drone_swarm_system::system::config::SwarmConfig;
use drone_swarm_system::types::DroneId;

let drone_id = DroneId::new(1);
let mut config = SwarmConfig::new(drone_id);

// Enable security features
config.encryption_enabled = true;
config.signature_verification = true;

// Network settings
config.max_neighbors = 10;
config.comm_range = 1000.0; // 1km range

// Consensus settings
config.consensus_enabled = true;
config.heartbeat_interval = 150; // milliseconds

// Federated learning
config.federated_learning_enabled = true;
config.learning_rate = 0.01;
```

### Cryptographic Setup

```rust
use drone_swarm_system::safety::crypto::CryptoContext;

// IMPORTANT: Use hardware RNG in production!
let seed = [42u8; 32]; // For testing only
let crypto = CryptoContext::new(seed);

// Encrypt a message
let message = b"Hello, Swarm!";
let nonce = [0u8; 12];
let encrypted = crypto.encrypt(message, &nonce);

// Decrypt
let decrypted = crypto.decrypt(&encrypted, &nonce).unwrap();
assert_eq!(message, decrypted.as_slice());
```

### Network Setup

```rust
use drone_swarm_system::network::core::MeshNetwork;
use drone_swarm_system::types::DroneId;

let drone_id = DroneId::new(1);
let mut network = MeshNetwork::new(drone_id);

// Add neighbors
network.add_neighbor(DroneId::new(2), 50.0); // 50m distance
network.add_neighbor(DroneId::new(3), 75.0); // 75m distance

// Send message
let dest = DroneId::new(2);
let payload = b"Hello from Drone 1";
network.send_message(dest, payload);
```

## Running Examples

The repository includes several examples:

### 1. Simple Swarm

```bash
cargo run --example simple_swarm
```

Demonstrates basic swarm initialization and formation control.

### 2. Encrypted Communication

```bash
cargo run --example encrypted_messaging
```

Shows how to use the cryptographic layer for secure communication.

### 3. Consensus Demo

```bash
cargo run --example consensus_demo
```

Demonstrates Raft consensus for distributed decision-making.

### 4. Federated Learning

```bash
cargo run --example federated_learning
```

Shows how drones can collaboratively train a model.

### 5. Path Planning

```bash
cargo run --example path_planning
```

Demonstrates ACO, PSO, and GWO algorithms for optimal path finding.

## Basic Workflow

Here's a typical workflow for using the system:

```rust
use drone_swarm_system::{
    types::{DroneId, Position},
    safety::crypto::CryptoContext,
    network::core::MeshNetwork,
    consensus::raft::ConsensusEngine,
    control::swarm::{SwarmController, Formation},
};

fn main() {
    // 1. Initialize drone
    let drone_id = DroneId::new(1);
    let position = Position { x: 0.0, y: 0.0, z: 10.0 };

    // 2. Setup cryptography
    let seed = get_hardware_seed(); // Use hardware RNG
    let crypto = CryptoContext::new(seed);

    // 3. Initialize network
    let mut network = MeshNetwork::new(drone_id);

    // 4. Initialize consensus
    let mut consensus = ConsensusEngine::new(drone_id, 150);

    // 5. Initialize swarm controller
    let mut swarm = SwarmController::new(drone_id, position);

    // 6. Set formation
    swarm.set_formation(Formation::Grid { spacing: 20 });

    // 7. Main loop
    loop {
        // Receive messages
        if let Some(msg) = network.receive() {
            // Decrypt message
            let decrypted = crypto.decrypt(&msg.payload, &msg.nonce).unwrap();

            // Process message
            handle_message(decrypted);
        }

        // Update consensus
        consensus.tick();

        // Update position
        swarm.update_position();

        // Sleep
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn get_hardware_seed() -> [u8; 32] {
    // TODO: Implement hardware RNG
    [0u8; 32]
}

fn handle_message(msg: Vec<u8>) {
    println!("Received: {:?}", msg);
}
```

## Common Issues

### Issue: Compilation Errors

**Problem**: Missing dependencies or incompatible Rust version

**Solution**:
```bash
# Update Rust
rustup update

# Clean and rebuild
cargo clean
cargo build
```

### Issue: Tests Failing

**Problem**: Some tests may fail due to timing issues

**Solution**:
```bash
# Run tests sequentially
cargo test -- --test-threads=1

# Run with verbose output
cargo test -- --nocapture
```

### Issue: Performance Issues

**Problem**: Debug builds are slow

**Solution**: Always use release builds for performance testing:
```bash
cargo build --release
cargo test --release
```

## Next Steps

Now that you have the basics working:

1. **Read the [Architecture Guide](architecture.md)** - Understand the system design
2. **Explore the [API Reference](api-reference.md)** - Learn about all available modules
3. **Review [Security Best Practices](security.md)** - Secure your deployment
4. **Check out [Examples](https://github.com/mahii6991/swarm-manager/tree/main/examples)** - See real-world usage

## Getting Help

If you run into issues:

- Check the [FAQ](faq.md)
- Search [GitHub Issues](https://github.com/mahii6991/swarm-manager/issues)
- Ask in [GitHub Discussions](https://github.com/mahii6991/swarm-manager/discussions)

## Embedded Deployment

For deploying to embedded systems (STM32, ESP32, etc.):

```bash
# Add target
rustup target add thumbv7em-none-eabihf

# Build for embedded
cargo build --release --target thumbv7em-none-eabihf --no-default-features

# Flash to device
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/swarm-manager
```

See the hardware guides ([ESP32](hardware-esp32.md), [STM32](hardware-stm32.md), [PX4](hardware-px4.md)) for detailed instructions.

---

**Ready to build your swarm?** Check out the [Examples](https://github.com/mahii6991/swarm-manager/tree/main/examples) directory for more advanced use cases!
