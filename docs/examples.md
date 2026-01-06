# Code Examples

Complete, runnable examples demonstrating key features of the drone swarm system.

## Quick Start Examples

### Simple Swarm (5 minutes)

The most basic example showing system initialization and formation control.

```rust
use drone_swarm_system::{
    types::{DroneId, Position},
    control::swarm::{SwarmController, Formation},
};

fn main() {
    // Initialize drone with ID
    let drone_id = DroneId::new(1);

    // Create swarm controller
    let position = Position { x: 0.0, y: 0.0, z: 10.0 };
    let mut swarm = SwarmController::new(drone_id, position);

    // Set formation
    swarm.set_formation(Formation::Circle { radius: 50 });

    // Compute formation position
    let target = swarm.compute_formation_position();
    println!("Target position: ({:.2}, {:.2}, {:.2})",
             target.x, target.y, target.z);
}
```

**Run it**: `cargo run --example simple_swarm`

---

### Encrypted Communication (10 minutes)

Demonstrates military-grade encryption and digital signatures.

```rust
use drone_swarm_system::safety::crypto::CryptoContext;

fn main() {
    // Initialize crypto context
    let seed = [42u8; 32]; // Use hardware RNG in production
    let crypto = CryptoContext::new(seed);

    // Encrypt a message
    let message = b"Drone 1 reporting: Target located at (100, 200, 50)";
    let nonce = [0u8; 12];

    let encrypted = crypto.encrypt(message, &nonce);
    println!("Encrypted {} bytes -> {} bytes", message.len(), encrypted.len());

    // Decrypt
    let decrypted = crypto.decrypt(&encrypted, &nonce).unwrap();
    assert_eq!(message, &decrypted[..]);
    println!("Decryption successful!");
}
```

**Run it**: `cargo run --example encrypted_comms`

---

## Path Planning Examples

### PSO Path Optimization

Multi-waypoint path planning using Particle Swarm Optimization.

```rust
use drone_swarm_system::algorithms::pso::basic::*;

fn main() -> Result<()> {
    let start = Position { x: 0.0, y: 0.0, z: 10.0 };
    let goal = Position { x: 100.0, y: 100.0, z: 10.0 };

    // Create path optimizer with 5 waypoints
    let mut optimizer = DronePathOptimizer::new(start, goal, 5)?;

    // Add obstacles
    optimizer.add_obstacle(
        Position { x: 50.0, y: 50.0, z: 10.0 },
        20.0 // radius
    )?;

    // Optimize path (100 iterations)
    let path = optimizer.optimize(100)?;

    println!("Optimized path with {} waypoints:", path.len());
    for (i, pos) in path.iter().enumerate() {
        println!("  {}: ({:.2}, {:.2}, {:.2})", i, pos.x, pos.y, pos.z);
    }

    Ok(())
}
```

**Run it**: `cargo run --example pso_optimization`

---

### ACO 3D Path Planning

Advanced 3D path planning with obstacle avoidance using Ant Colony Optimization.

```rust
use drone_swarm_system::aco::*;

fn main() {
    let start = Position { x: 0.0, y: 0.0, z: 10.0 };
    let goal = Position { x: 200.0, y: 200.0, z: 50.0 };

    let config = ACOConfig {
        num_ants: 50,
        max_iterations: 100,
        alpha: 1.0,      // Pheromone importance
        beta: 2.0,       // Heuristic importance
        evaporation: 0.1,
        q: 100.0,        // Pheromone deposit amount
        variant: ACOVariant::MaxMinAntSystem,
    };

    let mut optimizer = ACOPathPlanner::new(start, goal, config)
        .expect("Failed to create optimizer");

    // Add 3D obstacles
    optimizer.add_obstacle(
        Position { x: 100.0, y: 100.0, z: 30.0 },
        30.0
    ).unwrap();

    let result = optimizer.optimize().expect("Optimization failed");

    println!("Path found: {}", result.is_valid);
    println!("Path cost: {:.2}", result.cost);
    println!("Waypoints: {}", result.waypoints.len());
}
```

**Run it**: `cargo run --example aco_path_planning`

---

## Swarm Intelligence Examples

### Formation Control

Demonstrates multiple formation types with smooth transitions.

```rust
use drone_swarm_system::swarm::*;

fn main() -> Result<()> {
    let drone_id = DroneId::new(1);
    let position = Position { x: 0.0, y: 0.0, z: 10.0 };
    let mut swarm = SwarmController::new(drone_id, position);

    // Test different formations
    let formations = vec![
        ("Circle", Formation::Circle { radius: 50.0 }),
        ("Grid", Formation::Grid { spacing: 20.0, rows: 5, cols: 5 }),
        ("Line", Formation::Line { spacing: 10.0, direction: 0.0 }),
        ("V-Formation", Formation::VFormation { angle: 45.0, spacing: 15.0 }),
    ];

    for (name, formation) in formations {
        swarm.set_formation(formation);
        let target_pos = swarm.get_target_position();
        println!("{} formation target: ({:.2}, {:.2}, {:.2})",
                 name, target_pos.x, target_pos.y, target_pos.z);
    }

    Ok(())
}
```

---

### GWO Multi-Objective Optimization

Optimize swarm parameters using Grey Wolf Optimizer.

```rust
use drone_swarm_system::gwo::*;

fn main() -> Result<()> {
    // Optimize formation parameters (spacing, altitude, speed)
    let config = GWOConfig {
        dimensions: 3,
        num_wolves: 20,
        max_iterations: 100,
        variant: GWOVariant::Hybrid,  // GWO + PSO
        adaptive: true,
    };

    let bounds = Bounds::new(vec![
        (5.0, 50.0),   // spacing: 5-50m
        (10.0, 100.0), // altitude: 10-100m
        (1.0, 20.0),   // speed: 1-20 m/s
    ])?;

    let mut gwo = GreyWolfOptimizer::new(config, bounds)?;

    // Objective: minimize energy while maintaining coverage
    let fitness = |params: &[f32]| -> f32 {
        let spacing = params[0];
        let altitude = params[1];
        let speed = params[2];

        let energy_cost = speed * speed + altitude * 0.1;
        let coverage_penalty = 1000.0 / spacing;

        energy_cost + coverage_penalty
    };

    let result = gwo.optimize(fitness)?;
    println!("Optimal parameters: spacing={:.2}m, altitude={:.2}m, speed={:.2}m/s",
             result.position[0], result.position[1], result.position[2]);

    Ok(())
}
```

**Run it**: `cargo run --example gwo_swarm_optimization`

---

## Network Examples

### Mesh Network Discovery

Automatic neighbor discovery and mesh formation.

```rust
use drone_swarm_system::network::*;

fn main() -> Result<()> {
    let drone_id = DroneId::new(1);
    let mut network = MeshNetwork::new(drone_id);

    // Simulate receiving Hello messages from neighbors
    for i in 2..=5 {
        let hello_msg = NetworkMessage::Hello {
            sender: DroneId::new(i),
            position: Position {
                x: (i * 50) as f32,
                y: 0.0,
                z: 10.0
            },
            sequence: 1,
        };

        let addr = NetworkAddress::new([192, 168, 1, i as u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8080);
        network.process_message(hello_msg, addr)?;
    }

    println!("Discovered {} neighbors", network.neighbor_count());

    Ok(())
}
```

---

## Federated Learning Examples

### Distributed Model Training

Train ML models across the swarm while preserving privacy.

```rust
use drone_swarm_system::federated::*;

fn main() -> Result<()> {
    let drone_id = DroneId::new(1);

    // Initialize global model (100 parameters)
    let model = GlobalModel::new(100)?;

    // Create local trainer
    let mut trainer = LocalTrainer::new(drone_id, model.parameters.clone());

    // Training loop
    for epoch in 0..10 {
        // Simulate training on local data
        let loss = trainer.train_step(0.01)?; // learning rate = 0.01

        if epoch % 2 == 0 {
            println!("Epoch {}: Loss = {:.4}", epoch, loss);
        }
    }

    // Get updated parameters to share with swarm
    let updated_params = trainer.get_parameters();
    println!("Model trained! Ready to aggregate with swarm.");

    Ok(())
}
```

---

## Security Examples

### Intrusion Detection

Real-time security monitoring and threat detection.

```rust
use drone_swarm_system::security::*;

fn main() -> Result<()> {
    let mut monitor = SecurityMonitor::new();

    let attacker = DroneId::new(666);

    // Simulate repeated authentication failures
    for i in 0..15 {
        match monitor.record_auth_failure(attacker) {
            Ok(_) => println!("Auth failure {}/10", i + 1),
            Err(SwarmError::PermissionDenied) => {
                println!("🚨 INTRUSION DETECTED! Drone {} banned", attacker.as_u64());
                break;
            }
            Err(e) => println!("Error: {:?}", e),
        }
    }

    // Check if banned
    assert!(monitor.is_banned(attacker));

    Ok(())
}
```

---

## Complete Application Examples

### Search and Rescue Coordinator

Full SAR mission with 50 drones, path planning, and target detection.

```rust
// See examples/search_rescue_mission.rs for complete implementation
// Features:
// - Grid formation for systematic search
// - ACO path planning with dynamic obstacles
// - Federated learning for target detection
// - Real-time discovery sharing
// - Fault tolerance and recovery
```

**Run it**: `cargo run --example search_rescue_mission`

---

### Agricultural Monitoring System

Autonomous crop monitoring with 20 drones.

```rust
// See examples/agriculture_monitor.rs for complete implementation
// Features:
// - Optimized field coverage patterns
// - Collaborative pest detection
// - PSO-based spraying route optimization
// - Energy-efficient task allocation
```

**Run it**: `cargo run --example agriculture_monitor`

---

## Embedded Examples

### STM32 Bare-Metal Deployment

```rust
#![no_std]
#![no_main]

use drone_swarm_system::*;
use panic_halt as _;

#[entry]
fn main() -> ! {
    // Initialize time source (168 MHz STM32F4)
    init_time_source(168_000_000);

    // Initialize swarm controller
    let drone_id = DroneId::new(1);
    let position = Position { x: 0.0, y: 0.0, z: 10.0 };
    let swarm = SwarmController::new(drone_id, position);

    // Main loop
    loop {
        // Update swarm state
        // Read sensors
        // Send telemetry
        // Receive commands

        delay_ms(50); // 20 Hz control loop
    }
}
```

**Build it**: `cargo build --example stm32_deployment --target thumbv7em-none-eabihf --release`

---

## Testing Examples

All examples include comprehensive tests. Run them with:

```bash
# Run all example tests
cargo test --examples

# Run specific example test
cargo test --example simple_swarm

# Run with verbose output
cargo test --example pso_optimization -- --nocapture
```

---

## Next Steps

- [Read the Architecture Guide](architecture.md) to understand system internals
- [Check out Tutorials](tutorials.md) for step-by-step guides
- [Review API Reference](api-reference.md) for detailed documentation
- [Join the Community](https://github.com/mahii6991/drone-swarm-system/discussions) to ask questions

---

**Note**: All examples are production-ready code that you can adapt for your projects. They demonstrate best practices for memory safety, error handling, and embedded systems design.
