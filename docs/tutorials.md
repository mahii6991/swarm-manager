# Tutorials

Step-by-step guides to help you build real-world drone swarm applications.

## Getting Started Tutorials

### Your First Swarm
Create a simple 3-drone swarm with formation control and mesh networking in under 10 minutes. See the [examples](https://github.com/mahii6991/drone-swarm-system/tree/main/examples) directory.

### Path Planning with PSO
Implement particle swarm optimization for efficient multi-waypoint navigation. Run `cargo run --example pso_optimization`.

### Secure Communication
Set up end-to-end encrypted communication with military-grade cryptography. See [Security Guide](security.md).

## Hardware Integration

### STM32 Deployment
Deploy your swarm system on STM32F4/F7 microcontrollers with embedded HAL. See [STM32 Guide](hardware-stm32.md).

### ESP32 WiFi Mesh
Build a WiFi mesh network using ESP32 for cost-effective swarm communication. See [ESP32 Guide](hardware-esp32.md).

### PX4/ArduPilot Integration
Integrate with popular flight controllers using MAVLink protocol. See [PX4 Guide](hardware-px4.md).

## Algorithm Deep Dives

### Implementing Custom PSO Topologies
Learn how to create custom particle swarm topologies for specific optimization problems. Run `cargo run --example pso_optimization`.

### ACO for 3D Path Planning
Master ant colony optimization for obstacle avoidance in 3D environments. Run `cargo run --example aco_path_planning`.

### Grey Wolf Optimizer
Fine-tune GWO parameters for multi-objective swarm coordination. Run `cargo run --example gwo_swarm_optimization`.

## Example Projects

Check out the [examples directory](https://github.com/mahii6991/drone-swarm-system/tree/main/examples) for runnable demonstrations:

- `simple_swarm.rs` - Basic swarm initialization
- `pso_optimization.rs` - Particle swarm optimization
- `aco_path_planning.rs` - Ant colony path planning
- `gwo_swarm_optimization.rs` - Grey wolf optimizer
- `collision_avoidance_demo.rs` - Collision avoidance algorithms
- `telemetry_monitoring.rs` - Telemetry and monitoring

---

Need help? [Ask on GitHub Discussions](https://github.com/mahii6991/drone-swarm-system/discussions).
