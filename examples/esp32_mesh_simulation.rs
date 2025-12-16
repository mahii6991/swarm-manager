//! ESP32 Mesh Network Simulation
//!
//! This example simulates a multi-node ESP32 mesh network on desktop.
//! It demonstrates:
//! - Node discovery via heartbeat
//! - Position sharing
//! - Command distribution
//! - Swarm coordination
//!
//! No hardware required - runs entirely in simulation.
//!
//! # Running
//! ```bash
//! cargo run --example esp32_mesh_simulation --features std
//! ```

use std::collections::HashMap;
use std::time::Instant;

use drone_swarm_system::esp32_mesh::{MeshNode, ProcessResult};
use drone_swarm_system::mesh_protocol::{
    CommandAction, CommandTarget, EmergencyType, MeshMessage, MeshNodeId,
};

/// Number of simulated nodes
const NUM_NODES: usize = 5;

/// Simulation steps
const SIMULATION_STEPS: usize = 50;

/// Simulated mesh network (handles message passing between nodes)
struct MeshSimulator {
    nodes: HashMap<u8, MeshNode>,
    pending_messages: Vec<(MeshMessage, i8)>, // (message, rssi)
    time_ms: u64,
}

impl MeshSimulator {
    fn new(num_nodes: usize) -> Self {
        let mut nodes = HashMap::new();

        for i in 0..num_nodes {
            let node_id = MeshNodeId::new(i as u8);
            let mut node = MeshNode::new(node_id);

            // Initialize node
            node.init().expect("Failed to init node");
            node.start().expect("Failed to start node");

            // Set initial positions (spread in a grid)
            let x = (i % 3) as f32 * 20.0;
            let y = (i / 3) as f32 * 20.0;
            let z = 10.0;
            node.set_position([x, y, z]);
            node.set_battery(100 - (i * 5) as u8);

            nodes.insert(i as u8, node);
        }

        Self {
            nodes,
            pending_messages: Vec::new(),
            time_ms: 0,
        }
    }

    /// Simulate one step (100ms)
    fn step(&mut self) {
        self.time_ms += 100;

        // Collect outgoing messages from all nodes
        let mut new_messages = Vec::new();
        for node in self.nodes.values_mut() {
            // Update node (handles periodic heartbeat)
            node.update(self.time_ms).ok();

            // Collect transmitted messages
            while let Some(msg) = node.get_next_tx_message() {
                // Simulate RSSI based on distance (simplified)
                let rssi = -40; // Good signal
                new_messages.push((msg, rssi));
            }
        }

        // Add new messages to pending
        self.pending_messages.extend(new_messages);

        // Deliver messages to all nodes (simulate broadcast)
        let messages: Vec<_> = self.pending_messages.drain(..).collect();
        for (msg, rssi) in messages {
            for (node_id, node) in &mut self.nodes {
                // Don't deliver to sender
                if msg.source.as_u8() == *node_id {
                    continue;
                }

                // Process message
                match node.process_message(msg.clone(), rssi, self.time_ms) {
                    Ok(ProcessResult::Command(action)) => {
                        println!("  [Node {}] Received command: {:?}", node_id, action);
                    }
                    Ok(ProcessResult::Emergency {
                        node_id: src,
                        emergency_type,
                        position,
                    }) => {
                        println!(
                            "  [Node {}] EMERGENCY from {}: {:?} at ({:.1}, {:.1}, {:.1})",
                            node_id,
                            src.as_u8(),
                            emergency_type,
                            position[0],
                            position[1],
                            position[2]
                        );
                    }
                    _ => {}
                }
            }
        }
    }

    /// Print network status
    fn print_status(&self) {
        println!("\n--- Network Status (t={}ms) ---", self.time_ms);
        for (id, node) in &self.nodes {
            let pos = node.position();
            let stats = node.stats();
            println!(
                "Node {}: pos=({:5.1}, {:5.1}, {:5.1}) | neighbors={} | tx={} rx={} fwd={}",
                id,
                pos[0],
                pos[1],
                pos[2],
                stats.active_neighbors,
                stats.tx_count,
                stats.rx_count,
                stats.forward_count
            );
        }
    }

    /// Inject a command into the network
    fn inject_command(&mut self, from_node: u8, target: CommandTarget, action: CommandAction) {
        let msg = MeshMessage::command(MeshNodeId::new(from_node), target, action, self.time_ms);
        self.pending_messages.push((msg, -30));
    }

    /// Inject an emergency
    fn inject_emergency(&mut self, from_node: u8, emergency_type: EmergencyType) {
        if let Some(node) = self.nodes.get(&from_node) {
            let msg = MeshMessage::emergency(
                MeshNodeId::new(from_node),
                emergency_type,
                node.position(),
                self.time_ms,
            );
            self.pending_messages.push((msg, -30));
        }
    }

    /// Move a node
    fn move_node(&mut self, node_id: u8, new_position: [f32; 3]) {
        if let Some(node) = self.nodes.get_mut(&node_id) {
            node.set_position(new_position);
        }
    }

    /// Get swarm center
    fn get_swarm_center(&self) -> [f32; 3] {
        let mut center = [0.0, 0.0, 0.0];
        let count = self.nodes.len() as f32;

        for node in self.nodes.values() {
            let pos = node.position();
            center[0] += pos[0];
            center[1] += pos[1];
            center[2] += pos[2];
        }

        center[0] /= count;
        center[1] /= count;
        center[2] /= count;

        center
    }
}

fn main() {
    println!("=== ESP32 Mesh Network Simulation ===\n");
    println!("Simulating {} ESP32 nodes in a mesh network", NUM_NODES);
    println!("This demonstrates the mesh protocol without hardware.\n");

    let start_time = Instant::now();

    // Create simulator
    let mut sim = MeshSimulator::new(NUM_NODES);

    println!("--- Phase 1: Node Discovery ---");
    println!("Nodes broadcast heartbeats to discover neighbors.\n");

    // Run initial discovery (10 steps)
    for _ in 0..10 {
        sim.step();
    }
    sim.print_status();

    println!("\n--- Phase 2: Command Distribution ---");
    println!("Leader (Node 0) sends ARM command to all nodes.\n");

    // Node 0 sends ARM command
    sim.inject_command(0, CommandTarget::Broadcast, CommandAction::Arm);
    sim.step();
    sim.step();

    println!("\n--- Phase 3: Formation Control ---");
    println!("Nodes move into circle formation.\n");

    // Move nodes into circle formation
    let radius = 25.0;
    for i in 0..NUM_NODES {
        let angle = 2.0 * std::f32::consts::PI * (i as f32) / (NUM_NODES as f32);
        let x = radius * angle.cos() + 25.0;
        let y = radius * angle.sin() + 25.0;
        sim.move_node(i as u8, [x, y, 15.0]);
    }

    // Run a few steps to propagate position updates
    for _ in 0..5 {
        sim.step();
    }
    sim.print_status();

    let center = sim.get_swarm_center();
    println!(
        "\nSwarm center: ({:.1}, {:.1}, {:.1})",
        center[0], center[1], center[2]
    );

    println!("\n--- Phase 4: Emergency Handling ---");
    println!("Node 2 reports low battery emergency.\n");

    sim.inject_emergency(2, EmergencyType::LowBattery);
    sim.step();
    sim.step();

    println!("\n--- Phase 5: Waypoint Command ---");
    println!("Leader sends GOTO command to Node 3.\n");

    sim.inject_command(
        0,
        CommandTarget::Node(MeshNodeId::new(3)),
        CommandAction::GoTo {
            position: [50.0, 50.0, 20.0],
        },
    );
    sim.step();

    println!("\n--- Phase 6: Continued Operation ---");
    println!(
        "Running {} more simulation steps...\n",
        SIMULATION_STEPS - 20
    );

    for step in 20..SIMULATION_STEPS {
        sim.step();

        // Print status every 10 steps
        if step % 10 == 0 {
            println!("Step {}: {} nodes active", step, sim.nodes.len());
        }
    }

    sim.print_status();

    // Final statistics
    let elapsed = start_time.elapsed();
    println!("\n=== Simulation Complete ===");
    println!("Total time: {:.2}ms", elapsed.as_secs_f32() * 1000.0);
    println!("Simulation steps: {}", SIMULATION_STEPS);
    println!("Simulated time: {}ms", sim.time_ms);

    let mut total_tx = 0;
    let mut total_rx = 0;
    for node in sim.nodes.values() {
        total_tx += node.stats().tx_count;
        total_rx += node.stats().rx_count;
    }
    println!("Total messages: tx={} rx={}", total_tx, total_rx);

    println!("\n[OK] ESP32 mesh simulation completed!");
    println!("\nNext steps:");
    println!("  - Flash to real ESP32: cargo espflash flash --release");
    println!("  - See docs/hardware-esp32.md for hardware setup");
}
