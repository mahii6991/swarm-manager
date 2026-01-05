//! Simple swarm example demonstrating basic usage
//!
//! This example shows how to:
//! - Initialize a drone node
//! - Configure security
//! - Join a swarm network
//! - Participate in consensus
//! - Coordinate with other drones

use drone_swarm_system::config::SwarmConfig;
use drone_swarm_system::consensus::ConsensusEngine;
use drone_swarm_system::crypto::{CryptoContext, KeyStore};
use drone_swarm_system::fault_tolerance::FaultTolerance;
use drone_swarm_system::federated::{FederatedCoordinator, GlobalModel, LocalTrainer};
use drone_swarm_system::network::MeshNetwork;
use drone_swarm_system::security::SecurityMonitor;
use drone_swarm_system::swarm::{Formation, SwarmController};
use drone_swarm_system::*;

fn main() -> Result<()> {
    println!("🚁 Initializing Secure Drone Swarm System");
    println!("=========================================\n");

    // Step 1: Create drone identity
    let drone_id = DroneId::new(1);
    println!("✓ Drone ID: {}", drone_id);

    // Step 2: Configure the system
    let config = SwarmConfig::new(drone_id);
    config.validate()?;
    println!("✓ Configuration validated");

    // Step 3: Initialize cryptographic context
    let seed = [42u8; 32]; // In production, use hardware RNG
    let _crypto = CryptoContext::new(seed);
    println!("✓ Cryptographic context initialized");
    println!("  - Using ChaCha20-Poly1305 AEAD encryption");
    println!("  - Using Ed25519 digital signatures");
    println!("  - Using BLAKE3/SHA3 hashing");

    // Step 4: Initialize network layer
    let network = MeshNetwork::new(drone_id);
    println!("✓ Mesh network initialized");

    // Step 5: Initialize consensus engine (Raft-based)
    let _consensus = ConsensusEngine::new(drone_id, 150);
    println!("✓ Consensus engine initialized (SwarmRaft)");

    // Step 6: Initialize swarm controller
    let initial_position = Position {
        x: 0.0,
        y: 0.0,
        z: 10.0,
    };
    let mut swarm = SwarmController::new(drone_id, initial_position);
    swarm.set_formation(Formation::Circle { radius: 50 });
    println!("✓ Swarm controller initialized");
    println!("  - Formation: Circle (radius: 50m)");

    // Step 7: Initialize federated learning
    let model = GlobalModel::new(100)?;
    let key_store = KeyStore::new();
    let _fed_coordinator = FederatedCoordinator::new(drone_id, model.clone(), key_store);
    let mut trainer = LocalTrainer::new(drone_id, model.parameters.clone());
    println!("✓ Federated learning system initialized");
    println!("  - Model parameters: {}", model.parameters.len());

    // Step 8: Initialize fault tolerance
    let mut fault_tolerance = FaultTolerance::new();
    println!("✓ Fault tolerance system initialized");

    // Step 9: Initialize security monitor
    let _security = SecurityMonitor::new();
    println!("✓ Security monitor initialized");
    println!("  - Intrusion detection: Active");
    println!("  - Rate limiting: Active");
    println!("  - Byzantine fault tolerance: Active\n");

    println!("🎯 System Ready for Operation");
    println!("================================\n");

    // Simulation loop
    println!("📡 Starting simulation...\n");

    for round in 0..5 {
        println!("--- Round {} ---", round);

        // 1. Update local state
        let new_position = Position {
            x: round as f32 * 10.0,
            y: round as f32 * 5.0,
            z: 10.0,
        };
        let velocity = Velocity {
            vx: 1.0,
            vy: 0.5,
            vz: 0.0,
        };
        swarm.update_state(
            new_position,
            velocity,
            90 - round * 2,
            MissionStatus::Active,
        );
        println!(
            "  Position: ({:.1}, {:.1}, {:.1})",
            new_position.x, new_position.y, new_position.z
        );

        // 2. Compute control commands
        let control_velocity = swarm.compute_control_velocity(5.0);
        println!(
            "  Control velocity: ({:.2}, {:.2}, {:.2})",
            control_velocity.vx, control_velocity.vy, control_velocity.vz
        );

        // 3. Perform local training
        let loss = trainer.train_step(0.01)?;
        println!("  Training loss: {:.4}", loss);

        // 4. Check system health
        fault_tolerance.reset_watchdog();
        if fault_tolerance.is_healthy() {
            println!("  System health: ✓ Healthy");
        }

        // 5. Network heartbeat would be sent here
        // network.send_heartbeat()?;

        println!();
    }

    println!("✅ Simulation complete!");
    println!("\n📊 System Statistics:");
    println!("  - Swarm size: {}", swarm.swarm_size());
    println!(
        "  - Network messages sent: {}",
        network.statistics().messages_sent
    );
    println!(
        "  - Active faults: {}",
        fault_tolerance.active_fault_count()
    );

    let stats = fault_tolerance.fault_statistics();
    println!("  - Total faults: {}", stats.total_faults);
    println!("  - Resolved faults: {}", stats.resolved_faults);

    Ok(())
}
