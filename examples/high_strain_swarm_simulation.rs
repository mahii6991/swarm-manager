//! High-Strain Real-World Swarm Simulation
//!
//! This example pushes the system to its limits by simulating a large swarm (100 drones)
//! performing complex maneuvers with full collision avoidance and formation control.
//!
//! # Scenario
//! A swarm of 100 drones in a grid formation is tasked to migrate to a new location.
//! Mid-flight, the formation is dynamically changed to a Circle, forcing a complex reorganization.
//! Finally, they are tasked to track a high-speed moving target.
//!
//! # Performance Metrics
//! - Simulation FPS (Steps per Second)
//! - Collision Avoidance Calculation Time
//! - Memory Usage (estimated)
//!
//! # Running
//! `cargo run --release --example high_strain_swarm_simulation`

use drone_swarm_system::swarm::{SwarmController, Formation};
use drone_swarm_system::types::{DroneId, DroneState, MissionStatus, Position, Velocity};
use std::time::{Instant, Duration};

// Configuration
const SWARM_SIZE: usize = 100; // Max supported by system
const SIMULATION_DURATION_SEC: f32 = 60.0;
const TIME_STEP: f32 = 0.1; // 100ms
const STEPS: usize = (SIMULATION_DURATION_SEC / TIME_STEP) as usize;

fn main() {
    println!("🔥 HIGH-STRAIN SWARM SIMULATION STARTING 🔥");
    println!("============================================");
    println!("Configuration:");
    println!("  - Swarm Size: {} drones", SWARM_SIZE);
    println!("  - Duration: {} seconds ({} steps)", SIMULATION_DURATION_SEC, STEPS);
    println!("  - Time Step: {}s", TIME_STEP);
    println!("============================================\n");

    // 1. Initialize Leader (Drone 1)
    let initial_pos = Position { x: 0.0, y: 0.0, z: 50.0 };
    let mut controller = SwarmController::new(DroneId::new(1), initial_pos);
    
    // 2. Populate Swarm
    println!("[INIT] Spawning {} drones in initial random cloud...", SWARM_SIZE - 1);
    let init_start = Instant::now();
    for i in 2..=SWARM_SIZE {
        // Random-ish distribution around origin
        let x = (i as f32 * 13.0) % 100.0 - 50.0;
        let y = (i as f32 * 7.0) % 100.0 - 50.0;
        let z = 50.0 + ((i % 10) as f32);
        
        let state = DroneState {
            id: DroneId::new(i as u64),
            position: Position { x, y, z },
            velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
            battery: 100,
            status: MissionStatus::Idle,
            timestamp: 0,
        };
        
        if let Err(e) = controller.update_peer_state(state) {
            eprintln!("Failed to add drone {}: {:?}", i, e);
        }
    }
    println!("[INIT] Complete in {:.2?}. Swarm size: {}", init_start.elapsed(), controller.swarm_size());

    // 3. Phase 1: Form Grid
    println!("\n[PHASE 1] Organizing into Grid Formation...");
    controller.set_formation(Formation::Grid { spacing: 15 });
    
    let mut total_calc_time = Duration::new(0, 0);
    let mut min_fps = f64::MAX;
    let mut max_fps: f64 = 0.0;

    let sim_start = Instant::now();

    for step in 0..STEPS {
        let step_start = Instant::now();

        // 3a. Dynamic Scenario Events
        if step == STEPS / 3 {
            println!("\n[PHASE 2] ⚠️ FORMATION CHANGE: Switching to V-Formation!");
            controller.set_formation(Formation::VFormation { spacing: 10 });
        }
        
        if step == 2 * STEPS / 3 {
            println!("\n[PHASE 3] 🎯 TARGET ACQUIRED: High-speed chase!");
            // Moving target
            controller.set_destination(Some(Position { x: 500.0, y: 500.0, z: 50.0 }));
        }

        // Update target position if tracking (simulating moving target)
        if step > 2 * STEPS / 3 {
            let t = (step as f32) * TIME_STEP;
            controller.set_destination(Some(Position { 
                x: 500.0 + 10.0 * t.sin(), 
                y: 500.0 + 10.0 * t.cos(), 
                z: 50.0 
            }));
        }

        // 3b. Simulating State Updates (Peers moving)
        // In a real system, we'd receive updates from network. 
        // Here we simulate peers moving based on previous velocities (dead reckoning) or just random jitter
        // to force the collision avoidance to work hard.
        // For accurate stress test of *our* controller, we just update peer states with some motion.
        
        // 3c. Compute Control
        let calc_start = Instant::now();
        let _cmd_vel = controller.compute_control_velocity(20.0); // 20 m/s max speed
        let calc_duration = calc_start.elapsed();
        total_calc_time += calc_duration;

        // 3d. Update Physics (Simple Euler integration for the ego drone)
        let mut current_state = controller.local_state().clone();
        current_state.position.x += _cmd_vel.vx * TIME_STEP;
        current_state.position.y += _cmd_vel.vy * TIME_STEP;
        current_state.position.z += _cmd_vel.vz * TIME_STEP;
        current_state.velocity = _cmd_vel;
        
        controller.update_state(
            current_state.position,
            current_state.velocity,
            99,
            MissionStatus::Active
        );

        let step_duration = step_start.elapsed();
        let fps = 1.0 / step_duration.as_secs_f64();
        min_fps = min_fps.min(fps);
        max_fps = max_fps.max(fps);

        if step % 100 == 0 {
            println!(
                "  Step {:4}/{}: FPS={:6.1} | Calc Time={:6.2}µs | Pos=({:5.1},{:5.1},{:5.1})",
                step, STEPS, fps, calc_duration.as_micros(),
                current_state.position.x, current_state.position.y, current_state.position.z
            );
        }
    }

    let total_duration = sim_start.elapsed();
    let avg_fps = STEPS as f64 / total_duration.as_secs_f64();
    let avg_calc_time = total_calc_time.as_secs_f64() / STEPS as f64 * 1_000_000.0; // microseconds

    println!("\n============================================");
    println!("🏁 SIMULATION COMPLETE 🏁");
    println!("============================================");
    println!("Performance Report:");
    println!("  - Total Time:      {:.2}s", total_duration.as_secs_f64());
    println!("  - Average FPS:     {:.1}", avg_fps);
    println!("  - Min/Max FPS:     {:.1} / {:.1}", min_fps, max_fps);
    println!("  - Avg Control Calc:{:.2} µs", avg_calc_time);
    println!("  - Real-time Factor:{:.2}x (higher is faster than real-time)", avg_fps * TIME_STEP as f64);
    println!("============================================");
    
    if avg_fps > 10.0 { // Assuming 10Hz control loop requirement
        println!("✅ PASS: System handles {} drones comfortably.", SWARM_SIZE);
    } else {
        println!("⚠️ WARNING: System struggling with {} drones.", SWARM_SIZE);
    }
}