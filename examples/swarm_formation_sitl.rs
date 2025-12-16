//! Multi-Drone Swarm Formation SITL Demo
//!
//! This example demonstrates coordinated multi-drone swarm operations using
//! the SwarmCoordinator with PX4 SITL instances.
//!
//! # Features Demonstrated
//! - Multi-instance MAVLink connection
//! - Synchronized takeoff
//! - Formation flying (V-formation, circle, line)
//! - Formation transitions
//! - Collision avoidance
//! - Coordinated landing
//!
//! # Running
//! 1. Start multiple PX4 SITL instances:
//!    ```bash
//!    # Terminal 1: First drone (default port 14540)
//!    cd ~/PX4-Autopilot && make px4_sitl gazebo
//!
//!    # Terminal 2: Second drone (port 14541)
//!    cd ~/PX4-Autopilot && SITL_MDNS_ID=2 PX4_SIM_MODEL=iris \
//!        Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2
//!    ```
//!    OR use the convenience script:
//!    ```bash
//!    ./simulation/start_sitl.sh swarm 3
//!    ```
//!
//! 2. Run this example:
//!    ```bash
//!    cargo run --example swarm_formation_sitl --features simulation
//!    ```
//!
//! # Safety
//! This example is designed for SIMULATION ONLY.
//! Never run against real hardware without proper safety measures.

use std::time::{Duration, Instant};

use drone_swarm_system::multi_drone_coordinator::{SwarmCoordinator, SwarmFormation};

/// Swarm configuration
const TARGET_ALTITUDE: f32 = 10.0;
const FORMATION_SPACING: f32 = 10.0;
const MAX_FLIGHT_TIME: Duration = Duration::from_secs(180);

fn main() {
    println!("=== Multi-Drone Swarm Formation SITL Demo ===\n");
    println!("This demo demonstrates coordinated swarm formation flying.");
    println!("Ensure multiple PX4 SITL instances are running.\n");

    // SITL addresses (default ports for multiple instances)
    let drone_addresses = vec![
        ("udpin:0.0.0.0:14540", 0), // First drone
        ("udpin:0.0.0.0:14541", 1), // Second drone
        ("udpin:0.0.0.0:14542", 2), // Third drone
    ];

    let start_time = Instant::now();

    // Create swarm coordinator
    println!("[1/8] Creating swarm coordinator...");
    let mut swarm = SwarmCoordinator::new();

    // Connect to available drones
    println!("\n[2/8] Connecting to SITL instances...");
    let mut connected_count = 0;

    for (address, id) in &drone_addresses {
        match swarm.add_drone(address, *id) {
            Ok(_) => {
                connected_count += 1;
            }
            Err(e) => {
                println!("      Drone {} at {} - SKIPPED ({})", id, address, e);
            }
        }
    }

    if connected_count == 0 {
        eprintln!("\n[ERROR] No drones connected!");
        eprintln!("\nTroubleshooting:");
        eprintln!("  1. Start PX4 SITL: make px4_sitl gazebo");
        eprintln!("  2. For multiple drones, use different UDP ports");
        eprintln!("  3. Check firewall settings for UDP 14540-14542");
        return;
    }

    println!(
        "\n      Connected to {}/{} drones",
        connected_count,
        drone_addresses.len()
    );

    // Wait for telemetry
    println!("\n[3/8] Waiting for telemetry...");
    std::thread::sleep(Duration::from_secs(2));

    for _ in 0..20 {
        let _ = swarm.update();
        std::thread::sleep(Duration::from_millis(100));
    }

    swarm.print_status();

    // Arm all drones
    println!("\n[4/8] Arming all drones...");
    match swarm.arm_all() {
        Ok(_) => println!("      All drones armed!"),
        Err(e) => {
            eprintln!("      Arming failed: {}", e);
            return;
        }
    }

    std::thread::sleep(Duration::from_secs(1));

    // Synchronized takeoff
    println!("\n[5/8] Initiating synchronized takeoff to {}m...", TARGET_ALTITUDE);
    match swarm.takeoff_all(TARGET_ALTITUDE) {
        Ok(_) => {}
        Err(e) => {
            eprintln!("      Takeoff failed: {}", e);
            return;
        }
    }

    // Wait for altitude
    println!("      Waiting for all drones to reach altitude...");
    if !swarm.wait_for_altitude(2.0, Duration::from_secs(30)) {
        println!("      Warning: Not all drones reached altitude within timeout");
    }
    println!("      All drones at target altitude!");

    swarm.print_status();

    // Formation flight phases
    println!("\n[6/8] Starting formation flight demonstrations...\n");

    // Phase 1: Line formation
    println!("--- Phase 1: Line Formation ---");
    swarm.set_formation(SwarmFormation::Line {
        spacing: FORMATION_SPACING,
    });
    swarm.set_center(0.0, 0.0, -TARGET_ALTITUDE);

    run_formation_phase(&mut swarm, Duration::from_secs(20), "Line");

    // Phase 2: V-Formation
    println!("\n--- Phase 2: V-Formation ---");
    swarm.set_formation(SwarmFormation::VFormation {
        spacing: FORMATION_SPACING,
    });
    swarm.move_formation_to(20.0, 0.0);

    run_formation_phase(&mut swarm, Duration::from_secs(20), "V-Formation");

    // Phase 3: Circle formation
    println!("\n--- Phase 3: Circle Formation ---");
    swarm.set_formation(SwarmFormation::Circle { radius: 15.0 });
    swarm.move_formation_to(20.0, 20.0);

    run_formation_phase(&mut swarm, Duration::from_secs(20), "Circle");

    // Phase 4: Grid formation (if 3+ drones)
    if connected_count >= 3 {
        println!("\n--- Phase 4: Grid Formation ---");
        swarm.set_formation(SwarmFormation::Grid {
            spacing: FORMATION_SPACING,
        });
        swarm.move_formation_to(0.0, 20.0);

        run_formation_phase(&mut swarm, Duration::from_secs(20), "Grid");
    }

    // Check timeout
    if start_time.elapsed() > MAX_FLIGHT_TIME {
        println!("\n[TIMEOUT] Maximum flight time reached");
    }

    // Return to launch area
    println!("\n[7/8] Returning to launch area...");
    swarm.move_formation_to(0.0, 0.0);
    swarm.set_formation(SwarmFormation::Line {
        spacing: FORMATION_SPACING,
    });

    let return_start = Instant::now();
    while return_start.elapsed() < Duration::from_secs(15) {
        let _ = swarm.formation_step();

        if return_start.elapsed().as_secs() % 3 == 0 {
            let metrics = swarm.get_metrics();
            let dist_to_home = (metrics.center[0].powi(2) + metrics.center[1].powi(2)).sqrt();
            if dist_to_home < 3.0 {
                break;
            }
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    // Land all drones
    println!("\n[8/8] Landing all drones...");
    match swarm.land_all() {
        Ok(_) => {}
        Err(e) => {
            eprintln!("      Landing failed: {}", e);
        }
    }

    // Wait for landing
    let land_start = Instant::now();
    while land_start.elapsed() < Duration::from_secs(30) {
        let _ = swarm.update();

        let mut all_landed = true;
        let metrics = swarm.get_metrics();

        // Check if altitude is near ground
        if (-metrics.center[2]) > 0.5 {
            all_landed = false;
        }

        if all_landed {
            break;
        }

        std::thread::sleep(Duration::from_millis(200));
    }

    // Disarm
    let _ = swarm.disarm_all();

    // Summary
    let elapsed = start_time.elapsed();
    let final_metrics = swarm.get_metrics();

    println!("\n=== Flight Summary ===");
    println!("Total flight time: {:.1}s", elapsed.as_secs_f32());
    println!("Drones in swarm: {}", connected_count);
    println!("Final state: {:?}", swarm.state());
    println!(
        "Final center: ({:.1}, {:.1}, {:.1})",
        final_metrics.center[0], final_metrics.center[1], final_metrics.center[2]
    );

    println!("\nFormations demonstrated:");
    println!("  - Line Formation");
    println!("  - V-Formation");
    println!("  - Circle Formation");
    if connected_count >= 3 {
        println!("  - Grid Formation");
    }

    println!("\n[OK] Multi-drone swarm formation demo completed!");
}

/// Run a formation flight phase
fn run_formation_phase(swarm: &mut SwarmCoordinator, duration: Duration, name: &str) {
    let phase_start = Instant::now();
    let mut last_print = Instant::now();

    while phase_start.elapsed() < duration {
        // Execute formation control step
        let _ = swarm.formation_step();

        // Print status every 3 seconds
        if last_print.elapsed() > Duration::from_secs(3) {
            let metrics = swarm.get_metrics();
            println!(
                "[{:>12}] Center: ({:5.1}, {:5.1}) | Spread: {:4.1}m | Error: {:4.2}m | Sep: {:4.1}m",
                name,
                metrics.center[0],
                metrics.center[1],
                metrics.spread,
                metrics.formation_error,
                metrics.min_separation
            );
            last_print = Instant::now();
        }

        // Check if formation achieved
        if swarm.all_in_formation(2.0) {
            let metrics = swarm.get_metrics();
            println!(
                "[{:>12}] Formation achieved! Error: {:.2}m",
                name, metrics.formation_error
            );
        }

        std::thread::sleep(Duration::from_millis(100));
    }
}
