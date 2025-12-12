//! ACO SITL Navigation Example
//!
//! This example demonstrates using ACO (Ant Colony Optimization) to plan
//! optimal paths through obstacles, then flying the drone through those
//! waypoints in PX4 SITL simulation.
//!
//! # Running
//! 1. Start PX4 SITL: `cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4 -d`
//! 2. Run this example: `cargo run --example aco_sitl_navigation --features simulation`

use std::f32::consts::PI;
use std::time::{Duration, Instant};

use mavlink::common::{MavCmd, MavMessage, MavModeFlag, PositionTargetTypemask, COMMAND_LONG_DATA};
use mavlink::MavHeader;

// Re-use ACO types for path planning
use drone_swarm_system::aco::{ACOAlgorithm, ACOConfig, ACOOptimizer, Obstacle, Position3D};

const SITL_ADDRESS: &str = "udpout:127.0.0.1:14540";

/// Current drone state from telemetry
#[derive(Debug, Clone, Default)]
struct DroneState {
    position: [f32; 3],
    velocity: [f32; 3],
    armed: bool,
}

/// Send MAVLink command
fn send_command<M: mavlink::Message>(
    conn: &dyn mavlink::MavConnection<M>,
    target_system: u8,
    target_component: u8,
    cmd: MavCmd,
    params: [f32; 7],
) where
    MavMessage: Into<M>,
{
    let header = MavHeader {
        system_id: 255,
        component_id: 0,
        sequence: 0,
    };

    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system,
        target_component,
        command: cmd,
        confirmation: 0,
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
    });

    let _ = conn.send(&header, &msg.into());
}

/// Run ACO path planning with obstacles
fn plan_path_with_aco() -> Vec<[f32; 3]> {
    println!("[ACO] Starting path optimization...\n");

    // Define mission parameters
    let start = Position3D::new(0.0, 0.0, -10.0); // NED: negative Z is up
    let goal = Position3D::new(50.0, 50.0, -10.0);

    // Define flight envelope
    let bounds_min = Position3D::new(-10.0, -10.0, -50.0);
    let bounds_max = Position3D::new(60.0, 60.0, 0.0);

    // Configure ACO for navigation
    let config = ACOConfig {
        algorithm: ACOAlgorithm::ACS, // Best for exploitation
        num_ants: 25,
        max_iterations: 50, // Reduced for faster demo
        pheromone_init: 1.0,
        evaporation_rate: 0.1,
        alpha: 1.0,
        beta: 2.5,
    };

    // Create optimizer
    let mut optimizer = match ACOOptimizer::new(config, start, goal, bounds_min, bounds_max) {
        Ok(opt) => opt,
        Err(e) => {
            eprintln!("[ERROR] Failed to create ACO optimizer: {:?}", e);
            return generate_fallback_waypoints();
        }
    };

    // Add virtual obstacles (buildings, no-fly zones)
    let obstacles = [
        Obstacle::new(Position3D::new(15.0, 15.0, -10.0), 8.0), // Building 1
        Obstacle::new(Position3D::new(30.0, 25.0, -10.0), 6.0), // Building 2
        Obstacle::new(Position3D::new(25.0, 40.0, -10.0), 7.0), // Building 3
    ];

    println!("[ACO] Mission parameters:");
    println!("  Start: ({:.1}, {:.1}, {:.1})", start.x, start.y, start.z);
    println!("  Goal:  ({:.1}, {:.1}, {:.1})", goal.x, goal.y, goal.z);
    println!("  Obstacles: {}", obstacles.len());

    for (i, obs) in obstacles.iter().enumerate() {
        if let Err(e) = optimizer.add_obstacle(*obs) {
            eprintln!("[WARNING] Failed to add obstacle {}: {:?}", i, e);
        } else {
            println!(
                "    Obstacle {}: center=({:.1}, {:.1}), radius={:.1}m",
                i, obs.center.x, obs.center.y, obs.radius
            );
        }
    }

    // Run optimization
    println!("\n[ACO] Running Ant Colony System optimization...");
    let start_time = Instant::now();

    let result = match optimizer.optimize() {
        Ok(path) => path,
        Err(e) => {
            eprintln!("[ERROR] Optimization failed: {:?}", e);
            return generate_fallback_waypoints();
        }
    };

    let elapsed = start_time.elapsed();

    println!(
        "\n[ACO] Optimization complete in {:.2}ms",
        elapsed.as_secs_f32() * 1000.0
    );
    println!("  Path found: {}", result.is_valid);
    println!("  Path cost: {:.2}m", result.cost);
    println!("  Waypoints: {}", result.waypoints.len());

    // Convert ACO waypoints to flight waypoints
    if result.is_valid && !result.waypoints.is_empty() {
        let waypoints: Vec<[f32; 3]> = result
            .waypoints
            .iter()
            .map(|wp| [wp.x, wp.y, wp.z])
            .collect();

        println!("\n[ACO] Planned waypoints:");
        for (i, wp) in waypoints.iter().enumerate() {
            println!("  WP{}: ({:6.1}, {:6.1}, {:6.1})", i, wp[0], wp[1], wp[2]);
        }

        waypoints
    } else {
        println!("[WARNING] ACO did not find valid path, using fallback");
        generate_fallback_waypoints()
    }
}

/// Generate fallback waypoints if ACO fails
fn generate_fallback_waypoints() -> Vec<[f32; 3]> {
    // Simple circle pattern as fallback
    let mut waypoints = Vec::new();
    let radius = 15.0;
    let altitude = -10.0;
    let num_points = 8;

    for i in 0..num_points {
        let angle = 2.0 * PI * (i as f32) / (num_points as f32);
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        waypoints.push([x, y, altitude]);
    }

    waypoints
}

fn main() {
    println!("=== ACO SITL Navigation Demo ===\n");
    println!("This demo combines Ant Colony Optimization for path planning");
    println!("with PX4 SITL for drone flight simulation.\n");

    // Phase 1: Path Planning with ACO
    println!("--- Phase 1: ACO Path Planning ---\n");
    let waypoints = plan_path_with_aco();

    // Phase 2: Connect to SITL
    println!("\n--- Phase 2: SITL Connection ---\n");
    println!("Connecting to SITL at {}...", SITL_ADDRESS);

    let conn = match mavlink::connect::<MavMessage>(SITL_ADDRESS) {
        Ok(c) => {
            println!("[OK] Connected to PX4 SITL!");
            c
        }
        Err(e) => {
            eprintln!("[ERROR] Connection failed: {}", e);
            eprintln!("\nMake sure PX4 SITL is running:");
            eprintln!("  cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4 -d");
            return;
        }
    };

    // Phase 3: Execute Flight Plan
    println!("\n--- Phase 3: Flight Execution ---\n");

    // State tracking
    let mut state = DroneState::default();
    let mut current_waypoint = 0;
    let start_time = Instant::now();
    let mut last_wp_time = Instant::now();
    let mut position_count = 0;
    let mut waypoints_reached = 0;

    // Arm the drone
    println!("[CMD] Requesting arm...");
    send_command(
        conn.as_ref(),
        1,
        1,
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );

    println!("[NAV] Starting ACO-optimized flight path...");
    println!("Will navigate through {} ACO waypoints\n", waypoints.len());

    // Main control loop
    loop {
        // Check timeout (90 seconds for longer path)
        if start_time.elapsed() > Duration::from_secs(90) {
            println!("\n[TIMEOUT] 90 second limit reached");
            break;
        }

        // Receive telemetry
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                MavMessage::LOCAL_POSITION_NED(pos) => {
                    state.position = [pos.x, pos.y, pos.z];
                    state.velocity = [pos.vx, pos.vy, pos.vz];
                    position_count += 1;

                    // Print position occasionally
                    if position_count % 100 == 0 {
                        let target = &waypoints[current_waypoint];
                        let dx = pos.x - target[0];
                        let dy = pos.y - target[1];
                        let dz = pos.z - target[2];
                        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

                        println!(
                            "[POS] ({:6.1}, {:6.1}, {:6.1}) → WP{} dist={:.1}m",
                            pos.x, pos.y, pos.z, current_waypoint, distance
                        );
                    }

                    // Check if reached current waypoint (within 3m)
                    let target = &waypoints[current_waypoint];
                    let dx = pos.x - target[0];
                    let dy = pos.y - target[1];
                    let distance_2d = (dx * dx + dy * dy).sqrt();

                    if distance_2d < 3.0 && last_wp_time.elapsed() > Duration::from_secs(2) {
                        waypoints_reached += 1;
                        println!(
                            "[NAV] ✓ Reached WP{} ({:.1}m) [{}/{}]",
                            current_waypoint,
                            distance_2d,
                            waypoints_reached,
                            waypoints.len()
                        );
                        current_waypoint = (current_waypoint + 1) % waypoints.len();
                        last_wp_time = Instant::now();

                        if current_waypoint == 0 && waypoints_reached >= waypoints.len() {
                            println!("[NAV] ✓ Completed ACO-optimized flight path!");
                            break;
                        }
                    }
                }
                MavMessage::HEARTBEAT(hb) => {
                    let was_armed = state.armed;
                    state.armed =
                        (hb.base_mode.bits() & MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED.bits()) != 0;
                    if state.armed && !was_armed {
                        println!("[STATUS] Drone ARMED");
                    }
                }
                MavMessage::STATUSTEXT(text) => {
                    let msg_text: String = text
                        .text
                        .iter()
                        .take_while(|&&c| c != 0)
                        .map(|&c| c as char)
                        .collect();
                    if !msg_text.is_empty() {
                        println!("[PX4] {}", msg_text);
                    }
                }
                _ => {}
            },
            Err(_) => {
                // Receive error, continue
            }
        }

        // Send position setpoint continuously (for offboard control)
        let target = &waypoints[current_waypoint];
        let header = MavHeader {
            system_id: 255,
            component_id: 0,
            sequence: 0,
        };

        let setpoint = MavMessage::SET_POSITION_TARGET_LOCAL_NED(
            mavlink::common::SET_POSITION_TARGET_LOCAL_NED_DATA {
                time_boot_ms: 0,
                target_system: 1,
                target_component: 1,
                coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_LOCAL_NED,
                type_mask: PositionTargetTypemask::empty(),
                x: target[0],
                y: target[1],
                z: target[2],
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                afx: 0.0,
                afy: 0.0,
                afz: 0.0,
                yaw: 0.0,
                yaw_rate: 0.0,
            },
        );

        let _ = conn.send(&header, &setpoint);
    }

    // Summary
    println!("\n=== Mission Summary ===");
    println!("Algorithm: Ant Colony System (ACS)");
    println!(
        "Waypoints reached: {}/{}",
        waypoints_reached,
        waypoints.len()
    );
    println!("Runtime: {:.1}s", start_time.elapsed().as_secs_f32());
    println!("Position updates: {}", position_count);
    println!("\n[OK] ACO SITL navigation demo completed!");
}
