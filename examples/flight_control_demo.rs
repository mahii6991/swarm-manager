//! MAVLink Flight Control Demo
//!
//! This example demonstrates full flight control capabilities using MAVLink:
//! - Arm/Disarm
//! - Takeoff
//! - Position control (waypoints)
//! - Return to Launch (RTL)
//! - Land
//!
//! # Running
//! 1. Start PX4 SITL: `cd ~/PX4-Autopilot && make px4_sitl gazebo`
//!    OR: `./simulation/start_sitl.sh single`
//! 2. Run this example: `cargo run --example flight_control_demo --features simulation`
//!
//! # Safety
//! This example is designed for SIMULATION ONLY.
//! Never run against real hardware without proper safety measures.

use std::time::{Duration, Instant};

use drone_swarm_system::mavlink_controller::{ControllerError, FlightController, TelemetryState};

/// Waypoint mission
struct Mission {
    waypoints: Vec<[f32; 3]>,
    current_index: usize,
    waypoint_radius: f32,
}

impl Mission {
    fn new() -> Self {
        // Square pattern at 10m altitude
        let waypoints = vec![
            [0.0, 0.0, -10.0],   // Start/home
            [20.0, 0.0, -10.0],  // North
            [20.0, 20.0, -10.0], // North-East
            [0.0, 20.0, -10.0],  // East
            [0.0, 0.0, -10.0],   // Back to start
        ];

        Self {
            waypoints,
            current_index: 0,
            waypoint_radius: 2.0, // meters
        }
    }

    fn current_waypoint(&self) -> Option<[f32; 3]> {
        self.waypoints.get(self.current_index).copied()
    }

    fn advance(&mut self) -> bool {
        if self.current_index < self.waypoints.len() - 1 {
            self.current_index += 1;
            true
        } else {
            false
        }
    }

    #[allow(dead_code)]
    fn is_complete(&self) -> bool {
        self.current_index >= self.waypoints.len() - 1
    }

    fn reached_waypoint(&self, position: [f32; 3]) -> bool {
        if let Some(wp) = self.current_waypoint() {
            let dx = position[0] - wp[0];
            let dy = position[1] - wp[1];
            let distance = (dx * dx + dy * dy).sqrt();
            distance < self.waypoint_radius
        } else {
            false
        }
    }
}

/// Flight state machine
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
enum FlightState {
    Connecting,
    PreArm,
    Arming,
    Armed,
    TakingOff,
    Flying,
    Hovering,
    ReturningHome,
    Landing,
    Landed,
    Disarmed,
    Error,
}

fn print_telemetry(state: &TelemetryState) {
    println!(
        "  Position: ({:6.1}, {:6.1}, {:6.1})m | Alt: {:5.1}m",
        state.position[0], state.position[1], state.position[2], -state.position[2]
    );
    println!(
        "  Velocity: ({:5.1}, {:5.1}, {:5.1})m/s",
        state.velocity[0], state.velocity[1], state.velocity[2]
    );
    println!(
        "  Attitude: Roll={:5.1}° Pitch={:5.1}° Yaw={:5.1}°",
        state.attitude[0].to_degrees(),
        state.attitude[1].to_degrees(),
        state.attitude[2].to_degrees()
    );
    println!(
        "  Battery: {}% ({:.1}V)",
        state.battery_percent, state.battery_voltage
    );
    println!("  Armed: {} | Mode: {}", state.armed, state.mode);
}

fn main() {
    println!("=== MAVLink Flight Control Demo ===\n");
    println!("This demo demonstrates full autonomous flight control.");
    println!("Ensure PX4 SITL is running before proceeding.\n");

    // Configuration
    let sitl_address = "udpin:0.0.0.0:14540";
    let takeoff_altitude = 10.0; // meters
    let max_flight_time = Duration::from_secs(120);

    // Connect to SITL
    println!("[1/7] Connecting to SITL at {}...", sitl_address);
    let mut controller = match FlightController::connect(sitl_address) {
        Ok(c) => {
            println!("      Connected successfully!");
            c
        }
        Err(e) => {
            eprintln!("      Connection failed: {}", e);
            eprintln!("\nTroubleshooting:");
            eprintln!("  1. Start PX4 SITL: make px4_sitl gazebo");
            eprintln!("  2. Or use: ./simulation/start_sitl.sh single");
            return;
        }
    };

    // Initialize mission
    let mut mission = Mission::new();
    let mut flight_state = FlightState::PreArm;
    let start_time = Instant::now();
    let mut last_status_time = Instant::now();
    let mut waypoints_reached = 0;

    println!("\n[2/7] Waiting for telemetry...");

    // Main flight loop
    loop {
        // Check timeout
        if start_time.elapsed() > max_flight_time {
            println!("\n[TIMEOUT] Maximum flight time reached");
            flight_state = FlightState::Landing;
        }

        // Update controller (receive telemetry)
        if let Err(e) = controller.update() {
            eprintln!("Update error: {}", e);
        }

        // Print status every 2 seconds
        if last_status_time.elapsed() > Duration::from_secs(2) {
            println!(
                "\n--- Status ({:.0}s) ---",
                start_time.elapsed().as_secs_f32()
            );
            println!("  State: {:?}", flight_state);
            print_telemetry(controller.state());
            if let Some(wp) = mission.current_waypoint() {
                let dist = controller.distance_to(wp[0], wp[1], wp[2]);
                println!(
                    "  Target WP{}: ({:.1}, {:.1}, {:.1}) dist={:.1}m",
                    mission.current_index, wp[0], wp[1], wp[2], dist
                );
            }
            last_status_time = Instant::now();
        }

        // State machine
        match flight_state {
            FlightState::PreArm => {
                // Wait for valid telemetry
                if controller.state().last_heartbeat_ms > 0 {
                    println!("\n[3/7] Telemetry received. Arming...");
                    flight_state = FlightState::Arming;
                }
            }

            FlightState::Arming => {
                if controller.is_armed() {
                    println!("      Already armed!");
                    flight_state = FlightState::Armed;
                } else {
                    match controller.arm() {
                        Ok(_) => {
                            println!("      Arm command sent");
                            // Wait a bit for arm to complete
                            std::thread::sleep(Duration::from_millis(500));
                            if controller.is_armed() {
                                flight_state = FlightState::Armed;
                            }
                        }
                        Err(ControllerError::AlreadyArmed) => {
                            flight_state = FlightState::Armed;
                        }
                        Err(e) => {
                            eprintln!("      Arm failed: {}", e);
                            std::thread::sleep(Duration::from_secs(1));
                        }
                    }
                }
            }

            FlightState::Armed => {
                println!("\n[4/7] Taking off to {}m...", takeoff_altitude);
                match controller.takeoff(takeoff_altitude) {
                    Ok(_) => {
                        println!("      Takeoff command sent");
                        flight_state = FlightState::TakingOff;
                    }
                    Err(e) => {
                        eprintln!("      Takeoff failed: {}", e);
                        flight_state = FlightState::Error;
                    }
                }
            }

            FlightState::TakingOff => {
                // Wait until we reach target altitude
                let current_alt = controller.altitude();
                if current_alt > takeoff_altitude * 0.9 {
                    println!("\n[5/7] Takeoff complete! Starting mission...");
                    println!("      Mission: {} waypoints", mission.waypoints.len());
                    flight_state = FlightState::Flying;
                }

                // Keep sending position commands during takeoff
                let pos = controller.position();
                let _ = controller.goto_position(pos[0], pos[1], -takeoff_altitude);
            }

            FlightState::Flying => {
                if let Some(wp) = mission.current_waypoint() {
                    // Navigate to current waypoint
                    let _ = controller.goto_position(wp[0], wp[1], wp[2]);

                    // Check if reached
                    if mission.reached_waypoint(controller.position()) {
                        waypoints_reached += 1;
                        println!(
                            "\n      Reached WP{} ({}/{})",
                            mission.current_index,
                            waypoints_reached,
                            mission.waypoints.len()
                        );

                        if mission.advance() {
                            if let Some(next_wp) = mission.current_waypoint() {
                                println!(
                                    "      Next: WP{} ({:.1}, {:.1}, {:.1})",
                                    mission.current_index, next_wp[0], next_wp[1], next_wp[2]
                                );
                            }
                        } else {
                            println!("\n[6/7] Mission complete! Returning home...");
                            flight_state = FlightState::ReturningHome;
                        }
                    }
                }
            }

            FlightState::ReturningHome => {
                // Go back to origin
                let _ = controller.goto_position(0.0, 0.0, -takeoff_altitude);

                let dist_home = controller.distance_2d_to(0.0, 0.0);
                if dist_home < 2.0 {
                    println!("\n[7/7] Home reached. Landing...");
                    match controller.land() {
                        Ok(_) => {
                            flight_state = FlightState::Landing;
                        }
                        Err(e) => {
                            eprintln!("      Land command failed: {}", e);
                        }
                    }
                }
            }

            FlightState::Landing => {
                // Wait for landing to complete
                let alt = controller.altitude();
                if alt < 0.3 {
                    println!("      Landed!");
                    flight_state = FlightState::Landed;
                }
            }

            FlightState::Landed => {
                println!("\n      Disarming...");
                let _ = controller.disarm();
                flight_state = FlightState::Disarmed;
            }

            FlightState::Disarmed => {
                break;
            }

            FlightState::Hovering => {
                let _ = controller.hover();
            }

            FlightState::Error => {
                println!("\n[ERROR] Flight error occurred. Landing...");
                let _ = controller.land();
                flight_state = FlightState::Landing;
            }

            _ => {}
        }

        // Small delay to prevent CPU spinning
        std::thread::sleep(Duration::from_millis(50));
    }

    // Summary
    let elapsed = start_time.elapsed();
    println!("\n=== Flight Summary ===");
    println!("Total flight time: {:.1}s", elapsed.as_secs_f32());
    println!(
        "Waypoints reached: {}/{}",
        waypoints_reached,
        mission.waypoints.len()
    );
    println!("Final state: {:?}", flight_state);
    println!("\n[OK] Flight control demo completed!");
}
