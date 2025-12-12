//! SITL Connection Example
//!
//! Demonstrates connecting to PX4 SITL via MAVLink
//!
//! # Running
//! 1. Start PX4 SITL: `./simulation/start_sitl.sh single`
//! 2. Run this example: `cargo run --example sitl_connection --features simulation`

use std::time::{Duration, Instant};

use mavlink::common::MavMessage;

const SITL_ADDRESS: &str = "udpin:0.0.0.0:14540";

fn main() {
    println!("=== PX4 SITL Connection Example ===\n");
    println!("Connecting to PX4 SITL at {}", SITL_ADDRESS);
    println!("Make sure SITL is running: ./simulation/start_sitl.sh single\n");

    // Connect to SITL
    let connection = match mavlink::connect::<MavMessage>(SITL_ADDRESS) {
        Ok(conn) => {
            println!("[OK] Connected to SITL!");
            conn
        }
        Err(e) => {
            eprintln!("[ERROR] Failed to connect: {}", e);
            eprintln!("\nTroubleshooting:");
            eprintln!("  1. Is PX4 SITL running?");
            eprintln!("  2. Run: ./simulation/start_sitl.sh single");
            eprintln!("  3. Wait for 'Ready for takeoff' message");
            return;
        }
    };

    println!("\nListening for MAVLink messages...\n");

    let start_time = Instant::now();
    let mut heartbeat_count = 0;
    let mut attitude_count = 0;
    let mut position_count = 0;

    // Receive messages for 10 seconds
    while start_time.elapsed() < Duration::from_secs(10) {
        match connection.recv() {
            Ok((_header, msg)) => {
                match msg {
                    MavMessage::HEARTBEAT(hb) => {
                        heartbeat_count += 1;
                        if heartbeat_count == 1 {
                            println!(
                                "[HEARTBEAT] System type: {:?}, Autopilot: {:?}",
                                hb.mavtype, hb.autopilot
                            );
                        }
                    }
                    MavMessage::ATTITUDE(att) => {
                        attitude_count += 1;
                        if attitude_count % 50 == 1 {
                            println!(
                                "[ATTITUDE] Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°",
                                att.roll.to_degrees(),
                                att.pitch.to_degrees(),
                                att.yaw.to_degrees()
                            );
                        }
                    }
                    MavMessage::LOCAL_POSITION_NED(pos) => {
                        position_count += 1;
                        if position_count % 50 == 1 {
                            println!(
                                "[POSITION] X: {:.2}m, Y: {:.2}m, Z: {:.2}m",
                                pos.x, pos.y, pos.z
                            );
                        }
                    }
                    MavMessage::GPS_RAW_INT(gps) => {
                        println!(
                            "[GPS] Lat: {:.6}°, Lon: {:.6}°, Alt: {}m, Fix: {:?}",
                            gps.lat as f64 / 1e7,
                            gps.lon as f64 / 1e7,
                            gps.alt / 1000,
                            gps.fix_type
                        );
                    }
                    MavMessage::BATTERY_STATUS(bat) => {
                        if bat.battery_remaining >= 0 {
                            println!("[BATTERY] {}% remaining", bat.battery_remaining);
                        }
                    }
                    MavMessage::STATUSTEXT(text) => {
                        let msg_text: String = text
                            .text
                            .iter()
                            .take_while(|&&c| c != 0)
                            .map(|&c| c as char)
                            .collect();
                        println!("[STATUS] {}", msg_text);
                    }
                    _ => {
                        // Ignore other messages
                    }
                }
            }
            Err(e) => {
                eprintln!("[ERROR] Receive error: {}", e);
                break;
            }
        }
    }

    println!("\n=== Summary ===");
    println!("Heartbeats received: {}", heartbeat_count);
    println!("Attitude updates: {}", attitude_count);
    println!("Position updates: {}", position_count);
    println!("\n[OK] Connection test completed!");
}
