//! Swarm SITL Example
//!
//! Demonstrates connecting to multiple PX4 SITL drones for swarm simulation
//!
//! # Running
//! 1. Start swarm: `./simulation/start_sitl.sh swarm 3`
//! 2. Run this example: `cargo run --example swarm_sitl --features "simulation tokio"`

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use mavlink::common::MavMessage;
use tokio::sync::Mutex;
use tokio::time::interval;

/// Drone state from SITL
#[derive(Debug, Clone, Default)]
struct DroneState {
    id: u8,
    connected: bool,
    position: [f32; 3], // x, y, z in meters (NED frame)
    velocity: [f32; 3], // vx, vy, vz in m/s
    attitude: [f32; 3], // roll, pitch, yaw in radians
    battery_percent: i8,
    armed: bool,
    mode: String,
}

/// Swarm controller managing multiple SITL drones
struct SwarmController {
    drones: Arc<Mutex<HashMap<u8, DroneState>>>,
    num_drones: usize,
}

impl SwarmController {
    fn new(num_drones: usize) -> Self {
        Self {
            drones: Arc::new(Mutex::new(HashMap::new())),
            num_drones,
        }
    }

    /// Connect to all drones in the swarm
    async fn connect_all(&self) -> Vec<tokio::task::JoinHandle<()>> {
        let mut handles = Vec::new();

        for i in 0..self.num_drones {
            let port = 14540 + i;
            let drone_id = i as u8;
            let drones = self.drones.clone();

            let handle = tokio::spawn(async move {
                Self::drone_connection_task(drone_id, port, drones).await;
            });

            handles.push(handle);
        }

        handles
    }

    /// Task to handle connection to a single drone
    async fn drone_connection_task(
        drone_id: u8,
        port: usize,
        drones: Arc<Mutex<HashMap<u8, DroneState>>>,
    ) {
        let address = format!("udpin:0.0.0.0:{}", port);
        println!("[Drone {}] Connecting to {}...", drone_id, address);

        let connection = match mavlink::connect::<MavMessage>(&address) {
            Ok(conn) => {
                println!("[Drone {}] Connected!", drone_id);
                conn
            }
            Err(e) => {
                eprintln!("[Drone {}] Connection failed: {}", drone_id, e);
                return;
            }
        };

        // Initialize drone state
        {
            let mut drones_lock = drones.lock().await;
            drones_lock.insert(
                drone_id,
                DroneState {
                    id: drone_id,
                    connected: true,
                    ..Default::default()
                },
            );
        }

        // Receive messages
        loop {
            match connection.recv() {
                Ok((_header, msg)) => {
                    Self::process_message(drone_id, &msg, &drones).await;
                }
                Err(e) => {
                    eprintln!("[Drone {}] Receive error: {}", drone_id, e);
                    break;
                }
            }
        }

        // Mark as disconnected
        let mut drones_lock = drones.lock().await;
        if let Some(drone) = drones_lock.get_mut(&drone_id) {
            drone.connected = false;
        }
    }

    /// Process a MAVLink message and update drone state
    async fn process_message(
        drone_id: u8,
        msg: &MavMessage,
        drones: &Arc<Mutex<HashMap<u8, DroneState>>>,
    ) {
        let mut drones_lock = drones.lock().await;
        let drone = drones_lock.entry(drone_id).or_default();

        match msg {
            MavMessage::HEARTBEAT(hb) => {
                drone.armed = hb.base_mode.bits() & 128 != 0;
                drone.mode = format!("{:?}", hb.custom_mode);
            }
            MavMessage::LOCAL_POSITION_NED(pos) => {
                drone.position = [pos.x, pos.y, pos.z];
                drone.velocity = [pos.vx, pos.vy, pos.vz];
            }
            MavMessage::ATTITUDE(att) => {
                drone.attitude = [att.roll, att.pitch, att.yaw];
            }
            MavMessage::BATTERY_STATUS(bat) => {
                drone.battery_percent = bat.battery_remaining;
            }
            _ => {}
        }
    }

    /// Get current state of all drones
    async fn get_swarm_state(&self) -> Vec<DroneState> {
        let drones = self.drones.lock().await;
        drones.values().cloned().collect()
    }

    /// Calculate swarm center of mass
    async fn get_swarm_center(&self) -> [f32; 3] {
        let drones = self.drones.lock().await;
        if drones.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let mut center = [0.0f32; 3];
        let count = drones.len() as f32;

        for drone in drones.values() {
            center[0] += drone.position[0];
            center[1] += drone.position[1];
            center[2] += drone.position[2];
        }

        center[0] /= count;
        center[1] /= count;
        center[2] /= count;

        center
    }

    /// Calculate swarm spread (max distance from center)
    async fn get_swarm_spread(&self) -> f32 {
        let center = self.get_swarm_center().await;
        let drones = self.drones.lock().await;

        let mut max_dist = 0.0f32;

        for drone in drones.values() {
            let dx = drone.position[0] - center[0];
            let dy = drone.position[1] - center[1];
            let dz = drone.position[2] - center[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            max_dist = max_dist.max(dist);
        }

        max_dist
    }
}

#[tokio::main]
async fn main() {
    println!("=== Swarm SITL Connection Example ===\n");

    let num_drones = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(3);

    println!("Attempting to connect to {} drones", num_drones);
    println!(
        "Make sure swarm SITL is running: ./simulation/start_sitl.sh swarm {}\n",
        num_drones
    );

    let controller = SwarmController::new(num_drones);

    // Start connection tasks
    let handles = controller.connect_all().await;

    // Wait for connections
    tokio::time::sleep(Duration::from_secs(2)).await;

    // Status reporting loop
    let mut interval = interval(Duration::from_secs(1));

    for _ in 0..30 {
        interval.tick().await;

        let state = controller.get_swarm_state().await;
        let connected = state.iter().filter(|d| d.connected).count();

        println!(
            "\n=== Swarm Status ({}/{} connected) ===",
            connected, num_drones
        );

        if connected > 0 {
            let center = controller.get_swarm_center().await;
            let spread = controller.get_swarm_spread().await;

            println!(
                "Swarm center: ({:.2}, {:.2}, {:.2})",
                center[0], center[1], center[2]
            );
            println!("Swarm spread: {:.2}m", spread);

            for drone in &state {
                if drone.connected {
                    println!(
                        "  Drone {}: pos=({:.1}, {:.1}, {:.1}) armed={} bat={}%",
                        drone.id,
                        drone.position[0],
                        drone.position[1],
                        drone.position[2],
                        if drone.armed { "Y" } else { "N" },
                        drone.battery_percent
                    );
                }
            }
        } else {
            println!("No drones connected. Waiting...");
        }
    }

    println!("\n[OK] Swarm monitoring completed!");

    // Cleanup
    for handle in handles {
        handle.abort();
    }
}
