//! Multi-Drone SITL Coordinator
//!
//! This module provides coordination for multiple drone SITL instances,
//! enabling synchronized swarm operations with formation flying,
//! collision avoidance, and coordinated missions.
//!
//! # Features
//! - Multi-instance MAVLink connection management
//! - Formation control (V-formation, circle, line, grid)
//! - Collision avoidance using potential fields
//! - Synchronized takeoff, landing, and RTL
//! - Waypoint mission coordination
//!
//! # Example
//! ```ignore
//! use drone_swarm_system::multi_drone_coordinator::SwarmCoordinator;
//!
//! let mut coordinator = SwarmCoordinator::new();
//! coordinator.add_drone("udpin:0.0.0.0:14540", 1)?;
//! coordinator.add_drone("udpin:0.0.0.0:14541", 2)?;
//! coordinator.takeoff_all(10.0)?;
//! coordinator.set_formation(SwarmFormation::VFormation { spacing: 10.0 });
//! ```

#[cfg(feature = "simulation")]
use crate::mavlink_controller::{ControllerError, FlightController};
use core::f32::consts::PI;
#[cfg(feature = "simulation")]
use std::time::{Duration, Instant};

/// Maximum number of drones in a coordinated swarm
pub const MAX_COORDINATED_DRONES: usize = 10;

/// Collision avoidance parameters
pub const COLLISION_AVOIDANCE_RADIUS: f32 = 5.0; // meters
pub const COLLISION_AVOIDANCE_STRENGTH: f32 = 3.0;

/// Swarm formation patterns
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SwarmFormation {
    /// V-formation (like migrating birds)
    VFormation { spacing: f32 },
    /// Circular formation around center
    Circle { radius: f32 },
    /// Line formation along X axis
    Line { spacing: f32 },
    /// Grid formation
    Grid { spacing: f32 },
    /// Custom positions (set manually)
    Custom,
}

/// Coordinated swarm state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmState {
    /// Initializing connections
    Initializing,
    /// All drones connected and ready
    Ready,
    /// Armed and ready for takeoff
    Armed,
    /// Taking off
    TakingOff,
    /// In formation flight
    FormationFlight,
    /// Executing waypoint mission
    Mission,
    /// Returning to launch
    ReturningHome,
    /// Landing
    Landing,
    /// Landed and disarmed
    Landed,
    /// Emergency stop engaged
    Emergency,
}

/// Drone state within the swarm
#[cfg(feature = "simulation")]
pub struct DroneInstance {
    /// MAVLink flight controller
    pub controller: FlightController,
    /// Drone ID within swarm (0-indexed)
    pub swarm_id: usize,
    /// Target formation position
    pub formation_target: [f32; 3],
    /// Home position (takeoff location)
    pub home_position: [f32; 3],
    /// Is drone operational
    pub operational: bool,
    /// Connection address
    pub address: String,
}

#[cfg(feature = "simulation")]
impl DroneInstance {
    fn new(controller: FlightController, swarm_id: usize, address: &str) -> Self {
        Self {
            controller,
            swarm_id,
            formation_target: [0.0, 0.0, 0.0],
            home_position: [0.0, 0.0, 0.0],
            operational: true,
            address: address.to_string(),
        }
    }
}

/// Multi-drone swarm coordinator
#[cfg(feature = "simulation")]
pub struct SwarmCoordinator {
    /// Connected drone instances
    drones: Vec<DroneInstance>,
    /// Current swarm state
    state: SwarmState,
    /// Current formation pattern
    formation: SwarmFormation,
    /// Formation center position [x, y, z] (NED frame)
    center: [f32; 3],
    /// Formation heading (yaw) in radians
    heading: f32,
    /// Target altitude for formation flight (negative in NED)
    altitude: f32,
    /// Enable collision avoidance
    collision_avoidance_enabled: bool,
    /// Start time for timing operations
    #[allow(dead_code)]
    start_time: Instant,
}

#[cfg(feature = "simulation")]
impl SwarmCoordinator {
    /// Create a new swarm coordinator
    pub fn new() -> Self {
        Self {
            drones: Vec::new(),
            state: SwarmState::Initializing,
            formation: SwarmFormation::Line { spacing: 10.0 },
            center: [0.0, 0.0, -10.0],
            heading: 0.0,
            altitude: -10.0,
            collision_avoidance_enabled: true,
            start_time: Instant::now(),
        }
    }

    /// Add a drone to the swarm
    pub fn add_drone(&mut self, address: &str, swarm_id: usize) -> Result<(), ControllerError> {
        if self.drones.len() >= MAX_COORDINATED_DRONES {
            return Err(ControllerError::ConnectionFailed(
                "Maximum drones reached".to_string(),
            ));
        }

        println!(
            "[SWARM] Connecting to drone {} at {}...",
            swarm_id, address
        );

        let controller = FlightController::connect(address)?;
        let drone = DroneInstance::new(controller, swarm_id, address);
        self.drones.push(drone);

        println!("[SWARM] Drone {} connected successfully!", swarm_id);
        Ok(())
    }

    /// Get number of drones in swarm
    pub fn drone_count(&self) -> usize {
        self.drones.len()
    }

    /// Get current swarm state
    pub fn state(&self) -> SwarmState {
        self.state
    }

    /// Set formation pattern
    pub fn set_formation(&mut self, formation: SwarmFormation) {
        self.formation = formation;
        self.update_formation_targets();
        println!("[SWARM] Formation set to {:?}", formation);
    }

    /// Set formation center position
    pub fn set_center(&mut self, x: f32, y: f32, z: f32) {
        self.center = [x, y, z];
        self.update_formation_targets();
    }

    /// Set formation heading (yaw) in degrees
    pub fn set_heading_degrees(&mut self, heading_deg: f32) {
        self.heading = heading_deg.to_radians();
        self.update_formation_targets();
    }

    /// Set target altitude (positive, will be converted to NED)
    pub fn set_altitude(&mut self, altitude: f32) {
        self.altitude = -altitude.abs(); // NED frame (negative = up)
        self.update_formation_targets();
    }

    /// Enable/disable collision avoidance
    pub fn set_collision_avoidance(&mut self, enabled: bool) {
        self.collision_avoidance_enabled = enabled;
    }

    /// Calculate formation positions for all drones
    fn update_formation_targets(&mut self) {
        let n = self.drones.len();
        if n == 0 {
            return;
        }

        let positions = self.calculate_formation_positions(n);

        for (i, drone) in self.drones.iter_mut().enumerate() {
            if i < positions.len() {
                drone.formation_target = positions[i];
            }
        }
    }

    /// Calculate formation positions based on pattern
    fn calculate_formation_positions(&self, num_drones: usize) -> Vec<[f32; 3]> {
        let mut positions = Vec::new();
        let cos_h = self.heading.cos();
        let sin_h = self.heading.sin();

        match self.formation {
            SwarmFormation::VFormation { spacing } => {
                // V-formation: leader at front, others trailing at 30 degree angle
                let angle = PI / 6.0; // 30 degrees

                for i in 0..num_drones {
                    let (local_x, local_y) = if i == 0 {
                        (0.0, 0.0) // Leader at center
                    } else {
                        let side = if i % 2 == 1 { 1.0 } else { -1.0 };
                        let row = (i.div_ceil(2)) as f32;
                        let x = -row * spacing * angle.cos(); // Behind leader
                        let y = side * row * spacing * angle.sin(); // Left or right
                        (x, y)
                    };

                    // Rotate by formation heading
                    let world_x = self.center[0] + local_x * cos_h - local_y * sin_h;
                    let world_y = self.center[1] + local_x * sin_h + local_y * cos_h;

                    positions.push([world_x, world_y, self.altitude]);
                }
            }

            SwarmFormation::Circle { radius } => {
                for i in 0..num_drones {
                    let angle = self.heading + 2.0 * PI * (i as f32) / (num_drones as f32);
                    let x = self.center[0] + radius * angle.cos();
                    let y = self.center[1] + radius * angle.sin();
                    positions.push([x, y, self.altitude]);
                }
            }

            SwarmFormation::Line { spacing } => {
                let total_width = spacing * (num_drones - 1) as f32;
                let start_offset = -total_width / 2.0;

                for i in 0..num_drones {
                    let local_y = start_offset + spacing * (i as f32);
                    let world_x = self.center[0] - local_y * sin_h;
                    let world_y = self.center[1] + local_y * cos_h;
                    positions.push([world_x, world_y, self.altitude]);
                }
            }

            SwarmFormation::Grid { spacing } => {
                let cols = ((num_drones as f32).sqrt().ceil()) as usize;
                let rows = num_drones.div_ceil(cols);

                let x_offset = -spacing * (cols - 1) as f32 / 2.0;
                let y_offset = -spacing * (rows - 1) as f32 / 2.0;

                for i in 0..num_drones {
                    let row = i / cols;
                    let col = i % cols;

                    let local_x = x_offset + spacing * (col as f32);
                    let local_y = y_offset + spacing * (row as f32);

                    let world_x = self.center[0] + local_x * cos_h - local_y * sin_h;
                    let world_y = self.center[1] + local_x * sin_h + local_y * cos_h;

                    positions.push([world_x, world_y, self.altitude]);
                }
            }

            SwarmFormation::Custom => {
                // Use existing targets
                for drone in &self.drones {
                    positions.push(drone.formation_target);
                }
            }
        }

        positions
    }

    /// Update all drone controllers (receive telemetry)
    pub fn update(&mut self) -> Result<(), ControllerError> {
        for drone in &mut self.drones {
            if drone.operational {
                if let Err(e) = drone.controller.update() {
                    println!(
                        "[SWARM] Warning: Drone {} update failed: {}",
                        drone.swarm_id, e
                    );
                }
            }
        }
        Ok(())
    }

    /// Arm all drones
    pub fn arm_all(&mut self) -> Result<(), ControllerError> {
        println!("[SWARM] Arming {} drones...", self.drones.len());

        for drone in &mut self.drones {
            if drone.operational && !drone.controller.is_armed() {
                match drone.controller.arm() {
                    Ok(_) => println!("  Drone {} armed", drone.swarm_id),
                    Err(ControllerError::AlreadyArmed) => {
                        println!("  Drone {} already armed", drone.swarm_id)
                    }
                    Err(e) => {
                        println!("  Drone {} arm failed: {}", drone.swarm_id, e);
                        return Err(e);
                    }
                }
            }
        }

        self.state = SwarmState::Armed;
        Ok(())
    }

    /// Disarm all drones
    pub fn disarm_all(&mut self) -> Result<(), ControllerError> {
        println!("[SWARM] Disarming {} drones...", self.drones.len());

        for drone in &mut self.drones {
            if drone.operational {
                let _ = drone.controller.disarm();
            }
        }

        self.state = SwarmState::Landed;
        Ok(())
    }

    /// Synchronized takeoff for all drones
    pub fn takeoff_all(&mut self, altitude: f32) -> Result<(), ControllerError> {
        self.altitude = -altitude.abs();
        println!(
            "[SWARM] Initiating synchronized takeoff to {}m...",
            altitude
        );

        // Store home positions
        for drone in &mut self.drones {
            drone.home_position = drone.controller.position();
        }

        // Send takeoff commands
        for drone in &mut self.drones {
            if drone.operational {
                match drone.controller.takeoff(altitude) {
                    Ok(_) => println!("  Drone {} takeoff initiated", drone.swarm_id),
                    Err(e) => {
                        println!("  Drone {} takeoff failed: {}", drone.swarm_id, e);
                    }
                }
            }
        }

        self.state = SwarmState::TakingOff;
        self.update_formation_targets();
        Ok(())
    }

    /// Land all drones
    pub fn land_all(&mut self) -> Result<(), ControllerError> {
        println!("[SWARM] Initiating synchronized landing...");

        for drone in &mut self.drones {
            if drone.operational {
                let _ = drone.controller.land();
            }
        }

        self.state = SwarmState::Landing;
        Ok(())
    }

    /// Return all drones to launch
    pub fn rtl_all(&mut self) -> Result<(), ControllerError> {
        println!("[SWARM] Initiating Return-To-Launch for all drones...");

        for drone in &mut self.drones {
            if drone.operational {
                let _ = drone.controller.return_to_launch();
            }
        }

        self.state = SwarmState::ReturningHome;
        Ok(())
    }

    /// Check if all drones have reached target altitude
    pub fn all_at_altitude(&self, tolerance: f32) -> bool {
        for drone in &self.drones {
            if drone.operational {
                let alt = drone.controller.altitude();
                let target_alt = (-self.altitude).abs();
                if (alt - target_alt).abs() > tolerance {
                    return false;
                }
            }
        }
        true
    }

    /// Check if all drones are in formation
    pub fn all_in_formation(&self, tolerance: f32) -> bool {
        for drone in &self.drones {
            if drone.operational {
                let pos = drone.controller.position();
                let target = drone.formation_target;
                let dist = distance_3d(&pos, &target);
                if dist > tolerance {
                    return false;
                }
            }
        }
        true
    }

    /// Calculate collision avoidance velocity adjustment
    fn calculate_avoidance_velocity(&self, drone_index: usize) -> [f32; 3] {
        if !self.collision_avoidance_enabled {
            return [0.0, 0.0, 0.0];
        }

        let drone_pos = self.drones[drone_index].controller.position();
        let mut avoidance = [0.0f32; 3];

        for (i, other) in self.drones.iter().enumerate() {
            if i != drone_index && other.operational {
                let other_pos = other.controller.position();
                let dist = distance_3d(&drone_pos, &other_pos);

                if dist < COLLISION_AVOIDANCE_RADIUS && dist > 0.1 {
                    // Repulsion force (inverse square law)
                    let force = COLLISION_AVOIDANCE_STRENGTH / (dist * dist);

                    // Direction away from other drone
                    avoidance[0] += (drone_pos[0] - other_pos[0]) / dist * force;
                    avoidance[1] += (drone_pos[1] - other_pos[1]) / dist * force;
                    avoidance[2] += (drone_pos[2] - other_pos[2]) / dist * force;
                }
            }
        }

        avoidance
    }

    /// Execute one step of formation control
    pub fn formation_step(&mut self) -> Result<(), ControllerError> {
        self.update()?;

        // Send position commands to each drone
        for i in 0..self.drones.len() {
            if !self.drones[i].operational {
                continue;
            }

            let target = self.drones[i].formation_target;
            let avoidance = self.calculate_avoidance_velocity(i);

            // Combine target with avoidance
            let adjusted_target = [
                target[0] + avoidance[0],
                target[1] + avoidance[1],
                target[2] + avoidance[2],
            ];

            let _ = self.drones[i]
                .controller
                .goto_position(adjusted_target[0], adjusted_target[1], adjusted_target[2]);
        }

        Ok(())
    }

    /// Move formation center to new position
    pub fn move_formation_to(&mut self, x: f32, y: f32) {
        self.center[0] = x;
        self.center[1] = y;
        self.update_formation_targets();
    }

    /// Get swarm metrics
    pub fn get_metrics(&self) -> SwarmMetrics {
        let mut center = [0.0f32; 3];
        let mut active_count = 0;

        // Calculate center of mass
        for drone in &self.drones {
            if drone.operational {
                let pos = drone.controller.position();
                center[0] += pos[0];
                center[1] += pos[1];
                center[2] += pos[2];
                active_count += 1;
            }
        }

        if active_count > 0 {
            center[0] /= active_count as f32;
            center[1] /= active_count as f32;
            center[2] /= active_count as f32;
        }

        // Calculate spread and min separation
        let mut max_spread = 0.0f32;
        let mut min_separation = f32::INFINITY;
        let mut total_formation_error = 0.0f32;

        for (i, drone) in self.drones.iter().enumerate() {
            if !drone.operational {
                continue;
            }

            let pos = drone.controller.position();
            let spread = distance_3d(&pos, &center);
            max_spread = max_spread.max(spread);

            // Formation error
            let error = distance_3d(&pos, &drone.formation_target);
            total_formation_error += error;

            // Min separation
            for (j, other) in self.drones.iter().enumerate() {
                if i != j && other.operational {
                    let other_pos = other.controller.position();
                    let sep = distance_3d(&pos, &other_pos);
                    min_separation = min_separation.min(sep);
                }
            }
        }

        SwarmMetrics {
            center,
            spread: max_spread,
            min_separation: if min_separation == f32::INFINITY {
                0.0
            } else {
                min_separation
            },
            formation_error: if active_count > 0 {
                total_formation_error / active_count as f32
            } else {
                0.0
            },
            active_drones: active_count,
            total_drones: self.drones.len(),
        }
    }

    /// Print status of all drones
    pub fn print_status(&self) {
        println!("\n--- Swarm Status ({:?}) ---", self.state);
        let metrics = self.get_metrics();
        println!(
            "Center: ({:.1}, {:.1}, {:.1}) | Spread: {:.1}m | Formation Error: {:.2}m",
            metrics.center[0], metrics.center[1], metrics.center[2], metrics.spread, metrics.formation_error
        );
        println!(
            "Active: {}/{} | Min Separation: {:.1}m",
            metrics.active_drones, metrics.total_drones, metrics.min_separation
        );

        for drone in &self.drones {
            let pos = drone.controller.position();
            let alt = drone.controller.altitude();
            let armed = drone.controller.is_armed();
            let target = drone.formation_target;
            let dist = distance_3d(&pos, &target);

            println!(
                "  Drone {}: pos=({:6.1}, {:6.1}, {:5.1}m) armed={} dist_to_target={:.1}m",
                drone.swarm_id,
                pos[0],
                pos[1],
                alt,
                if armed { "Y" } else { "N" },
                dist
            );
        }
    }

    /// Get reference to drone by swarm ID
    pub fn get_drone(&self, swarm_id: usize) -> Option<&DroneInstance> {
        self.drones.iter().find(|d| d.swarm_id == swarm_id)
    }

    /// Get mutable reference to drone by swarm ID
    pub fn get_drone_mut(&mut self, swarm_id: usize) -> Option<&mut DroneInstance> {
        self.drones.iter_mut().find(|d| d.swarm_id == swarm_id)
    }

    /// Wait for all drones to reach altitude
    pub fn wait_for_altitude(&mut self, tolerance: f32, timeout: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < timeout {
            let _ = self.update();
            if self.all_at_altitude(tolerance) {
                self.state = SwarmState::FormationFlight;
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }

    /// Wait for all drones to reach formation
    pub fn wait_for_formation(&mut self, tolerance: f32, timeout: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < timeout {
            let _ = self.formation_step();
            if self.all_in_formation(tolerance) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }
}

#[cfg(feature = "simulation")]
impl Default for SwarmCoordinator {
    fn default() -> Self {
        Self::new()
    }
}

/// Swarm metrics for monitoring
#[derive(Debug, Clone)]
pub struct SwarmMetrics {
    /// Swarm center of mass
    pub center: [f32; 3],
    /// Maximum distance from center (spread)
    pub spread: f32,
    /// Minimum separation between drones
    pub min_separation: f32,
    /// Average distance from formation target
    pub formation_error: f32,
    /// Number of active (operational) drones
    pub active_drones: usize,
    /// Total drones in swarm
    pub total_drones: usize,
}

/// Calculate 3D Euclidean distance
fn distance_3d(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance_3d() {
        assert!((distance_3d(&[0.0, 0.0, 0.0], &[3.0, 4.0, 0.0]) - 5.0).abs() < 0.001);
        assert!((distance_3d(&[0.0, 0.0, 0.0], &[1.0, 1.0, 1.0]) - 1.732).abs() < 0.01);
    }

    #[test]
    fn test_formation_positions() {
        // Test that formation position calculation doesn't panic
        #[cfg(feature = "simulation")]
        {
            let coordinator = SwarmCoordinator::new();
            let positions = coordinator.calculate_formation_positions(5);
            assert_eq!(positions.len(), 5);
        }
    }
}
