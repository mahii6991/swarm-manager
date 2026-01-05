//! Collision Avoidance System Demo
//!
//! This example demonstrates the multi-algorithm collision avoidance system:
//! - Velocity Obstacles (VO)
//! - Reciprocal Velocity Obstacles (RVO)
//! - Optimal Reciprocal Collision Avoidance (ORCA)
//! - Artificial Potential Fields (APF)
//!
//! # Running
//! ```bash
//! cargo run --example collision_avoidance_demo --features std
//! ```

use std::time::Instant;

use drone_swarm_system::control::collision::{AvoidanceConfig, AvoidanceAlgorithm, CollisionAvoidance};

/// Number of drones in simulation
const NUM_DRONES: usize = 8;

/// Simulation time step (seconds)
const DT: f32 = 0.1;

/// Total simulation time (seconds)
const SIMULATION_TIME: f32 = 30.0;

/// Drone state for simulation
struct DroneState {
    id: usize,
    position: [f32; 3],
    velocity: [f32; 3],
    target: [f32; 3],
    radius: f32,
}

impl DroneState {
    fn new(id: usize, position: [f32; 3], target: [f32; 3]) -> Self {
        Self {
            id,
            position,
            velocity: [0.0, 0.0, 0.0],
            target,
            radius: 0.5,
        }
    }

    fn distance_to_target(&self) -> f32 {
        let dx = self.target[0] - self.position[0];
        let dy = self.target[1] - self.position[1];
        let dz = self.target[2] - self.position[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn compute_desired_velocity(&self, max_speed: f32) -> [f32; 3] {
        let dx = self.target[0] - self.position[0];
        let dy = self.target[1] - self.position[1];
        let dz = self.target[2] - self.position[2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        if dist < 0.1 {
            return [0.0, 0.0, 0.0];
        }

        let speed = max_speed.min(dist * 2.0);
        let scale = speed / dist;

        [dx * scale, dy * scale, dz * scale]
    }

    fn update_position(&mut self, dt: f32) {
        self.position[0] += self.velocity[0] * dt;
        self.position[1] += self.velocity[1] * dt;
        self.position[2] += self.velocity[2] * dt;
    }
}

fn check_collisions(drones: &[DroneState], min_separation: f32) -> Vec<(usize, usize, f32)> {
    let mut collisions = Vec::new();

    for i in 0..drones.len() {
        for j in (i + 1)..drones.len() {
            let dx = drones[i].position[0] - drones[j].position[0];
            let dy = drones[i].position[1] - drones[j].position[1];
            let dz = drones[i].position[2] - drones[j].position[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();

            if dist < min_separation {
                collisions.push((i, j, dist));
            }
        }
    }

    collisions
}

fn run_simulation(algorithm: AvoidanceAlgorithm) -> (usize, f32, usize) {
    let config = AvoidanceConfig {
        algorithm,
        min_separation: 3.0,
        safety_margin: 1.0,
        max_velocity: 5.0,
        time_horizon: 3.0,
        responsiveness: 0.8,
        ..AvoidanceConfig::default()
    };

    let mut avoidance = CollisionAvoidance::new(config.clone());

    // Initialize drones in crossing pattern
    let mut drones = Vec::new();
    for i in 0..NUM_DRONES {
        let angle = 2.0 * std::f32::consts::PI * (i as f32) / (NUM_DRONES as f32);
        let radius = 20.0;

        let start = [
            radius * angle.cos(),
            radius * angle.sin(),
            -10.0,
        ];

        let target = [
            -start[0],
            -start[1],
            -10.0,
        ];

        drones.push(DroneState::new(i, start, target));
    }

    let mut collision_count = 0;
    let mut total_path_length = 0.0f32;
    let steps = (SIMULATION_TIME / DT) as usize;
    let mut reached_target = vec![false; NUM_DRONES];

    for _step in 0..steps {
        // Compute safe velocities for each drone
        for i in 0..drones.len() {
            if reached_target[i] {
                drones[i].velocity = [0.0, 0.0, 0.0];
                continue;
            }

            // Clear and add other drones as obstacles
            avoidance.clear_obstacles();
            for (j, drone) in drones.iter().enumerate() {
                if i != j {
                    avoidance.add_obstacle(drone.position, drone.velocity, drone.radius);
                }
            }

            // Compute desired velocity toward target
            let desired = drones[i].compute_desired_velocity(config.max_velocity);

            // Compute safe velocity using collision avoidance
            let safe_velocity = avoidance.compute_safe_velocity(drones[i].position, desired);

            drones[i].velocity = safe_velocity;
        }

        // Update positions
        for (i, drone) in drones.iter_mut().enumerate() {
            let old_pos = drone.position;
            drone.update_position(DT);

            // Track path length
            let dx = drone.position[0] - old_pos[0];
            let dy = drone.position[1] - old_pos[1];
            let dz = drone.position[2] - old_pos[2];
            total_path_length += (dx * dx + dy * dy + dz * dz).sqrt();

            // Check if reached target
            if drone.distance_to_target() < 1.0 && !reached_target[i] {
                reached_target[i] = true;
            }
        }

        // Check for collisions
        let collisions = check_collisions(&drones, 2.0);
        collision_count += collisions.len();
    }

    let targets_reached = reached_target.iter().filter(|&&x| x).count();

    (collision_count, total_path_length, targets_reached)
}

fn main() {
    println!("=== Collision Avoidance System Demo ===\n");
    println!("Simulating {} drones crossing paths.\n", NUM_DRONES);
    println!("Scenario: Drones start at edges of a circle and fly to opposite side.");
    println!("This creates multiple potential collision points.\n");

    let start_time = Instant::now();

    // Test each algorithm
    let algorithms = [
        (AvoidanceAlgorithm::VelocityObstacle, "Velocity Obstacles (VO)"),
        (AvoidanceAlgorithm::RVO, "Reciprocal VO (RVO)"),
        (AvoidanceAlgorithm::ORCA, "Optimal RCA (ORCA)"),
        (AvoidanceAlgorithm::PotentialField, "Artificial Potential Fields"),
        (AvoidanceAlgorithm::Hybrid, "Hybrid (ORCA + APF)"),
    ];

    println!("--- Algorithm Comparison ---\n");
    println!("{:<30} {:>12} {:>15} {:>12}",
        "Algorithm", "Collisions", "Path Length", "Completed");
    println!("{}", "-".repeat(72));

    for (algorithm, name) in &algorithms {
        let (collisions, path_length, completed) = run_simulation(*algorithm);

        println!("{:<30} {:>12} {:>15.1}m {:>10}/{}",
            name, collisions, path_length, completed, NUM_DRONES);
    }

    println!("\n--- Detailed Hybrid Algorithm Analysis ---\n");

    // Run detailed simulation with Hybrid algorithm
    let config = AvoidanceConfig {
        algorithm: AvoidanceAlgorithm::Hybrid,
        min_separation: 3.0,
        safety_margin: 1.0,
        max_velocity: 5.0,
        time_horizon: 3.0,
        responsiveness: 0.8,
        ..AvoidanceConfig::default()
    };

    let mut avoidance = CollisionAvoidance::new(config.clone());

    // Create scenario
    let mut drones = Vec::new();
    for i in 0..NUM_DRONES {
        let angle = 2.0 * std::f32::consts::PI * (i as f32) / (NUM_DRONES as f32);
        let radius = 20.0;
        let start = [radius * angle.cos(), radius * angle.sin(), -10.0];
        let target = [-start[0], -start[1], -10.0];
        drones.push(DroneState::new(i, start, target));
    }

    println!("Initial positions:");
    for (i, drone) in drones.iter().enumerate() {
        println!("  Drone {}: ({:6.1}, {:6.1}, {:5.1}) -> ({:6.1}, {:6.1}, {:5.1})",
            i,
            drone.position[0], drone.position[1], drone.position[2],
            drone.target[0], drone.target[1], drone.target[2]);
    }

    // Run a few simulation steps with detailed output
    println!("\nSimulation progress (every 5 seconds):");

    let report_interval = (5.0 / DT) as usize;
    let steps = (SIMULATION_TIME / DT) as usize;

    for step in 0..steps {
        for i in 0..drones.len() {
            avoidance.clear_obstacles();
            for (j, drone) in drones.iter().enumerate() {
                if i != j {
                    avoidance.add_obstacle(drone.position, drone.velocity, drone.radius);
                }
            }

            let desired = drones[i].compute_desired_velocity(config.max_velocity);
            let safe = avoidance.compute_safe_velocity(drones[i].position, desired);
            drones[i].velocity = safe;
        }

        for drone in &mut drones {
            drone.update_position(DT);
        }

        if step % report_interval == 0 && step > 0 {
            let time = step as f32 * DT;
            let collisions = check_collisions(&drones, 2.0);
            let targets_reached: usize = drones.iter()
                .filter(|d| d.distance_to_target() < 1.0)
                .count();

            println!("  t={:5.1}s: {:>2} at target, {:>2} near-misses, avg dist: {:.1}m",
                time, targets_reached, collisions.len(),
                drones.iter().map(|d| d.distance_to_target()).sum::<f32>() / NUM_DRONES as f32);
        }
    }

    // Final positions
    println!("\nFinal positions:");
    for (i, drone) in drones.iter().enumerate() {
        let dist = drone.distance_to_target();
        let status = if dist < 1.0 { "REACHED" } else { "en route" };
        println!("  Drone {}: ({:6.1}, {:6.1}, {:5.1}) | dist: {:5.1}m | {}",
            i,
            drone.position[0], drone.position[1], drone.position[2],
            dist, status);
    }

    let elapsed = start_time.elapsed();
    println!("\n=== Demo Complete ===");
    println!("Total runtime: {:.2}ms", elapsed.as_secs_f32() * 1000.0);

    println!("\n[OK] Collision avoidance demo completed!");
    println!("\nThe Hybrid algorithm (ORCA + APF) provides:");
    println!("  - Mathematically optimal collision avoidance (ORCA)");
    println!("  - Smooth trajectories with potential fields (APF)");
    println!("  - Real-time performance suitable for embedded systems");
}
