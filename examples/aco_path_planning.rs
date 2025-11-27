//! Ant Colony Optimization (ACO) Path Planning Example
//!
//! Demonstrates how to use ACO for finding optimal paths in 3D space
//! with obstacles, suitable for drone navigation.
//!
//! This example shows:
//! - Setting up start and goal positions
//! - Defining 3D obstacles
//! - Configuring ACO parameters
//! - Running optimization
//! - Analyzing results

#![no_std]
#![no_main]

extern crate drone_swarm_system;

use drone_swarm_system::aco::*;
use drone_swarm_system::types::*;

#[no_mangle]
pub extern "C" fn main() -> ! {
    // Example 1: Simple 2D path planning
    example_2d_path_planning();

    // Example 2: 3D path planning with obstacles
    example_3d_with_obstacles();

    // Example 3: Multiple algorithm comparison
    compare_aco_algorithms();

    // Example 4: Real-world search & rescue scenario
    search_rescue_scenario();

    loop {}
}

/// Example 1: Simple 2D path planning
fn example_2d_path_planning() {
    println!("\n=== Example 1: Simple 2D Path Planning ===\n");

    // Define start and goal
    let start = Position3D::new(0.0, 0.0, 10.0);
    let goal = Position3D::new(100.0, 100.0, 10.0);

    // Define bounds
    let bounds_min = Position3D::new(-10.0, -10.0, 0.0);
    let bounds_max = Position3D::new(110.0, 110.0, 50.0);

    // Configure ACO
    let config = ACOConfig {
        algorithm: ACOAlgorithm::MMAS,
        num_ants: 20,
        max_iterations: 100,
        pheromone_init: 1.0,
        evaporation_rate: 0.1,
        alpha: 1.0,
        beta: 2.0,
    };

    // Create optimizer
    let mut optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max)
        .expect("Failed to create ACO optimizer");

    println!("Configuration:");
    println!("  Algorithm: Max-Min Ant System (MMAS)");
    println!("  Ants: 20");
    println!("  Iterations: 100");
    println!("  Start: ({}, {}, {})", start.x, start.y, start.z);
    println!("  Goal: ({}, {}, {})", goal.x, goal.y, goal.z);

    // Run optimization
    println!("\nRunning optimization...");
    let result = optimizer.optimize().expect("Optimization failed");

    // Print results
    println!("\n Results:");
    println!("  Path found: {}", result.is_valid);
    println!("  Path cost: {:.2}", result.cost);
    println!("  Waypoints: {}", result.waypoints.len());
    println!("  Iterations completed: {}", optimizer.get_iteration());

    if result.waypoints.len() > 0 {
        println!("\n  Path waypoints:");
        for (i, wp) in result.waypoints.iter().enumerate() {
            println!("    {}: ({:.2}, {:.2}, {:.2})", i, wp.x, wp.y, wp.z);
        }
    }
}

/// Example 2: 3D path planning with obstacles
fn example_3d_with_obstacles() {
    println!("\n=== Example 2: 3D Path Planning with Obstacles ===\n");

    // Define start and goal
    let start = Position3D::new(0.0, 0.0, 20.0);
    let goal = Position3D::new(100.0, 100.0, 20.0);

    // Define bounds
    let bounds_min = Position3D::new(-10.0, -10.0, 0.0);
    let bounds_max = Position3D::new(110.0, 110.0, 50.0);

    // Configure ACO
    let config = ACOConfig {
        algorithm: ACOAlgorithm::ACS,
        num_ants: 30,
        max_iterations: 150,
        pheromone_init: 1.0,
        evaporation_rate: 0.1,
        alpha: 1.0,
        beta: 2.5,
    };

    // Create optimizer
    let mut optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max)
        .expect("Failed to create ACO optimizer");

    // Add obstacles (buildings, mountains, no-fly zones)
    let obstacles = [
        Obstacle::new(Position3D::new(30.0, 30.0, 20.0), 15.0),  // Building 1
        Obstacle::new(Position3D::new(50.0, 60.0, 25.0), 12.0),  // Building 2
        Obstacle::new(Position3D::new(70.0, 40.0, 15.0), 10.0),  // Mountain
        Obstacle::new(Position3D::new(80.0, 80.0, 20.0), 18.0),  // Tower
    ];

    println!("Added {} obstacles:", obstacles.len());
    for (i, obs) in obstacles.iter().enumerate() {
        optimizer.add_obstacle(*obs).expect("Failed to add obstacle");
        println!("  Obstacle {}: center=({:.1}, {:.1}, {:.1}), radius={:.1}",
                 i, obs.center.x, obs.center.y, obs.center.z, obs.radius);
    }

    // Run optimization
    println!("\nRunning optimization with Ant Colony System (ACS)...");
    let result = optimizer.optimize().expect("Optimization failed");

    // Print results
    println!("\nResults:");
    println!("  Path found: {}", result.is_valid);
    println!("  Path cost: {:.2}", result.cost);
    println!("  Waypoints: {}", result.waypoints.len());
    println!("  Collision-free: {}", result.is_valid);

    if result.is_valid {
        println!("\n  ✓ Found valid collision-free path!");
        println!("  Total distance: {:.2} meters", result.cost);
    } else {
        println!("\n  ✗ No valid path found (all paths collide with obstacles)");
    }
}

/// Example 3: Compare different ACO algorithms
fn compare_aco_algorithms() {
    println!("\n=== Example 3: Algorithm Comparison ===\n");

    let start = Position3D::new(0.0, 0.0, 15.0);
    let goal = Position3D::new(80.0, 80.0, 15.0);
    let bounds_min = Position3D::new(-10.0, -10.0, 0.0);
    let bounds_max = Position3D::new(90.0, 90.0, 40.0);

    let algorithms = [
        (ACOAlgorithm::AntSystem, "Ant System (AS)"),
        (ACOAlgorithm::MMAS, "Max-Min Ant System (MMAS)"),
        (ACOAlgorithm::ACS, "Ant Colony System (ACS)"),
    ];

    println!("Comparing ACO algorithms on the same problem:\n");

    for (algo, name) in &algorithms {
        let config = ACOConfig {
            algorithm: *algo,
            num_ants: 25,
            max_iterations: 100,
            pheromone_init: 1.0,
            evaporation_rate: 0.1,
            alpha: 1.0,
            beta: 2.0,
        };

        let mut optimizer = ACOOptimizer::new(config, start, goal, bounds_min, bounds_max)
            .expect("Failed to create optimizer");

        // Add some obstacles
        optimizer.add_obstacle(Obstacle::new(Position3D::new(40.0, 40.0, 15.0), 12.0))
            .expect("Failed to add obstacle");

        let result = optimizer.optimize().expect("Optimization failed");

        println!("{}:", name);
        println!("  Best cost: {:.2}", result.cost);
        println!("  Valid path: {}", result.is_valid);
        println!("  Waypoints: {}", result.waypoints.len());
        println!();
    }
}

/// Example 4: Real-world search & rescue scenario
fn search_rescue_scenario() {
    println!("\n=== Example 4: Search & Rescue Scenario ===\n");
    println!("Scenario: Drone must navigate from base to disaster site");
    println!("          through urban environment with buildings.\n");

    // Base camp position
    let base = Position3D::new(0.0, 0.0, 25.0);

    // Disaster site (collapsed building)
    let disaster_site = Position3D::new(200.0, 200.0, 25.0);

    // Flight bounds (urban area)
    let bounds_min = Position3D::new(-20.0, -20.0, 10.0);  // Min altitude: 10m
    let bounds_max = Position3D::new(220.0, 220.0, 100.0); // Max altitude: 100m

    // Configure ACO for search & rescue (favor speed)
    let config = ACOConfig {
        algorithm: ACOAlgorithm::ACS,  // Fast convergence
        num_ants: 40,                   // More ants for thorough search
        max_iterations: 200,            // More iterations for complex environment
        pheromone_init: 1.0,
        evaporation_rate: 0.15,         // Higher evaporation for adaptability
        alpha: 1.0,
        beta: 3.0,                      // Strong heuristic influence (favor shorter paths)
    };

    let mut optimizer = ACOOptimizer::new(config, base, disaster_site, bounds_min, bounds_max)
        .expect("Failed to create optimizer");

    // Add urban obstacles (buildings)
    let buildings = [
        // Downtown area
        (50.0, 40.0, 30.0, 20.0),   // (x, y, height, radius)
        (80.0, 60.0, 40.0, 25.0),
        (60.0, 100.0, 35.0, 18.0),
        (100.0, 80.0, 45.0, 22.0),
        // Mid-town
        (120.0, 120.0, 30.0, 20.0),
        (150.0, 100.0, 38.0, 23.0),
        (140.0, 160.0, 32.0, 19.0),
        // Near disaster site
        (180.0, 180.0, 28.0, 17.0),
        (170.0, 200.0, 33.0, 21.0),
    ];

    println!("Urban environment:");
    println!("  {} buildings in flight path", buildings.len());
    println!("  Altitude range: {:.0}m - {:.0}m", bounds_min.z, bounds_max.z);
    println!("  Distance (straight line): {:.2}m", base.distance_to(&disaster_site));

    for (x, y, z, radius) in &buildings {
        let obstacle = Obstacle::new(Position3D::new(*x, *y, *z), *radius);
        optimizer.add_obstacle(obstacle).expect("Failed to add obstacle");
    }

    println!("\nOptimizing flight path...");
    let start_time = get_current_time();
    let result = optimizer.optimize().expect("Optimization failed");
    let elapsed = get_current_time() - start_time;

    println!("\n=== Results ===");
    println!("Optimization completed in {} ms", elapsed);
    println!();
    println!("Path Status:");
    println!("  ✓ Valid path found: {}", result.is_valid);
    println!("  ✓ Collision-free: {}", result.is_valid);
    println!();
    println!("Path Metrics:");
    println!("  Total distance: {:.2} meters", result.cost);
    println!("  Straight-line distance: {:.2} meters", base.distance_to(&disaster_site));
    println!("  Path efficiency: {:.1}%",
             (base.distance_to(&disaster_site) / result.cost) * 100.0);
    println!("  Number of waypoints: {}", result.waypoints.len());
    println!();
    println!("Flight Parameters:");
    println!("  Average altitude: {:.2} meters",
             result.waypoints.iter().map(|wp| wp.z).sum::<f32>() / result.waypoints.len() as f32);
    println!("  Estimated flight time: {:.1} seconds (at 10 m/s)",
             result.cost / 10.0);

    if result.waypoints.len() > 0 {
        println!("\nFlight Plan (waypoints):");
        for (i, wp) in result.waypoints.iter().enumerate() {
            println!("  WP{}: ({:.1}, {:.1}, {:.1})",
                     i, wp.x, wp.y, wp.z);
        }
    }

    println!("\n=== Mission Ready ===");
    println!("Drone can proceed with confidence on optimized path.");
}

// Placeholder for time function (implement based on target platform)
fn get_current_time() -> u32 {
    // TODO: Replace with actual time source
    0
}

// Placeholder for printing (implement based on target platform)
fn println(msg: &str) {
    // TODO: Replace with actual output mechanism
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
