//! Grey Wolf Optimizer (GWO) Swarm Optimization Example
//!
//! Demonstrates how to use GWO for optimizing drone swarm parameters
//! and coordination strategies.
//!
//! This example shows:
//! - Optimizing swarm formation parameters
//! - Multi-objective optimization
//! - Different GWO variants comparison
//! - Real-world swarm coordination scenarios

#![no_std]
#![no_main]

extern crate drone_swarm_system;

use drone_swarm_system::gwo::*;
use drone_swarm_system::types::*;

#[no_mangle]
pub extern "C" fn main() -> ! {
    // Example 1: Simple optimization (Sphere function)
    example_sphere_optimization();

    // Example 2: Complex optimization (Rastrigin function)
    example_rastrigin_optimization();

    // Example 3: Swarm formation optimization
    example_formation_optimization();

    // Example 4: Compare GWO variants
    compare_gwo_variants();

    // Example 5: Real-world swarm coordination
    swarm_coordination_scenario();

    loop {}
}

/// Example 1: Simple optimization using sphere function
fn example_sphere_optimization() {
    println!("\n=== Example 1: Sphere Function Optimization ===\n");
    println!("Objective: Minimize f(x) = x1² + x2² + ... + xn²");
    println!("Global minimum: f(0, 0, ..., 0) = 0\n");

    // Configure GWO
    let mut config = GWOConfig::default();
    config.dimensions = 5;
    config.num_wolves = 30;
    config.max_iterations = 100;
    config.variant = GWOVariant::Standard;

    // Define search space
    let bounds = Bounds::uniform(config.dimensions, -100.0, 100.0)
        .expect("Failed to create bounds");

    println!("Configuration:");
    println!("  Algorithm: Standard GWO");
    println!("  Wolves: {}", config.num_wolves);
    println!("  Dimensions: {}", config.dimensions);
    println!("  Iterations: {}", config.max_iterations);
    println!("  Search space: [-100, 100]^{}", config.dimensions);

    // Create optimizer
    let mut optimizer = GWOOptimizer::new(config, bounds)
        .expect("Failed to create optimizer");

    // Define fitness function (sphere)
    let sphere_fn = |x: &[f32]| -> f32 {
        x.iter().map(|&xi| xi * xi).sum()
    };

    println!("\nOptimizing...");
    let start_time = get_current_time();
    let result = optimizer.optimize(sphere_fn).expect("Optimization failed");
    let elapsed = get_current_time() - start_time;

    // Print results
    println!("\nResults:");
    println!("  Best fitness: {:.6}", result.fitness);
    println!("  Iterations: {}", optimizer.get_iteration());
    println!("  Time: {} ms", elapsed);
    println!("\n  Best solution:");
    for (i, &val) in result.position.iter().enumerate() {
        println!("    x{} = {:.6}", i, val);
    }

    if result.fitness < 0.01 {
        println!("\n  ✓ Successfully converged to global minimum!");
    }
}

/// Example 2: Complex optimization using Rastrigin function
fn example_rastrigin_optimization() {
    println!("\n=== Example 2: Rastrigin Function Optimization ===\n");
    println!("Objective: Minimize f(x) = 10n + Σ(xi² - 10cos(2πxi))");
    println!("Global minimum: f(0, 0, ..., 0) = 0");
    println!("Challenge: Multiple local minima\n");

    // Configure GWO with improved variant
    let mut config = GWOConfig::default();
    config.dimensions = 10;
    config.num_wolves = 50;
    config.max_iterations = 200;
    config.variant = GWOVariant::Improved;
    config.adaptive = true;

    let bounds = Bounds::uniform(config.dimensions, -5.12, 5.12)
        .expect("Failed to create bounds");

    println!("Configuration:");
    println!("  Algorithm: Improved GWO (IGWO)");
    println!("  Wolves: {}", config.num_wolves);
    println!("  Dimensions: {}", config.dimensions);
    println!("  Iterations: {}", config.max_iterations);
    println!("  Adaptive parameters: {}", config.adaptive);

    let mut optimizer = GWOOptimizer::new(config, bounds)
        .expect("Failed to create optimizer");

    // Rastrigin function
    let rastrigin_fn = |x: &[f32]| -> f32 {
        let n = x.len() as f32;
        let a = 10.0;
        a * n + x.iter().map(|&xi| {
            xi * xi - a * (2.0 * core::f32::consts::PI * xi).cos()
        }).sum::<f32>()
    };

    println!("\nOptimizing (this is a challenging problem)...");
    let result = optimizer.optimize(rastrigin_fn).expect("Optimization failed");

    println!("\nResults:");
    println!("  Best fitness: {:.6}", result.fitness);
    println!("  Alpha (best): {:.6}", optimizer.get_best().fitness);
    println!("  Beta (2nd best): {:.6}", optimizer.get_beta().fitness);
    println!("  Delta (3rd best): {:.6}", optimizer.get_delta().fitness);

    if result.fitness < 1.0 {
        println!("\n  ✓ Excellent! Near-optimal solution found!");
    } else if result.fitness < 10.0 {
        println!("\n  ✓ Good solution found!");
    } else {
        println!("\n  • Reasonable solution found (Rastrigin is very challenging)");
    }
}

/// Example 3: Optimize drone swarm formation parameters
fn example_formation_optimization() {
    println!("\n=== Example 3: Swarm Formation Optimization ===\n");
    println!("Objective: Find optimal formation parameters for drone swarm");
    println!("Parameters: spacing, altitude, velocity, cohesion, separation\n");

    // 5D optimization problem
    // Dimensions: [spacing, altitude, velocity, cohesion_weight, separation_weight]
    let config = GWOConfig {
        variant: GWOVariant::Hybrid,
        num_wolves: 30,
        max_iterations: 150,
        dimensions: 5,
        a_decay: true,
        adaptive: true,
    };

    // Bounds for each parameter
    let mut lower = heapless::Vec::<f32, MAX_DIMENSIONS>::new();
    let mut upper = heapless::Vec::<f32, MAX_DIMENSIONS>::new();

    // spacing: 5-50 meters
    lower.push(5.0).ok();
    upper.push(50.0).ok();

    // altitude: 20-100 meters
    lower.push(20.0).ok();
    upper.push(100.0).ok();

    // velocity: 2-15 m/s
    lower.push(2.0).ok();
    upper.push(15.0).ok();

    // cohesion weight: 0-1
    lower.push(0.0).ok();
    upper.push(1.0).ok();

    // separation weight: 0-1
    lower.push(0.0).ok();
    upper.push(1.0).ok();

    let bounds = Bounds::new(lower, upper);

    println!("Search space:");
    println!("  Spacing: 5-50 meters");
    println!("  Altitude: 20-100 meters");
    println!("  Velocity: 2-15 m/s");
    println!("  Cohesion weight: 0-1");
    println!("  Separation weight: 0-1");

    let mut optimizer = GWOOptimizer::new(config, bounds)
        .expect("Failed to create optimizer");

    // Fitness function: balance formation quality, energy, and safety
    let formation_fitness = |params: &[f32]| -> f32 {
        let spacing = params[0];
        let altitude = params[1];
        let velocity = params[2];
        let cohesion = params[3];
        let separation = params[4];

        // Energy cost (favor lower velocity, moderate altitude)
        let energy_cost = velocity * velocity + (altitude - 50.0).abs() * 0.1;

        // Formation quality (prefer moderate spacing, balanced weights)
        let quality_cost = (spacing - 20.0).abs() * 0.5
            + (cohesion - 0.5).abs() * 10.0
            + (separation - 0.5).abs() * 10.0;

        // Safety margin (penalize too close or too low)
        let safety_cost = if spacing < 10.0 { 100.0 } else { 0.0 }
            + if altitude < 25.0 { 50.0 } else { 0.0 };

        // Total fitness (minimize)
        energy_cost + quality_cost + safety_cost
    };

    println!("\nOptimizing formation parameters...");
    let result = optimizer.optimize(formation_fitness)
        .expect("Optimization failed");

    println!("\nOptimal Formation Parameters:");
    println!("  Spacing: {:.2} meters", result.position[0]);
    println!("  Altitude: {:.2} meters", result.position[1]);
    println!("  Velocity: {:.2} m/s", result.position[2]);
    println!("  Cohesion weight: {:.3}", result.position[3]);
    println!("  Separation weight: {:.3}", result.position[4]);
    println!("\n  Total cost: {:.4}", result.fitness);
    println!("\n  ✓ Formation parameters optimized for efficiency and safety!");
}

/// Example 4: Compare different GWO variants
fn compare_gwo_variants() {
    println!("\n=== Example 4: GWO Variants Comparison ===\n");

    let variants = [
        (GWOVariant::Standard, "Standard GWO"),
        (GWOVariant::Improved, "Improved GWO (IGWO)"),
        (GWOVariant::Hybrid, "Hybrid GWO-PSO"),
        (GWOVariant::Chaotic, "Chaotic GWO"),
    ];

    // Test function: Rosenbrock (challenging)
    let rosenbrock = |x: &[f32]| -> f32 {
        let mut sum = 0.0;
        for i in 0..x.len() - 1 {
            let a = 1.0 - x[i];
            let b = x[i + 1] - x[i] * x[i];
            sum += a * a + 100.0 * b * b;
        }
        sum
    };

    println!("Test function: Rosenbrock (challenging optimization problem)");
    println!("Comparing performance of different GWO variants:\n");

    for (variant, name) in &variants {
        let config = GWOConfig {
            variant: *variant,
            num_wolves: 30,
            max_iterations: 100,
            dimensions: 5,
            a_decay: true,
            adaptive: true,
        };

        let bounds = Bounds::uniform(config.dimensions, -5.0, 10.0)
            .expect("Failed to create bounds");

        let mut optimizer = GWOOptimizer::new(config, bounds)
            .expect("Failed to create optimizer");

        let result = optimizer.optimize(&rosenbrock)
            .expect("Optimization failed");

        println!("{}:", name);
        println!("  Best fitness: {:.6}", result.fitness);
        println!("  Convergence quality: {}",
                 if result.fitness < 1.0 { "Excellent" }
                 else if result.fitness < 10.0 { "Good" }
                 else { "Fair" });
        println!();
    }
}

/// Example 5: Real-world swarm coordination scenario
fn swarm_coordination_scenario() {
    println!("\n=== Example 5: Real-World Swarm Coordination ===\n");
    println!("Scenario: Optimize patrol pattern for 10-drone security swarm");
    println!("Objectives: Maximize coverage, minimize energy, ensure safety\n");

    // Optimization parameters:
    // [patrol_radius, patrol_speed, formation_type, rotation_period, altitude]
    let config = GWOConfig {
        variant: GWOVariant::Hybrid,
        num_wolves: 40,
        max_iterations: 200,
        dimensions: 5,
        a_decay: true,
        adaptive: true,
    };

    let mut lower = heapless::Vec::<f32, MAX_DIMENSIONS>::new();
    let mut upper = heapless::Vec::<f32, MAX_DIMENSIONS>::new();

    // patrol_radius: 50-500 meters
    lower.push(50.0).ok();
    upper.push(500.0).ok();

    // patrol_speed: 3-12 m/s
    lower.push(3.0).ok();
    upper.push(12.0).ok();

    // formation_type: 0-4 (encoded: circle=0, grid=1, line=2, v=3, star=4)
    lower.push(0.0).ok();
    upper.push(4.0).ok();

    // rotation_period: 30-300 seconds
    lower.push(30.0).ok();
    upper.push(300.0).ok();

    // altitude: 30-120 meters
    lower.push(30.0).ok();
    upper.push(120.0).ok();

    let bounds = Bounds::new(lower, upper);

    println!("Optimization constraints:");
    println!("  Patrol radius: 50-500 meters");
    println!("  Patrol speed: 3-12 m/s");
    println!("  Formation types: Circle, Grid, Line, V, Star");
    println!("  Rotation period: 30-300 seconds");
    println!("  Altitude: 30-120 meters");

    let mut optimizer = GWOOptimizer::new(config, bounds)
        .expect("Failed to create optimizer");

    // Multi-objective fitness function
    let patrol_fitness = |params: &[f32]| -> f32 {
        let patrol_radius = params[0];
        let patrol_speed = params[1];
        let formation_type = params[2].round();
        let rotation_period = params[3];
        let altitude = params[4];

        // Coverage score (larger radius = better coverage)
        let coverage_score = 1000.0 - patrol_radius;  // Minimize negative coverage

        // Energy cost (higher speed = more energy)
        let energy_cost = patrol_speed * patrol_speed * 10.0;

        // Formation efficiency (some formations are more efficient)
        let formation_cost = match formation_type as i32 {
            0 => 0.0,   // Circle - excellent
            1 => 5.0,   // Grid - good
            2 => 10.0,  // Line - fair
            3 => 3.0,   // V - very good
            4 => 7.0,   // Star - good
            _ => 20.0,  // Invalid
        };

        // Rotation efficiency (moderate period is best)
        let rotation_cost = (rotation_period - 120.0).abs() * 0.1;

        // Altitude cost (moderate altitude preferred)
        let altitude_cost = (altitude - 60.0).abs() * 0.2;

        // Total fitness (minimize)
        coverage_score + energy_cost + formation_cost + rotation_cost + altitude_cost
    };

    println!("\nOptimizing patrol parameters...");
    let start_time = get_current_time();
    let result = optimizer.optimize(patrol_fitness)
        .expect("Optimization failed");
    let elapsed = get_current_time() - start_time;

    let formation_names = ["Circle", "Grid", "Line", "V-Formation", "Star"];
    let formation_idx = result.position[2].round() as usize % formation_names.len();

    println!("\n=== Optimal Patrol Configuration ===");
    println!();
    println!("Formation Parameters:");
    println!("  Formation type: {}", formation_names[formation_idx]);
    println!("  Patrol radius: {:.1} meters", result.position[0]);
    println!("  Patrol speed: {:.2} m/s", result.position[1]);
    println!("  Rotation period: {:.1} seconds", result.position[3]);
    println!("  Patrol altitude: {:.1} meters", result.position[4]);
    println!();
    println!("Performance Metrics:");
    println!("  Coverage area: {:.0} m²",
             core::f32::consts::PI * result.position[0] * result.position[0]);
    println!("  Patrol cycle time: {:.1} seconds",
             2.0 * core::f32::consts::PI * result.position[0] / result.position[1]);
    println!("  Energy efficiency: {}",
             if result.position[1] < 8.0 { "High" } else { "Moderate" });
    println!("  Optimization time: {} ms", elapsed);
    println!();
    println!("  ✓ Patrol pattern optimized!");
    println!("  ✓ Ready for deployment!");
}

// Placeholder for time function
fn get_current_time() -> u32 {
    0
}

// Placeholder for printing
fn println(msg: &str) {
    // TODO: Replace with actual output
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
