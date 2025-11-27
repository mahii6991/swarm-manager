//! Advanced Particle Swarm Optimization Examples
//!
//! Demonstrates PSO algorithms inspired by pySwarms:
//! - Global-best PSO (Star topology)
//! - Local-best PSO (Ring topology)
//! - Path planning optimization
//! - Formation optimization
//! - Multi-objective optimization

use drone_swarm_system::*;
use drone_swarm_system::pso::*;

fn main() -> Result<()> {
    println!("═══════════════════════════════════════════════════════");
    println!("  Particle Swarm Optimization for Drone Swarms");
    println!("  Inspired by pySwarms Library");
    println!("═══════════════════════════════════════════════════════\n");

    // Example 1: Basic Optimization - Sphere Function
    println!("📊 Example 1: Sphere Function Minimization");
    println!("─────────────────────────────────────────────────────");
    example_sphere_function()?;
    println!();

    // Example 2: Rastrigin Function (more challenging)
    println!("📊 Example 2: Rastrigin Function Optimization");
    println!("─────────────────────────────────────────────────────");
    example_rastrigin_function()?;
    println!();

    // Example 3: Drone Path Planning with Obstacles
    println!("🛤️  Example 3: Drone Path Planning");
    println!("─────────────────────────────────────────────────────");
    example_path_planning()?;
    println!();

    // Example 4: Formation Optimization
    println!("🎯 Example 4: Swarm Formation Optimization");
    println!("─────────────────────────────────────────────────────");
    example_formation_optimization()?;
    println!();

    // Example 5: Comparison of Global vs Local Best PSO
    println!("⚖️  Example 5: Global-Best vs Local-Best PSO");
    println!("─────────────────────────────────────────────────────");
    example_comparison()?;
    println!();

    println!("✅ All PSO examples completed successfully!");

    Ok(())
}

/// Example 1: Minimize sphere function
/// f(x) = sum(x_i^2), minimum at x = [0, 0, ..., 0]
fn example_sphere_function() -> Result<()> {
    println!("  Problem: Minimize f(x) = Σ(x_i²)");
    println!("  Dimensions: 10");
    println!("  Search space: [-100, 100]^10");
    println!("  Global minimum: f(0, 0, ..., 0) = 0\n");

    let cost_fn = |x: &[f32]| -> f32 {
        x.iter().map(|&xi| xi * xi).sum()
    };

    let bounds = Bounds::uniform(10, -100.0, 100.0)?;
    let options = PSOOptions::default();
    let mut pso = GlobalBestPSO::new(30, 10, bounds, options)?;

    println!("  Running PSO with 30 particles...");
    print!("  Iteration: ");

    for i in 0..100 {
        let cost = pso.step(&cost_fn)?;
        if i % 10 == 0 {
            print!("{} ", i);
        }
    }

    println!("\n");
    println!("  ✓ Optimization complete!");
    println!("  Best cost: {:.6}", pso.best_cost());
    println!("  Best position (first 5 dims): [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
             pso.best_position()[0],
             pso.best_position()[1],
             pso.best_position()[2],
             pso.best_position()[3],
             pso.best_position()[4]);

    Ok(())
}

/// Example 2: Rastrigin function (highly multimodal)
/// f(x) = 10n + sum(x_i^2 - 10*cos(2*pi*x_i))
fn example_rastrigin_function() -> Result<()> {
    println!("  Problem: Minimize Rastrigin function (highly multimodal)");
    println!("  Dimensions: 5");
    println!("  Search space: [-5.12, 5.12]^5");
    println!("  Global minimum: f(0, 0, ..., 0) = 0\n");

    let cost_fn = |x: &[f32]| -> f32 {
        let n = x.len() as f32;
        let pi = core::f32::consts::PI;

        10.0 * n + x.iter()
            .map(|&xi| xi * xi - 10.0 * libm::cosf(2.0 * pi * xi))
            .sum::<f32>()
    };

    let bounds = Bounds::uniform(5, -5.12, 5.12)?;

    // Use constriction coefficient for better convergence
    let options = PSOOptions::constriction();
    let mut pso = GlobalBestPSO::new(40, 5, bounds, options)?;

    println!("  Running PSO with constriction coefficient...");
    print!("  Iteration: ");

    for i in 0..200 {
        pso.step(&cost_fn)?;
        if i % 20 == 0 {
            print!("{} ", i);
        }
    }

    println!("\n");
    println!("  ✓ Optimization complete!");
    println!("  Best cost: {:.6}", pso.best_cost());
    println!("  Best position: [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
             pso.best_position()[0],
             pso.best_position()[1],
             pso.best_position()[2],
             pso.best_position()[3],
             pso.best_position()[4]);

    Ok(())
}

/// Example 3: Drone path planning with obstacles
fn example_path_planning() -> Result<()> {
    let start = Position { x: 0.0, y: 0.0, z: 10.0 };
    let goal = Position { x: 500.0, y: 500.0, z: 10.0 };

    println!("  Start: ({:.1}, {:.1}, {:.1})", start.x, start.y, start.z);
    println!("  Goal:  ({:.1}, {:.1}, {:.1})", goal.x, goal.y, goal.z);
    println!("  Obstacles: 3 circular obstacles\n");

    let mut optimizer = DronePathOptimizer::new(start, goal, 5)?;

    // Add obstacles
    optimizer.add_obstacle(Position { x: 150.0, y: 150.0, z: 10.0 }, 50.0)?;
    optimizer.add_obstacle(Position { x: 300.0, y: 200.0, z: 10.0 }, 40.0)?;
    optimizer.add_obstacle(Position { x: 400.0, y: 400.0, z: 10.0 }, 60.0)?;

    println!("  Optimizing path with 5 waypoints...");
    let path = optimizer.optimize(100)?;

    println!("\n  ✓ Path optimization complete!");
    println!("  Path length: {} waypoints", path.len());
    println!("  Waypoints:");
    for (i, pos) in path.iter().enumerate() {
        println!("    {}. ({:.1}, {:.1}, {:.1})", i + 1, pos.x, pos.y, pos.z);
    }

    Ok(())
}

/// Example 4: Formation optimization
fn example_formation_optimization() -> Result<()> {
    println!("  Goal: Optimize drone positions for communication range");
    println!("  Constraints:");
    println!("    - Maintain connectivity (all drones within 200m of at least 2 others)");
    println!("    - Maximize coverage area");
    println!("    - Avoid collisions (min 20m spacing)\n");

    // Optimize positions for 6 drones
    let n_drones = 6;
    let dimensions = n_drones * 3; // x, y, z for each drone

    let bounds = Bounds::uniform(dimensions, 0.0, 1000.0)?;
    let options = PSOOptions::balanced();
    let mut pso = GlobalBestPSO::new(25, dimensions, bounds, options)?;

    let cost_fn = |positions: &[f32]| -> f32 {
        let mut cost = 0.0;

        // Extract drone positions
        let mut drones = heapless::Vec::<Position, 10>::new();
        for i in 0..n_drones {
            drones.push(Position {
                x: positions[i * 3],
                y: positions[i * 3 + 1],
                z: positions[i * 3 + 2],
            }).ok();
        }

        // Penalty for drones too close (collision)
        for i in 0..drones.len() {
            for j in (i + 1)..drones.len() {
                let dist = drones[i].distance_to(&drones[j]);
                if dist < 20.0 {
                    cost += (20.0 - dist) * 10.0; // Collision penalty
                }
            }
        }

        // Penalty for disconnected drones (outside 200m from all others)
        for i in 0..drones.len() {
            let mut connected = 0;
            for j in 0..drones.len() {
                if i != j {
                    let dist = drones[i].distance_to(&drones[j]);
                    if dist <= 200.0 {
                        connected += 1;
                    }
                }
            }
            if connected < 2 {
                cost += 500.0; // Disconnection penalty
            }
        }

        // Reward for coverage (negative cost for larger spread)
        let mut center_x = 0.0;
        let mut center_y = 0.0;
        for drone in &drones {
            center_x += drone.x;
            center_y += drone.y;
        }
        center_x /= drones.len() as f32;
        center_y /= drones.len() as f32;

        let mut spread = 0.0;
        for drone in &drones {
            let dx = drone.x - center_x;
            let dy = drone.y - center_y;
            spread += libm::sqrtf(dx * dx + dy * dy);
        }

        cost -= spread * 0.1; // Reward larger spread

        cost
    };

    println!("  Running formation optimization...");
    let (best_positions, best_cost) = pso.optimize(150, cost_fn)?;

    println!("\n  ✓ Formation optimization complete!");
    println!("  Final cost: {:.2}", best_cost);
    println!("  Optimized drone positions:");

    for i in 0..n_drones {
        println!("    Drone {}: ({:.1}, {:.1}, {:.1})",
                 i + 1,
                 best_positions[i * 3],
                 best_positions[i * 3 + 1],
                 best_positions[i * 3 + 2]);
    }

    Ok(())
}

/// Example 5: Compare Global-Best vs Local-Best PSO
fn example_comparison() -> Result<()> {
    println!("  Comparing PSO variants on Rosenbrock function");
    println!("  Function: f(x,y) = (1-x)² + 100(y-x²)²\n");

    let cost_fn = |x: &[f32]| -> f32 {
        let mut sum = 0.0;
        for i in 0..(x.len() - 1) {
            let term1 = 1.0 - x[i];
            let term2 = x[i + 1] - x[i] * x[i];
            sum += term1 * term1 + 100.0 * term2 * term2;
        }
        sum
    };

    let bounds = Bounds::uniform(5, -5.0, 10.0)?;
    let options = PSOOptions::balanced();

    // Global-best PSO
    println!("  Running Global-Best PSO (Star topology)...");
    let mut gbest_pso = GlobalBestPSO::new(30, 5, bounds.clone(), options)?;
    gbest_pso.optimize(100, &cost_fn)?;

    println!("    Best cost: {:.6}", gbest_pso.best_cost());

    // Local-best PSO
    println!("\n  Running Local-Best PSO (Ring topology, k=3)...");
    let mut lbest_pso = LocalBestPSO::new(30, 5, bounds, options, 3)?;

    for _ in 0..100 {
        lbest_pso.step(&cost_fn)?;
    }

    println!("    Best cost: {:.6}", lbest_pso.best_cost());

    println!("\n  Analysis:");
    println!("    - Global-Best: Faster convergence, may get stuck in local optima");
    println!("    - Local-Best: Slower convergence, better exploration");
    println!("    - Choose based on problem characteristics!");

    Ok(())
}
