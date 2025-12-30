//! Deep Reinforcement Learning Optimization Example
//!
//! This example demonstrates the Q-learning based algorithm selector
//! that dynamically chooses between PSO, GWO, and WOA optimizers
//! based on fitness landscape characteristics.
//!
//! Run with: cargo run --example deep_rl_optimization --features deep_rl

use drone_swarm_system::algorithm_selector::QLearningConfig;
use drone_swarm_system::hybrid_optimizer::{
    ackley_function, rastrigin_function, rosenbrock_function, sphere_function, HybridConfig,
    HybridOptimizer,
};
use drone_swarm_system::meta_heuristic::Bounds;

fn main() {
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║     Deep RL Algorithm Selection for Optimization            ║");
    println!("║     Q-Learning Based PSO/GWO/WOA Hybrid Optimizer           ║");
    println!("╚══════════════════════════════════════════════════════════════╝\n");

    // Configure the hybrid optimizer
    let config = HybridConfig {
        max_iterations: 200,
        population_size: 30,
        dimensions: 3,
        target_fitness: 0.01,
        switch_interval: 25, // Consider switching every 25 iterations
        seed: 42,
        rl_config: QLearningConfig {
            alpha: 0.2,              // Learning rate
            gamma: 0.95,             // Discount factor
            epsilon_initial: 0.4,    // Initial exploration
            epsilon_final: 0.05,     // Final exploration
            epsilon_decay: 0.98,     // Decay rate
            use_replay: true,
            batch_size: 8,
            ucb_c: 1.0,
        },
    };

    let bounds = Bounds::uniform(3, -5.0, 5.0).expect("Failed to create bounds");

    println!("Configuration:");
    println!("  - Max iterations: {}", config.max_iterations);
    println!("  - Population size: {}", config.population_size);
    println!("  - Dimensions: {}", config.dimensions);
    println!("  - Switch interval: {} iterations", config.switch_interval);
    println!("  - Initial epsilon: {:.2}", config.rl_config.epsilon_initial);
    println!("  - Bounds: [{}, {}] per dimension", -5.0, 5.0);
    println!();

    // Test on multiple benchmark functions
    let benchmarks: Vec<(&str, fn(&[f32]) -> f32, f32)> = vec![
        ("Sphere (unimodal)", sphere_function, 0.0),
        ("Rastrigin (multimodal)", rastrigin_function, 0.0),
        ("Rosenbrock (valley)", rosenbrock_function, 0.0),
        ("Ackley (many local minima)", ackley_function, 0.0),
    ];

    println!("═══════════════════════════════════════════════════════════════");
    println!("Running optimization on benchmark functions...\n");

    for (name, cost_fn, optimal) in benchmarks {
        println!("┌─────────────────────────────────────────────────────────────┐");
        println!("│ Benchmark: {:<49}│", name);
        println!("│ Global optimum: {:.4}{:<42}│", optimal, "");
        println!("└─────────────────────────────────────────────────────────────┘");

        let mut optimizer = HybridOptimizer::new(config.clone());

        match optimizer.optimize(cost_fn, &bounds) {
            Ok(result) => {
                let stats = optimizer.stats();

                println!("  Result:");
                println!(
                    "    Best fitness: {:.6} (error: {:.6})",
                    result.fitness,
                    (result.fitness - optimal).abs()
                );
                println!("    Best solution: [");
                for (i, val) in result.solution.iter().enumerate() {
                    println!("      x[{}] = {:.6}", i, val);
                }
                println!("    ]");
                println!("    Total evaluations: {}", result.evaluations);
                println!("    Converged: {}", result.converged);
                println!("    Final algorithm: {:?}", result.algorithm);
                println!();

                println!("  Statistics:");
                println!("    Total iterations: {}", stats.total_iterations);
                println!("    Algorithm switches: {}", stats.num_switches);
                println!("    Final epsilon: {:.4}", stats.final_epsilon);
                println!("    Average reward: {:.4}", stats.avg_reward);
                println!();

                println!("  Algorithm usage:");
                let alg_names = ["PSO", "ACO", "GWO", "WOA"];
                for (i, name) in alg_names.iter().enumerate() {
                    let iters = stats.algorithm_iterations[i];
                    let best = stats.algorithm_best_fitness[i];
                    if iters > 0 {
                        println!(
                            "    {}: {} iterations, best fitness: {:.6}",
                            name, iters, best
                        );
                    }
                }
            }
            Err(e) => {
                println!("  Error: {:?}", e);
            }
        }

        println!();
    }

    println!("═══════════════════════════════════════════════════════════════");
    println!("Demonstration complete!");
    println!();
    println!("Key features demonstrated:");
    println!("  1. Q-learning based algorithm selection");
    println!("  2. Dynamic switching between PSO, GWO, and WOA");
    println!("  3. Epsilon-greedy exploration with decay");
    println!("  4. Experience replay for stable learning");
    println!("  5. UCB exploration bonus for under-explored actions");
}
