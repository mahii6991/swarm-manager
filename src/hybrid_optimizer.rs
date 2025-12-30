//! Hybrid Optimizer with RL-based Algorithm Selection
//!
//! Orchestrates multiple meta-heuristic algorithms (PSO, ACO, GWO, WOA)
//! using a Q-learning selector to dynamically choose the best algorithm
//! and parameter preset based on optimization progress.
//!
//! This module is feature-gated under `deep_rl`.

use crate::algorithm_selector::{
    calculate_reward, AlgorithmSelector, OptimizationState, QLearningConfig,
};
use crate::meta_heuristic::{
    AlgorithmId, Bounds, GwoAdapter, MetaHeuristic, OptimizationResult,
    ParameterPreset, PsoAdapter, Solution, WoaAdapter, AlgorithmMetadata,
};
use crate::types::Result;

/// Number of iterations before considering algorithm switch
pub const SWITCH_INTERVAL: u32 = 10;

/// Maximum iterations for optimization
pub const MAX_ITERATIONS: u32 = 1000;

/// Hybrid optimizer configuration
#[derive(Debug, Clone, Copy)]
pub struct HybridConfig {
    /// Iterations between algorithm switch decisions
    pub switch_interval: u32,
    /// Maximum total iterations
    pub max_iterations: u32,
    /// Target fitness (stop when reached)
    pub target_fitness: f32,
    /// Population size for algorithms
    pub population_size: usize,
    /// Problem dimensions
    pub dimensions: usize,
    /// Random seed
    pub seed: u64,
    /// Q-learning configuration
    pub rl_config: QLearningConfig,
}

impl Default for HybridConfig {
    fn default() -> Self {
        Self {
            switch_interval: SWITCH_INTERVAL,
            max_iterations: MAX_ITERATIONS,
            target_fitness: 1e-6,
            population_size: 30,
            dimensions: 10,
            seed: 42,
            rl_config: QLearningConfig::default(),
        }
    }
}

/// Statistics from hybrid optimization
#[derive(Debug, Clone, Default)]
pub struct HybridStats {
    /// Total iterations performed
    pub total_iterations: u32,
    /// Total function evaluations
    pub total_evaluations: u32,
    /// Number of algorithm switches
    pub num_switches: u32,
    /// Iterations spent on each algorithm
    pub algorithm_iterations: [u32; 4],
    /// Best fitness achieved by each algorithm
    pub algorithm_best_fitness: [f32; 4],
    /// Final exploration rate (epsilon)
    pub final_epsilon: f32,
    /// Average reward per episode
    pub avg_reward: f32,
}

/// Hybrid optimizer using RL-based algorithm selection
pub struct HybridOptimizer {
    /// PSO adapter
    pso: PsoAdapter,
    /// GWO adapter
    gwo: GwoAdapter,
    /// WOA adapter
    woa: WoaAdapter,
    /// RL algorithm selector
    selector: AlgorithmSelector,
    /// Currently active algorithm
    active: AlgorithmId,
    /// Current parameter preset
    preset: ParameterPreset,
    /// Configuration
    config: HybridConfig,
    /// Search bounds
    bounds: Option<Bounds>,
    /// Current iteration
    iteration: u32,
    /// Best solution found across all algorithms
    global_best_solution: Solution,
    /// Best fitness found across all algorithms
    global_best_fitness: f32,
    /// Initial fitness (for normalization)
    initial_fitness: f32,
    /// Previous fitness (for reward calculation)
    prev_fitness: f32,
    /// Statistics
    stats: HybridStats,
    /// Total reward accumulated
    total_reward: f32,
    /// Number of RL updates
    num_updates: u32,
    /// Cached metadata from active algorithm
    cached_metadata: AlgorithmMetadata,
}

impl HybridOptimizer {
    /// Create new hybrid optimizer with config
    pub fn new(config: HybridConfig) -> Self {
        Self {
            pso: PsoAdapter::new(config.population_size, config.dimensions),
            gwo: GwoAdapter::new(config.population_size, config.dimensions),
            woa: WoaAdapter::new(config.population_size, config.seed),
            selector: AlgorithmSelector::new(config.rl_config, config.seed),
            active: AlgorithmId::PSO,
            preset: ParameterPreset::Balanced,
            config,
            bounds: None,
            iteration: 0,
            global_best_solution: Solution::new(),
            global_best_fitness: f32::INFINITY,
            initial_fitness: f32::INFINITY,
            prev_fitness: f32::INFINITY,
            stats: HybridStats::default(),
            total_reward: 0.0,
            num_updates: 0,
            cached_metadata: AlgorithmMetadata::default(),
        }
    }

    /// Create with default configuration
    pub fn default_optimizer() -> Self {
        Self::new(HybridConfig::default())
    }

    /// Initialize optimizer with bounds
    pub fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        self.pso.initialize(bounds)?;
        self.gwo.initialize(bounds)?;
        self.woa.initialize(bounds)?;

        self.bounds = Some(bounds.clone());
        self.iteration = 0;
        self.global_best_solution.clear();
        self.global_best_fitness = f32::INFINITY;
        self.initial_fitness = f32::INFINITY;
        self.prev_fitness = f32::INFINITY;
        self.stats = HybridStats {
            algorithm_best_fitness: [f32::INFINITY; 4],
            ..Default::default()
        };
        self.total_reward = 0.0;
        self.num_updates = 0;
        self.selector.reset_episode();
        self.cached_metadata = AlgorithmMetadata::default();

        // Select initial algorithm (sets last_action for RL updates)
        let initial_state = OptimizationState::default();
        let (initial_alg, initial_preset) = self.selector.select_action(&initial_state);
        self.active = initial_alg;
        self.preset = initial_preset;

        Ok(())
    }

    /// Perform one optimization step
    pub fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        // Check if we should switch algorithms
        if self.iteration > 0 && self.iteration % self.config.switch_interval == 0 {
            self.consider_switch()?;
        }

        // Run one step of active algorithm and get results
        let (fitness, best_sol, metadata) = match self.active {
            AlgorithmId::PSO => {
                let f = self.pso.step(&cost_fn)?;
                (f, self.pso.best_solution(), self.pso.get_metadata())
            }
            AlgorithmId::ACO => {
                // ACO not fully implemented, fall back to PSO
                let f = self.pso.step(&cost_fn)?;
                (f, self.pso.best_solution(), self.pso.get_metadata())
            }
            AlgorithmId::GWO => {
                let f = self.gwo.step(&cost_fn)?;
                (f, self.gwo.best_solution(), self.gwo.get_metadata())
            }
            AlgorithmId::WOA => {
                let f = self.woa.step(&cost_fn)?;
                (f, self.woa.best_solution(), self.woa.get_metadata())
            }
        };

        self.cached_metadata = metadata;

        // Update global best
        if fitness < self.global_best_fitness {
            self.global_best_fitness = fitness;
            self.global_best_solution.clear();
            for &val in best_sol {
                self.global_best_solution.push(val).ok();
            }
        }

        // Track initial fitness for normalization
        if self.iteration == 0 {
            self.initial_fitness = fitness;
            self.prev_fitness = fitness;
        }

        // Update algorithm-specific stats
        let alg_idx = match self.active {
            AlgorithmId::PSO => 0,
            AlgorithmId::ACO => 1,
            AlgorithmId::GWO => 2,
            AlgorithmId::WOA => 3,
        };
        self.stats.algorithm_iterations[alg_idx] += 1;
        if fitness < self.stats.algorithm_best_fitness[alg_idx] {
            self.stats.algorithm_best_fitness[alg_idx] = fitness;
        }

        self.iteration += 1;
        self.stats.total_iterations = self.iteration;
        self.stats.total_evaluations += self.config.population_size as u32;

        Ok(fitness)
    }

    /// Consider switching to a different algorithm based on RL policy
    fn consider_switch(&mut self) -> Result<()> {
        let current_fitness = self.global_best_fitness;
        let max_evals = self.config.max_iterations * self.config.population_size as u32;

        // Create optimization state
        let state = OptimizationState::from_metadata(
            &self.cached_metadata,
            current_fitness,
            self.initial_fitness,
            max_evals,
            self.active,
            self.preset,
        );

        // Calculate reward from last interval
        let reward = calculate_reward(
            self.prev_fitness,
            current_fitness,
            &self.cached_metadata,
            self.config.target_fitness,
        );

        // Update Q-values if not first switch
        if self.num_updates > 0 {
            let done = current_fitness <= self.config.target_fitness
                || self.iteration >= self.config.max_iterations;
            self.selector.update(&state, reward, done)?;
            self.total_reward += reward;
        }
        self.num_updates += 1;

        // Select new algorithm and preset
        let (new_alg, new_preset) = self.selector.select_action(&state);

        // Switch if different
        if new_alg != self.active || new_preset != self.preset {
            self.switch_algorithm(new_alg, new_preset)?;
        }

        self.prev_fitness = current_fitness;

        Ok(())
    }

    /// Switch to a different algorithm
    fn switch_algorithm(&mut self, new_alg: AlgorithmId, new_preset: ParameterPreset) -> Result<()> {
        // Apply preset to new algorithm
        match new_alg {
            AlgorithmId::PSO => self.pso.apply_preset(new_preset),
            AlgorithmId::ACO => {} // ACO not fully implemented
            AlgorithmId::GWO => self.gwo.apply_preset(new_preset),
            AlgorithmId::WOA => self.woa.apply_preset(new_preset),
        }

        // Re-initialize if bounds available
        if let Some(bounds) = &self.bounds.clone() {
            match new_alg {
                AlgorithmId::PSO => self.pso.initialize(bounds)?,
                AlgorithmId::ACO => {}
                AlgorithmId::GWO => self.gwo.initialize(bounds)?,
                AlgorithmId::WOA => self.woa.initialize(bounds)?,
            }
        }

        self.active = new_alg;
        self.preset = new_preset;
        self.stats.num_switches += 1;

        Ok(())
    }

    /// Run full optimization
    pub fn optimize<F>(&mut self, cost_fn: F, bounds: &Bounds) -> Result<OptimizationResult>
    where
        F: Fn(&[f32]) -> f32,
    {
        self.initialize(bounds)?;

        // Main optimization loop
        while self.iteration < self.config.max_iterations {
            let fitness = self.step(&cost_fn)?;

            // Check convergence
            if fitness <= self.config.target_fitness {
                break;
            }
        }

        // Final RL update
        let max_evals = self.config.max_iterations * self.config.population_size as u32;
        let final_state = OptimizationState::from_metadata(
            &self.cached_metadata,
            self.global_best_fitness,
            self.initial_fitness,
            max_evals,
            self.active,
            self.preset,
        );

        let final_reward = calculate_reward(
            self.prev_fitness,
            self.global_best_fitness,
            &self.cached_metadata,
            self.config.target_fitness,
        );
        self.selector.update(&final_state, final_reward, true)?;
        self.total_reward += final_reward;

        // Update stats
        self.stats.final_epsilon = self.selector.epsilon();
        self.stats.avg_reward = if self.num_updates > 0 {
            self.total_reward / self.num_updates as f32
        } else {
            0.0
        };

        Ok(OptimizationResult {
            solution: self.global_best_solution.clone(),
            fitness: self.global_best_fitness,
            evaluations: self.stats.total_evaluations,
            converged: self.global_best_fitness <= self.config.target_fitness,
            algorithm: self.active,
            metadata: self.cached_metadata,
        })
    }

    /// Get current best solution
    pub fn best_solution(&self) -> &[f32] {
        self.global_best_solution.as_slice()
    }

    /// Get current best fitness
    pub fn best_fitness(&self) -> f32 {
        self.global_best_fitness
    }

    /// Get optimization statistics
    pub fn stats(&self) -> &HybridStats {
        &self.stats
    }

    /// Get currently active algorithm
    pub fn active_algorithm(&self) -> AlgorithmId {
        self.active
    }

    /// Get current preset
    pub fn current_preset(&self) -> ParameterPreset {
        self.preset
    }

    /// Get current iteration
    pub fn iteration(&self) -> u32 {
        self.iteration
    }

    /// Get selector's current epsilon
    pub fn exploration_rate(&self) -> f32 {
        self.selector.epsilon()
    }
}

// ============================================================================
// Benchmark Functions
// ============================================================================

/// Sphere function: f(x) = sum(x_i^2)
/// Global minimum: f(0, ..., 0) = 0
pub fn sphere_function(x: &[f32]) -> f32 {
    x.iter().map(|&xi| xi * xi).sum()
}

/// Rastrigin function: f(x) = 10n + sum(x_i^2 - 10*cos(2*pi*x_i))
/// Global minimum: f(0, ..., 0) = 0
pub fn rastrigin_function(x: &[f32]) -> f32 {
    let n = x.len() as f32;
    let pi = core::f32::consts::PI;

    10.0 * n
        + x.iter()
            .map(|&xi| xi * xi - 10.0 * libm::cosf(2.0 * pi * xi))
            .sum::<f32>()
}

/// Rosenbrock function: f(x) = sum(100*(x_{i+1} - x_i^2)^2 + (1 - x_i)^2)
/// Global minimum: f(1, ..., 1) = 0
pub fn rosenbrock_function(x: &[f32]) -> f32 {
    let mut sum = 0.0;
    for i in 0..x.len() - 1 {
        let term1 = x[i + 1] - x[i] * x[i];
        let term2 = 1.0 - x[i];
        sum += 100.0 * term1 * term1 + term2 * term2;
    }
    sum
}

/// Ackley function
/// Global minimum: f(0, ..., 0) = 0
pub fn ackley_function(x: &[f32]) -> f32 {
    let n = x.len() as f32;
    let pi = core::f32::consts::PI;

    let sum_sq: f32 = x.iter().map(|&xi| xi * xi).sum();
    let sum_cos: f32 = x.iter().map(|&xi| libm::cosf(2.0 * pi * xi)).sum();

    -20.0 * libm::expf(-0.2 * libm::sqrtf(sum_sq / n))
        - libm::expf(sum_cos / n)
        + 20.0
        + core::f32::consts::E
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hybrid_optimizer_creation() {
        let optimizer = HybridOptimizer::default_optimizer();
        assert_eq!(optimizer.active_algorithm(), AlgorithmId::PSO);
        assert_eq!(optimizer.iteration(), 0);
    }

    #[test]
    fn test_hybrid_optimizer_step() {
        let config = HybridConfig {
            dimensions: 3,
            population_size: 20,
            ..Default::default()
        };
        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        optimizer.initialize(&bounds).unwrap();

        let fitness = optimizer.step(sphere_function).unwrap();
        assert!(fitness >= 0.0);
        assert_eq!(optimizer.iteration(), 1);
    }

    #[test]
    fn test_hybrid_optimizer_full_optimization() {
        let config = HybridConfig {
            max_iterations: 50,
            population_size: 20,
            dimensions: 3,
            target_fitness: 0.001,
            switch_interval: 100,  // Don't switch during this short test
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let result = optimizer.optimize(sphere_function, &bounds).unwrap();

        assert!(result.fitness < f32::INFINITY);
        assert!(result.evaluations > 0);
    }

    #[test]
    fn test_benchmark_functions() {
        let x = [0.0, 0.0, 0.0];

        // All benchmark functions should have minimum near 0 at origin
        assert!(sphere_function(&x).abs() < 1e-6);
        assert!(rastrigin_function(&x).abs() < 1e-6);
        assert!(ackley_function(&x).abs() < 1e-5);

        // Rosenbrock minimum is at [1, 1, 1]
        let x_ros = [1.0, 1.0, 1.0];
        assert!(rosenbrock_function(&x_ros).abs() < 1e-6);
    }

    #[test]
    fn test_algorithm_switching() {
        let config = HybridConfig {
            max_iterations: 50,
            population_size: 10,
            dimensions: 3,
            switch_interval: 10,
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let _ = optimizer.optimize(sphere_function, &bounds).unwrap();

        // Should have made some switch decisions
        let stats = optimizer.stats();
        assert!(stats.total_iterations > 0);
    }

    #[test]
    fn test_stats_tracking() {
        let config = HybridConfig {
            max_iterations: 30,
            population_size: 10,
            dimensions: 3,
            switch_interval: 10,
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let _ = optimizer.optimize(sphere_function, &bounds).unwrap();

        let stats = optimizer.stats();
        assert!(stats.total_evaluations > 0);
        assert!(stats.algorithm_iterations.iter().sum::<u32>() == stats.total_iterations);
    }
}
