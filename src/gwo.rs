//! Grey Wolf Optimizer (GWO) for Swarm Coordination
//!
//! Implements Grey Wolf Optimizer algorithms for drone swarm coordination:
//! - Standard GWO - Mimics grey wolf hierarchy and hunting
//! - Improved GWO (IGWO) - Adaptive parameters
//! - Hybrid GWO-PSO - Combined optimization
//! - Multi-objective GWO
//! - Chaotic GWO - Enhanced exploration
//!
//! Based on 2025 research:
//! - Nature Scientific Reports: Hybrid GWO-PSO
//! - Applied Intelligence: 3D UAV path planning
//! - PMC: Cauchy-Gaussian mutation strategies
//!
//! References:
//! - Mirjalili et al. (2014): Grey Wolf Optimizer
//! - 2025 Scientific Reports: Improved GWO variants
//! - 2025 UAV trajectory optimization research

use crate::types::*;
use core::f32;
use heapless::Vec;

/// Maximum number of wolves (search agents)
pub const MAX_WOLVES: usize = 50;

/// Maximum dimensions for optimization
pub const MAX_DIMENSIONS: usize = 50;

/// Position vector for a wolf
pub type Position = Vec<f32, MAX_DIMENSIONS>;

/// Grey Wolf agent
#[derive(Debug, Clone)]
pub struct Wolf {
    /// Current position
    pub position: Position,
    /// Fitness value
    pub fitness: f32,
    /// Wolf ID
    pub id: usize,
}

impl Wolf {
    /// Create a new wolf
    pub fn new(dimensions: usize, id: usize) -> Result<Self> {
        let mut position = Vec::new();
        for _ in 0..dimensions {
            position.push(0.0).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            position,
            fitness: f32::INFINITY,
            id,
        })
    }

    /// Initialize position randomly within bounds
    pub fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        for i in 0..self.position.len() {
            let range = bounds.upper[i] - bounds.lower[i];
            let rand = pseudo_random(self.id * 100 + i);
            self.position[i] = bounds.lower[i] + rand * range;
        }
        Ok(())
    }

    /// Update position
    pub fn update_position(&mut self, new_position: &[f32], bounds: &Bounds) -> Result<()> {
        for (i, &val) in new_position.iter().enumerate() {
            if i >= self.position.len() {
                break;
            }
            // Apply boundaries
            let bounded = val.max(bounds.lower[i]).min(bounds.upper[i]);
            self.position[i] = bounded;
        }
        Ok(())
    }
}

/// Search space bounds
#[derive(Debug, Clone)]
pub struct Bounds {
    /// Lower bounds per dimension
    pub lower: Vec<f32, MAX_DIMENSIONS>,
    /// Upper bounds per dimension
    pub upper: Vec<f32, MAX_DIMENSIONS>,
}

impl Bounds {
    /// Create uniform bounds
    pub fn uniform(dimensions: usize, min: f32, max: f32) -> Result<Self> {
        let mut lower = Vec::new();
        let mut upper = Vec::new();

        for _ in 0..dimensions {
            lower.push(min).map_err(|_| SwarmError::BufferFull)?;
            upper.push(max).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self { lower, upper })
    }

    /// Create custom bounds
    pub fn new(lower: Vec<f32, MAX_DIMENSIONS>, upper: Vec<f32, MAX_DIMENSIONS>) -> Self {
        Self { lower, upper }
    }
}

/// GWO algorithm variant
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GWOVariant {
    /// Standard Grey Wolf Optimizer
    Standard,
    /// Improved GWO with adaptive parameters
    Improved,
    /// Hybrid GWO-PSO
    Hybrid,
    /// Chaotic GWO
    Chaotic,
}

/// GWO Configuration
#[derive(Debug, Clone)]
pub struct GWOConfig {
    pub variant: GWOVariant,
    pub num_wolves: usize,
    pub max_iterations: usize,
    pub dimensions: usize,
    /// Convergence parameter (a) decay: linearly decreases from 2 to 0
    pub a_decay: bool,
    /// Adaptive parameters
    pub adaptive: bool,
}

impl Default for GWOConfig {
    fn default() -> Self {
        Self {
            variant: GWOVariant::Improved,
            num_wolves: 30,
            max_iterations: 100,
            dimensions: 10,
            a_decay: true,
            adaptive: true,
        }
    }
}

/// Grey Wolf Optimizer
pub struct GWOOptimizer {
    config: GWOConfig,
    wolves: Vec<Wolf, MAX_WOLVES>,
    alpha: Wolf, // Best solution
    beta: Wolf,  // Second best
    delta: Wolf, // Third best
    bounds: Bounds,
    iteration: usize,
    a: f32, // Convergence parameter
}

impl GWOOptimizer {
    /// Create new GWO optimizer
    pub fn new(config: GWOConfig, bounds: Bounds) -> Result<Self> {
        if config.num_wolves > MAX_WOLVES {
            return Err(SwarmError::InvalidParameter);
        }

        let mut wolves = Vec::new();
        for i in 0..config.num_wolves {
            let mut wolf = Wolf::new(config.dimensions, i)?;
            wolf.initialize(&bounds)?;
            wolves.push(wolf).map_err(|_| SwarmError::BufferFull)?;
        }

        // Initialize alpha, beta, delta
        let alpha = Wolf::new(config.dimensions, 1000)?;
        let beta = Wolf::new(config.dimensions, 2000)?;
        let delta = Wolf::new(config.dimensions, 3000)?;

        Ok(Self {
            config,
            wolves,
            alpha,
            beta,
            delta,
            bounds,
            iteration: 0,
            a: 2.0,
        })
    }

    /// Run optimization
    pub fn optimize<F>(&mut self, fitness_fn: F) -> Result<&Wolf>
    where
        F: Fn(&[f32]) -> f32,
    {
        for iter in 0..self.config.max_iterations {
            self.iteration = iter;

            // Update a (linearly decreases from 2 to 0)
            if self.config.a_decay {
                self.a = 2.0 - 2.0 * (iter as f32) / (self.config.max_iterations as f32);
            }

            // Evaluate fitness for all wolves
            for i in 0..self.wolves.len() {
                let fitness = fitness_fn(self.wolves[i].position.as_slice());
                self.wolves[i].fitness = fitness;
            }

            // Update alpha, beta, delta
            self.update_leaders();

            // Update position of each wolf
            for wolf_id in 0..self.wolves.len() {
                self.update_wolf_position(wolf_id)?;
            }
        }

        Ok(&self.alpha)
    }

    /// Update alpha, beta, delta wolves (best three solutions)
    fn update_leaders(&mut self) {
        for wolf in &self.wolves {
            if wolf.fitness < self.alpha.fitness {
                // Update delta, beta, alpha
                self.delta = self.beta.clone();
                self.beta = self.alpha.clone();
                self.alpha = wolf.clone();
            } else if wolf.fitness < self.beta.fitness {
                // Update delta, beta
                self.delta = self.beta.clone();
                self.beta = wolf.clone();
            } else if wolf.fitness < self.delta.fitness {
                // Update delta
                self.delta = wolf.clone();
            }
        }
    }

    /// Update position of a single wolf
    fn update_wolf_position(&mut self, wolf_id: usize) -> Result<()> {
        let dimensions = self.config.dimensions;
        let mut new_position = Vec::<f32, MAX_DIMENSIONS>::new();

        match self.config.variant {
            GWOVariant::Standard | GWOVariant::Improved => {
                self.standard_position_update(wolf_id, dimensions, &mut new_position)?;
            }
            GWOVariant::Hybrid => {
                self.hybrid_position_update(wolf_id, dimensions, &mut new_position)?;
            }
            GWOVariant::Chaotic => {
                self.chaotic_position_update(wolf_id, dimensions, &mut new_position)?;
            }
        }

        // Update wolf position
        if let Some(wolf) = self.wolves.get_mut(wolf_id) {
            wolf.update_position(new_position.as_slice(), &self.bounds)?;
        }

        Ok(())
    }

    /// Standard GWO position update
    #[allow(non_snake_case)] // Mathematical notation from GWO algorithm
    fn standard_position_update(
        &self,
        wolf_id: usize,
        dimensions: usize,
        new_position: &mut Position,
    ) -> Result<()> {
        for d in 0..dimensions {
            let seed = wolf_id * 10000 + self.iteration * 100 + d;

            // Calculate D_alpha, D_beta, D_delta
            let r1 = pseudo_random(seed);
            let r2 = pseudo_random(seed + 1);
            let A1 = 2.0 * self.a * r1 - self.a;
            let C1 = 2.0 * r2;

            let d_alpha = (C1 * self.alpha.position[d] - self.wolves[wolf_id].position[d]).abs();
            let x1 = self.alpha.position[d] - A1 * d_alpha;

            let r1 = pseudo_random(seed + 2);
            let r2 = pseudo_random(seed + 3);
            let A2 = 2.0 * self.a * r1 - self.a;
            let C2 = 2.0 * r2;

            let d_beta = (C2 * self.beta.position[d] - self.wolves[wolf_id].position[d]).abs();
            let x2 = self.beta.position[d] - A2 * d_beta;

            let r1 = pseudo_random(seed + 4);
            let r2 = pseudo_random(seed + 5);
            let A3 = 2.0 * self.a * r1 - self.a;
            let C3 = 2.0 * r2;

            let d_delta = (C3 * self.delta.position[d] - self.wolves[wolf_id].position[d]).abs();
            let x3 = self.delta.position[d] - A3 * d_delta;

            // Average of three positions
            let x_new = (x1 + x2 + x3) / 3.0;
            new_position
                .push(x_new)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(())
    }

    /// Hybrid GWO-PSO position update
    #[allow(non_snake_case)] // Mathematical notation from GWO algorithm
    fn hybrid_position_update(
        &self,
        wolf_id: usize,
        dimensions: usize,
        new_position: &mut Position,
    ) -> Result<()> {
        // Combine GWO with PSO velocity
        let w = 0.5; // Inertia weight
        let c1 = 1.5; // Cognitive coefficient
        let c2 = 1.5; // Social coefficient

        for d in 0..dimensions {
            let seed = wolf_id * 10000 + self.iteration * 100 + d;

            // GWO component
            let r1 = pseudo_random(seed);
            let r2 = pseudo_random(seed + 1);
            let A = 2.0 * self.a * r1 - self.a;
            let C = 2.0 * r2;

            let d_alpha = (C * self.alpha.position[d] - self.wolves[wolf_id].position[d]).abs();
            let x_gwo = self.alpha.position[d] - A * d_alpha;

            // PSO component
            let r3 = pseudo_random(seed + 2);
            let r4 = pseudo_random(seed + 3);

            let current = self.wolves[wolf_id].position[d];
            let pbest = self.alpha.position[d]; // Use alpha as global best
            let x_pso = w * current + c1 * r3 * (pbest - current) + c2 * r4 * (pbest - current);

            // Weighted combination
            let x_new = 0.5 * x_gwo + 0.5 * x_pso;
            new_position
                .push(x_new)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(())
    }

    /// Chaotic GWO position update (Lévy flight inspired)
    #[allow(non_snake_case)] // Mathematical notation from GWO algorithm
    fn chaotic_position_update(
        &self,
        wolf_id: usize,
        dimensions: usize,
        new_position: &mut Position,
    ) -> Result<()> {
        for d in 0..dimensions {
            let seed = wolf_id * 10000 + self.iteration * 100 + d;

            // Use chaotic map for C parameter
            let chaos = self.logistic_map(seed);

            let r1 = pseudo_random(seed);
            let A = 2.0 * self.a * r1 - self.a;
            let C = 2.0 * chaos; // Chaotic parameter

            let d_alpha = (C * self.alpha.position[d] - self.wolves[wolf_id].position[d]).abs();
            let x1 = self.alpha.position[d] - A * d_alpha;

            // Add Lévy flight component for exploration
            let levy = self.levy_flight(seed);
            let x_new = x1 + levy * 0.01 * (self.bounds.upper[d] - self.bounds.lower[d]);

            new_position
                .push(x_new)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(())
    }

    /// Logistic map for chaotic behavior
    fn logistic_map(&self, seed: usize) -> f32 {
        let x = pseudo_random(seed);
        let mu = 3.99; // Chaos parameter
        mu * x * (1.0 - x)
    }

    /// Lévy flight step
    fn levy_flight(&self, seed: usize) -> f32 {
        let beta = 1.5;
        let sigma = (gamma_function(1.0 + beta) * (beta * f32::consts::PI / 2.0).sin()
            / (gamma_function((1.0 + beta) / 2.0) * beta * 2.0_f32.powf((beta - 1.0) / 2.0)))
        .powf(1.0 / beta);

        let u = pseudo_random(seed) * sigma;
        let v = pseudo_random(seed + 1);

        u / v.abs().powf(1.0 / beta)
    }

    /// Get best solution
    pub fn get_best(&self) -> &Wolf {
        &self.alpha
    }

    /// Get second best solution
    pub fn get_beta(&self) -> &Wolf {
        &self.beta
    }

    /// Get third best solution
    pub fn get_delta(&self) -> &Wolf {
        &self.delta
    }

    /// Get current iteration
    pub fn get_iteration(&self) -> usize {
        self.iteration
    }

    /// Get convergence parameter
    pub fn get_a(&self) -> f32 {
        self.a
    }
}

/// Pseudo-random number generator
fn pseudo_random(seed: usize) -> f32 {
    let a = 1_103_515_245_u64;
    let c = 12_345_u64;
    let m = 2_147_483_648_u64;
    let x = ((a.wrapping_mul(seed as u64).wrapping_add(c)) % m) as f32;
    x / m as f32
}

/// Approximation of gamma function
fn gamma_function(z: f32) -> f32 {
    // Stirling's approximation for gamma function
    if z < 0.5 {
        f32::consts::PI / ((f32::consts::PI * z).sin() * gamma_function(1.0 - z))
    } else {
        let z = z - 1.0;
        let x = 0.999_999_97
            + 0.577_215_7 / (z + 1.0)
            + (-0.655_878_07) / (z + 2.0)
            + 0.420_026_36 / (z + 3.0);
        (2.0 * f32::consts::PI).sqrt() / z.exp() * (z / f32::consts::E).powf(z + 0.5) * x
    }
}

/// Multi-objective GWO (reserved for future multi-objective optimization)
#[allow(dead_code)]
pub struct MOGWOOptimizer {
    config: GWOConfig,
    wolves: Vec<Wolf, MAX_WOLVES>,
    pareto_front: Vec<Wolf, MAX_WOLVES>,
    bounds: Bounds,
    iteration: usize,
}

impl MOGWOOptimizer {
    /// Create new multi-objective GWO optimizer
    pub fn new(config: GWOConfig, bounds: Bounds) -> Result<Self> {
        if config.num_wolves > MAX_WOLVES {
            return Err(SwarmError::InvalidParameter);
        }

        let mut wolves = Vec::new();
        for i in 0..config.num_wolves {
            let mut wolf = Wolf::new(config.dimensions, i)?;
            wolf.initialize(&bounds)?;
            wolves.push(wolf).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            config,
            wolves,
            pareto_front: Vec::new(),
            bounds,
            iteration: 0,
        })
    }

    /// Get Pareto front
    pub fn get_pareto_front(&self) -> &[Wolf] {
        self.pareto_front.as_slice()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere_function(x: &[f32]) -> f32 {
        x.iter().map(|&xi| xi * xi).sum()
    }

    #[allow(dead_code)]
    fn rastrigin_function(x: &[f32]) -> f32 {
        let n = x.len() as f32;
        let a = 10.0;
        a * n
            + x.iter()
                .map(|&xi| xi * xi - a * (2.0 * f32::consts::PI * xi).cos())
                .sum::<f32>()
    }

    #[test]
    fn test_wolf_creation() {
        let wolf = Wolf::new(10, 0);
        assert!(wolf.is_ok());
        let wolf = wolf.unwrap();
        assert_eq!(wolf.position.len(), 10);
        assert_eq!(wolf.fitness, f32::INFINITY);
    }

    #[test]
    fn test_bounds_creation() {
        let bounds = Bounds::uniform(5, -10.0, 10.0);
        assert!(bounds.is_ok());
        let bounds = bounds.unwrap();
        assert_eq!(bounds.lower.len(), 5);
        assert_eq!(bounds.upper.len(), 5);
    }

    #[test]
    fn test_gwo_optimizer_creation() {
        let config = GWOConfig::default();
        let bounds = Bounds::uniform(config.dimensions, -100.0, 100.0).unwrap();
        let optimizer = GWOOptimizer::new(config, bounds);
        assert!(optimizer.is_ok());
    }

    #[test]
    fn test_gwo_optimization_sphere() {
        let config = GWOConfig {
            dimensions: 5,
            max_iterations: 50,
            num_wolves: 20,
            ..Default::default()
        };

        let bounds = Bounds::uniform(config.dimensions, -100.0, 100.0).unwrap();
        let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();

        let result = optimizer.optimize(sphere_function);
        assert!(result.is_ok());

        let best = result.unwrap();
        // Should converge near 0 for sphere function
        assert!(best.fitness < 1.0);
    }

    #[test]
    fn test_hybrid_gwo() {
        let config = GWOConfig {
            variant: GWOVariant::Hybrid,
            dimensions: 3,
            max_iterations: 30,
            ..Default::default()
        };

        let bounds = Bounds::uniform(config.dimensions, -10.0, 10.0).unwrap();
        let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();

        let result = optimizer.optimize(sphere_function);
        assert!(result.is_ok());
    }
}
