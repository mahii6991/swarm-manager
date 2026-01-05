//! Unified Meta-Heuristic Trait for Algorithm Switching
//!
//! Provides a common interface for all optimization algorithms (PSO, ACO, GWO, WOA)
//! enabling dynamic algorithm selection via reinforcement learning.
//!
//! This module is feature-gated under `hybrid_metaheuristic`.

use crate::types::{Result, SwarmError};
use heapless::Vec;

/// Maximum solution dimensions across all algorithms
pub const MAX_SOLUTION_DIM: usize = 50;

/// Maximum population size
pub const MAX_POPULATION: usize = 128;

/// Solution vector type (fixed-size for no_std)
pub type Solution = Vec<f32, MAX_SOLUTION_DIM>;

/// Algorithm identifier for selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmId {
    /// Particle Swarm Optimization
    PSO,
    /// Ant Colony Optimization
    ACO,
    /// Grey Wolf Optimizer
    GWO,
    /// Whale Optimization Algorithm
    WOA,
}

/// Parameter preset for algorithms
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterPreset {
    /// Conservative: slower but more stable convergence
    Conservative,
    /// Balanced: default parameters
    Balanced,
    /// Aggressive: faster convergence, risk of local optima
    Aggressive,
    /// Exploratory: high diversity, slower convergence
    Exploratory,
}

impl Default for ParameterPreset {
    fn default() -> Self {
        Self::Balanced
    }
}

/// Algorithm-specific performance metadata
#[derive(Debug, Clone, Copy, Default)]
pub struct AlgorithmMetadata {
    /// Population diversity measure (0.0-1.0)
    pub diversity: f32,
    /// Convergence rate (fitness improvement per iteration)
    pub convergence_rate: f32,
    /// Exploration vs exploitation ratio (0.0 = pure exploit, 1.0 = pure explore)
    pub exploration_ratio: f32,
    /// Number of function evaluations performed
    pub evaluations: u32,
    /// Iterations since last improvement
    pub stagnation_count: u32,
}

/// Optimization result with solution and metadata
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// Best solution found
    pub solution: Solution,
    /// Fitness/cost of best solution
    pub fitness: f32,
    /// Number of function evaluations used
    pub evaluations: u32,
    /// Whether convergence was achieved
    pub converged: bool,
    /// Algorithm that produced this result
    pub algorithm: AlgorithmId,
    /// Performance metadata
    pub metadata: AlgorithmMetadata,
}

/// Search space bounds
#[derive(Debug, Clone)]
pub struct Bounds {
    /// Lower bounds per dimension
    pub lower: Vec<f32, MAX_SOLUTION_DIM>,
    /// Upper bounds per dimension
    pub upper: Vec<f32, MAX_SOLUTION_DIM>,
}

impl Bounds {
    /// Create uniform bounds for all dimensions
    pub fn uniform(dimensions: usize, min: f32, max: f32) -> Result<Self> {
        let mut lower = Vec::new();
        let mut upper = Vec::new();

        for _ in 0..dimensions {
            lower.push(min).map_err(|_| SwarmError::BufferFull)?;
            upper.push(max).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self { lower, upper })
    }

    /// Get number of dimensions
    pub fn dimensions(&self) -> usize {
        self.lower.len()
    }
}

/// Unified meta-heuristic optimizer trait
///
/// All optimization algorithms implement this trait to enable
/// dynamic algorithm selection and switching.
pub trait MetaHeuristic {
    /// Get the algorithm identifier
    fn algorithm_id(&self) -> AlgorithmId;

    /// Initialize/reset the optimizer with given bounds
    fn initialize(&mut self, bounds: &Bounds) -> Result<()>;

    /// Perform one optimization step/iteration
    ///
    /// Returns the current best fitness value
    fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32;

    /// Get current best solution
    fn best_solution(&self) -> &[f32];

    /// Get current best fitness value
    fn best_fitness(&self) -> f32;

    /// Get algorithm performance metadata
    fn get_metadata(&self) -> AlgorithmMetadata;

    /// Reset optimizer to initial state
    fn reset(&mut self) -> Result<()>;

    /// Apply parameter preset
    fn apply_preset(&mut self, preset: ParameterPreset);
}

// ============================================================================
// PSO Adapter
// ============================================================================

use crate::pso::{Bounds as PsoBounds, GlobalBestPSO, PSOOptions, MAX_DIMENSIONS, MAX_PARTICLES};
use heapless::Vec as HVec;

/// Adapter wrapping GlobalBestPSO to implement MetaHeuristic trait
pub struct PsoAdapter {
    inner: Option<GlobalBestPSO>,
    bounds: Option<Bounds>,
    preset: ParameterPreset,
    n_particles: usize,
    dimensions: usize,
    metadata: AlgorithmMetadata,
    prev_fitness: f32,
}

impl PsoAdapter {
    /// Create a new PSO adapter
    pub fn new(n_particles: usize, dimensions: usize) -> Self {
        Self {
            inner: None,
            bounds: None,
            preset: ParameterPreset::Balanced,
            n_particles: n_particles.min(MAX_PARTICLES),
            dimensions: dimensions.min(MAX_DIMENSIONS),
            metadata: AlgorithmMetadata::default(),
            prev_fitness: f32::INFINITY,
        }
    }

    fn get_options(&self) -> PSOOptions {
        match self.preset {
            ParameterPreset::Conservative => PSOOptions {
                cognitive: 1.5,
                social: 1.5,
                inertia: 0.9,
                velocity_clamp: 0.3,
                use_constriction: false,
            },
            ParameterPreset::Balanced => PSOOptions::balanced(),
            ParameterPreset::Aggressive => PSOOptions {
                cognitive: 2.5,
                social: 2.5,
                inertia: 0.5,
                velocity_clamp: 1.0,
                use_constriction: true,
            },
            ParameterPreset::Exploratory => PSOOptions {
                cognitive: 2.8,
                social: 1.2,
                inertia: 0.95,
                velocity_clamp: 0.8,
                use_constriction: false,
            },
        }
    }

    fn convert_bounds(bounds: &Bounds) -> Result<PsoBounds> {
        let mut pso_lower: HVec<f32, MAX_DIMENSIONS> = HVec::new();
        let mut pso_upper: HVec<f32, MAX_DIMENSIONS> = HVec::new();

        for i in 0..bounds.lower.len() {
            pso_lower.push(bounds.lower[i]).map_err(|_| SwarmError::BufferFull)?;
            pso_upper.push(bounds.upper[i]).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(PsoBounds {
            lower: pso_lower,
            upper: pso_upper,
        })
    }
}

impl MetaHeuristic for PsoAdapter {
    fn algorithm_id(&self) -> AlgorithmId {
        AlgorithmId::PSO
    }

    fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        let pso_bounds = Self::convert_bounds(bounds)?;
        let options = self.get_options();

        self.inner = Some(GlobalBestPSO::new(
            self.n_particles,
            self.dimensions,
            pso_bounds,
            options,
        )?);
        self.bounds = Some(bounds.clone());
        self.metadata = AlgorithmMetadata::default();
        self.prev_fitness = f32::INFINITY;

        Ok(())
    }

    fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        let pso = self.inner.as_mut().ok_or(SwarmError::InvalidParameter)?;
        let fitness = pso.step(cost_fn)?;

        // Update metadata
        self.metadata.evaluations += self.n_particles as u32;

        let improvement = self.prev_fitness - fitness;
        if improvement > 1e-8 {
            self.metadata.convergence_rate = improvement;
            self.metadata.stagnation_count = 0;
        } else {
            self.metadata.stagnation_count += 1;
        }
        self.prev_fitness = fitness;

        // Estimate exploration ratio based on inertia
        let options = self.get_options();
        self.metadata.exploration_ratio = options.inertia;

        Ok(fitness)
    }

    fn best_solution(&self) -> &[f32] {
        self.inner
            .as_ref()
            .map(|p| p.best_position().as_slice())
            .unwrap_or(&[])
    }

    fn best_fitness(&self) -> f32 {
        self.inner
            .as_ref()
            .map(|p| p.best_cost())
            .unwrap_or(f32::INFINITY)
    }

    fn get_metadata(&self) -> AlgorithmMetadata {
        self.metadata
    }

    fn reset(&mut self) -> Result<()> {
        if let Some(bounds) = self.bounds.clone() {
            self.initialize(&bounds)?;
        }
        Ok(())
    }

    fn apply_preset(&mut self, preset: ParameterPreset) {
        self.preset = preset;
        // Re-initialize if already initialized
        if let Some(bounds) = self.bounds.clone() {
            let _ = self.initialize(&bounds);
        }
    }
}

// ============================================================================
// GWO Adapter
// ============================================================================

use crate::gwo::{Bounds as GwoBounds, GWOConfig, GWOOptimizer, GWOVariant, MAX_DIMENSIONS as GWO_MAX_DIM};

/// Adapter wrapping GWOOptimizer to implement MetaHeuristic trait
pub struct GwoAdapter {
    inner: Option<GWOOptimizer>,
    bounds: Option<Bounds>,
    preset: ParameterPreset,
    config: GWOConfig,
    metadata: AlgorithmMetadata,
    best_solution_cache: Solution,
    best_fitness_cache: f32,
    prev_fitness: f32,
}

impl GwoAdapter {
    /// Create a new GWO adapter
    pub fn new(num_wolves: usize, dimensions: usize) -> Self {
        Self {
            inner: None,
            bounds: None,
            preset: ParameterPreset::Balanced,
            config: GWOConfig {
                variant: GWOVariant::Improved,
                num_wolves,
                max_iterations: 1000,
                dimensions,
                a_decay: true,
                adaptive: true,
            },
            metadata: AlgorithmMetadata::default(),
            best_solution_cache: Vec::new(),
            best_fitness_cache: f32::INFINITY,
            prev_fitness: f32::INFINITY,
        }
    }

    fn convert_bounds(bounds: &Bounds) -> Result<GwoBounds> {
        let mut gwo_lower: HVec<f32, GWO_MAX_DIM> = HVec::new();
        let mut gwo_upper: HVec<f32, GWO_MAX_DIM> = HVec::new();

        for i in 0..bounds.lower.len() {
            gwo_lower.push(bounds.lower[i]).map_err(|_| SwarmError::BufferFull)?;
            gwo_upper.push(bounds.upper[i]).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(GwoBounds {
            lower: gwo_lower,
            upper: gwo_upper,
        })
    }
}

impl MetaHeuristic for GwoAdapter {
    fn algorithm_id(&self) -> AlgorithmId {
        AlgorithmId::GWO
    }

    fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        let gwo_bounds = Self::convert_bounds(bounds)?;

        // Update config based on preset
        let config = match self.preset {
            ParameterPreset::Conservative => GWOConfig {
                variant: GWOVariant::Standard,
                a_decay: true,
                adaptive: false,
                ..self.config
            },
            ParameterPreset::Balanced => GWOConfig {
                variant: GWOVariant::Improved,
                a_decay: true,
                adaptive: true,
                ..self.config
            },
            ParameterPreset::Aggressive => GWOConfig {
                variant: GWOVariant::Hybrid,
                a_decay: true,
                adaptive: true,
                ..self.config
            },
            ParameterPreset::Exploratory => GWOConfig {
                variant: GWOVariant::Chaotic,
                a_decay: false,
                adaptive: true,
                ..self.config
            },
        };

        self.config = config;
        self.inner = Some(GWOOptimizer::new(self.config.clone(), gwo_bounds)?);
        self.bounds = Some(bounds.clone());
        self.metadata = AlgorithmMetadata::default();
        self.best_solution_cache.clear();
        self.best_fitness_cache = f32::INFINITY;
        self.prev_fitness = f32::INFINITY;

        Ok(())
    }

    fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        // Check we have bounds (means initialized)
        let bounds = self.bounds.as_ref().ok_or(SwarmError::InvalidParameter)?;

        // GWO's optimize runs all iterations, but we want single step
        // We'll run optimize with max_iterations=1
        let mut single_step_config = self.config.clone();
        single_step_config.max_iterations = 1;

        // Create temporary optimizer for single step
        let gwo_bounds = Self::convert_bounds(bounds)?;
        let mut temp_gwo = GWOOptimizer::new(single_step_config, gwo_bounds)?;
        let result = temp_gwo.optimize(&cost_fn)?;

        // Cache the result (copy values before moving temp_gwo)
        let fitness = result.fitness;
        self.best_solution_cache.clear();
        for &val in result.position.iter() {
            self.best_solution_cache.push(val).ok();
        }
        self.best_fitness_cache = fitness;

        // Update metadata
        self.metadata.evaluations += self.config.num_wolves as u32;

        let improvement = self.prev_fitness - fitness;
        if improvement > 1e-8 {
            self.metadata.convergence_rate = improvement;
            self.metadata.stagnation_count = 0;
        } else {
            self.metadata.stagnation_count += 1;
        }
        self.prev_fitness = fitness;

        // Update inner optimizer reference
        self.inner = Some(temp_gwo);

        Ok(fitness)
    }

    fn best_solution(&self) -> &[f32] {
        self.best_solution_cache.as_slice()
    }

    fn best_fitness(&self) -> f32 {
        self.best_fitness_cache
    }

    fn get_metadata(&self) -> AlgorithmMetadata {
        self.metadata
    }

    fn reset(&mut self) -> Result<()> {
        if let Some(bounds) = self.bounds.clone() {
            self.initialize(&bounds)?;
        }
        Ok(())
    }

    fn apply_preset(&mut self, preset: ParameterPreset) {
        self.preset = preset;
        if let Some(bounds) = self.bounds.clone() {
            let _ = self.initialize(&bounds);
        }
    }
}

// ============================================================================
// WOA Adapter
// ============================================================================

use crate::types::Position;
use crate::woa::{WhaleOptimizer, WoaConfig};

/// Adapter wrapping WhaleOptimizer to implement MetaHeuristic trait
pub struct WoaAdapter {
    inner: Option<WhaleOptimizer>,
    bounds: Option<Bounds>,
    preset: ParameterPreset,
    config: WoaConfig,
    metadata: AlgorithmMetadata,
    best_solution_cache: Solution,
    best_fitness_cache: f32,
    prev_fitness: f32,
    iteration: u32,
    seed: u64,
}

impl WoaAdapter {
    /// Create a new WOA adapter
    pub fn new(population_size: usize, seed: u64) -> Self {
        Self {
            inner: None,
            bounds: None,
            preset: ParameterPreset::Balanced,
            config: WoaConfig {
                max_iterations: 1000,
                population_size,
                spiral_param: 1.0,
            },
            metadata: AlgorithmMetadata::default(),
            best_solution_cache: Vec::new(),
            best_fitness_cache: f32::INFINITY,
            prev_fitness: f32::INFINITY,
            iteration: 0,
            seed,
        }
    }
}

impl MetaHeuristic for WoaAdapter {
    fn algorithm_id(&self) -> AlgorithmId {
        AlgorithmId::WOA
    }

    fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        // WOA uses 3D Position, so we only support 3 dimensions
        if bounds.dimensions() < 3 {
            return Err(SwarmError::InvalidParameter);
        }

        let config = match self.preset {
            ParameterPreset::Conservative => WoaConfig {
                spiral_param: 0.5,
                ..self.config
            },
            ParameterPreset::Balanced => WoaConfig {
                spiral_param: 1.0,
                ..self.config
            },
            ParameterPreset::Aggressive => WoaConfig {
                spiral_param: 1.5,
                ..self.config
            },
            ParameterPreset::Exploratory => WoaConfig {
                spiral_param: 2.0,
                ..self.config
            },
        };

        self.config = config;
        let mut optimizer = WhaleOptimizer::new(self.config, self.seed);

        let min_pos = Position {
            x: bounds.lower[0],
            y: bounds.lower[1],
            z: bounds.lower[2],
        };
        let max_pos = Position {
            x: bounds.upper[0],
            y: bounds.upper[1],
            z: bounds.upper[2],
        };

        optimizer.initialize(min_pos, max_pos)?;

        self.inner = Some(optimizer);
        self.bounds = Some(bounds.clone());
        self.metadata = AlgorithmMetadata::default();
        self.best_solution_cache.clear();
        self.best_fitness_cache = f32::INFINITY;
        self.prev_fitness = f32::INFINITY;
        self.iteration = 0;

        Ok(())
    }

    fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        let woa = self.inner.as_mut().ok_or(SwarmError::InvalidParameter)?;

        // Wrap the cost function to work with Position
        let pos_cost_fn = |pos: &Position| -> f32 {
            let arr = [pos.x, pos.y, pos.z];
            cost_fn(&arr)
        };

        let fitness = woa.step(self.iteration, &pos_cost_fn)?;
        self.iteration += 1;

        // Update cache - WOA doesn't expose best directly, so use fitness
        self.best_fitness_cache = fitness;

        // Update metadata
        self.metadata.evaluations += self.config.population_size as u32;

        let improvement = self.prev_fitness - fitness;
        if improvement > 1e-8 {
            self.metadata.convergence_rate = improvement;
            self.metadata.stagnation_count = 0;
        } else {
            self.metadata.stagnation_count += 1;
        }
        self.prev_fitness = fitness;

        // Estimate exploration ratio based on iteration progress
        let progress = self.iteration as f32 / self.config.max_iterations as f32;
        self.metadata.exploration_ratio = 1.0 - progress; // Decreases over time

        Ok(fitness)
    }

    fn best_solution(&self) -> &[f32] {
        self.best_solution_cache.as_slice()
    }

    fn best_fitness(&self) -> f32 {
        self.best_fitness_cache
    }

    fn get_metadata(&self) -> AlgorithmMetadata {
        self.metadata
    }

    fn reset(&mut self) -> Result<()> {
        self.seed += 1; // Change seed for variety
        if let Some(bounds) = self.bounds.clone() {
            self.initialize(&bounds)?;
        }
        Ok(())
    }

    fn apply_preset(&mut self, preset: ParameterPreset) {
        self.preset = preset;
        if let Some(bounds) = self.bounds.clone() {
            let _ = self.initialize(&bounds);
        }
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Calculate population diversity from solutions
pub fn calculate_diversity(solutions: &[&[f32]]) -> f32 {
    if solutions.len() < 2 || solutions[0].is_empty() {
        return 0.0;
    }

    let n = solutions.len();
    let dims = solutions[0].len();

    // Calculate centroid
    let mut centroid = [0.0f32; MAX_SOLUTION_DIM];
    for sol in solutions {
        for (i, &val) in sol.iter().enumerate() {
            if i < MAX_SOLUTION_DIM {
                centroid[i] += val / n as f32;
            }
        }
    }

    // Calculate average distance from centroid
    let mut total_dist = 0.0f32;
    for sol in solutions {
        let mut dist_sq = 0.0f32;
        for (i, &val) in sol.iter().enumerate() {
            if i < dims {
                let diff = val - centroid[i];
                dist_sq += diff * diff;
            }
        }
        total_dist += libm::sqrtf(dist_sq);
    }

    total_dist / n as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere_function(x: &[f32]) -> f32 {
        x.iter().map(|&xi| xi * xi).sum()
    }

    #[test]
    fn test_pso_adapter_basic() {
        let mut pso = PsoAdapter::new(20, 5);
        let bounds = Bounds::uniform(5, -5.0, 5.0).unwrap();

        pso.initialize(&bounds).unwrap();

        for _ in 0..10 {
            let fitness = pso.step(sphere_function).unwrap();
            assert!(fitness >= 0.0);
        }

        assert!(pso.best_fitness() < f32::INFINITY);
        assert_eq!(pso.algorithm_id(), AlgorithmId::PSO);
    }

    #[test]
    fn test_gwo_adapter_basic() {
        let mut gwo = GwoAdapter::new(20, 5);
        let bounds = Bounds::uniform(5, -5.0, 5.0).unwrap();

        gwo.initialize(&bounds).unwrap();

        let fitness = gwo.step(sphere_function).unwrap();
        assert!(fitness >= 0.0);
        assert_eq!(gwo.algorithm_id(), AlgorithmId::GWO);
    }

    #[test]
    fn test_preset_application() {
        let mut pso = PsoAdapter::new(20, 5);
        pso.apply_preset(ParameterPreset::Aggressive);
        assert_eq!(pso.preset, ParameterPreset::Aggressive);
    }

    #[test]
    fn test_bounds_creation() {
        let bounds = Bounds::uniform(10, -1.0, 1.0).unwrap();
        assert_eq!(bounds.dimensions(), 10);
        assert_eq!(bounds.lower[0], -1.0);
        assert_eq!(bounds.upper[0], 1.0);
    }
}
