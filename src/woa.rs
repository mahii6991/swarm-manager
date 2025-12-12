//! Whale Optimization Algorithm (WOA)
//!
//! A bio-inspired meta-heuristic optimization algorithm mimicking the bubble-net
//! hunting behavior of humpback whales.
//!
//! Features:
//! - Encircling prey (Exploitation)
//! - Bubble-net attacking (Exploitation)
//! - Search for prey (Exploration)
//!
//! Suitable for:
//! - 3D Trajectory planning
//! - Task allocation optimization
//! - Network routing optimization

use crate::types::{Position, Result, SwarmError};
use libm::{cosf, expf, fabsf};

/// Configuration for WOA
#[derive(Debug, Clone, Copy)]
pub struct WoaConfig {
    /// Maximum number of iterations
    pub max_iterations: u32,
    /// Number of whales (search agents)
    pub population_size: usize,
    /// Spiral shape parameter (b)
    pub spiral_param: f32,
}

impl Default for WoaConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            population_size: 30,
            spiral_param: 1.0,
        }
    }
}

/// A single whale (search agent)
#[derive(Debug, Clone, Copy)]
pub struct Whale {
    pub position: Position,
    pub fitness: f32,
}

/// Simple pseudo-random number generator for optimization algorithms
/// Uses Xorshift32 for efficiency
pub struct OptimizationRng {
    state: u32,
}

impl OptimizationRng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed as u32 | 1, // Ensure non-zero
        }
    }

    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Generate random f32 in [0.0, 1.0]
    pub fn next_f32(&mut self) -> f32 {
        (self.next_u32() as f32) / (u32::MAX as f32)
    }
}

/// Whale Optimization Algorithm Optimizer
pub struct WhaleOptimizer {
    config: WoaConfig,
    whales: heapless::Vec<Whale, 100>,
    best_whale: Option<Whale>,
    rng: OptimizationRng,
}

impl WhaleOptimizer {
    pub fn new(config: WoaConfig, seed: u64) -> Self {
        Self {
            config,
            whales: heapless::Vec::new(),
            best_whale: None,
            rng: OptimizationRng::new(seed),
        }
    }

    /// Initialize population randomly within bounds
    pub fn initialize(&mut self, min: Position, max: Position) -> Result<()> {
        self.whales.clear();
        for _ in 0..self.config.population_size {
            let x = min.x + self.rng.next_f32() * (max.x - min.x);
            let y = min.y + self.rng.next_f32() * (max.y - min.y);
            let z = min.z + self.rng.next_f32() * (max.z - min.z);

            self.whales
                .push(Whale {
                    position: Position { x, y, z },
                    fitness: f32::INFINITY, // Assuming minimization
                })
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Run one iteration of the optimization
    ///
    /// Cost function: f(Position) -> f32 (lower is better)
    pub fn step<F>(&mut self, iter: u32, cost_function: &F) -> Result<f32>
    where
        F: Fn(&Position) -> f32,
    {
        // 1. Calculate fitness and update best
        for whale in &mut self.whales {
            // Apply boundary checks here if needed (clamping)

            let fitness = cost_function(&whale.position);
            whale.fitness = fitness;

            if let Some(best) = &self.best_whale {
                if fitness < best.fitness {
                    self.best_whale = Some(*whale);
                }
            } else {
                self.best_whale = Some(*whale);
            }
        }

        let best_pos = self.best_whale.unwrap().position;

        // a decreases linearly from 2 to 0
        let a = 2.0 - 2.0 * (iter as f32 / self.config.max_iterations as f32);

        // 2. Update position of each whale
        for i in 0..self.whales.len() {
            let r1 = self.rng.next_f32();
            let r2 = self.rng.next_f32();

            let coeff_a = 2.0 * a * r1 - a;
            let coeff_c = 2.0 * r2;

            let p = self.rng.next_f32();

            let current_pos = self.whales[i].position;
            let mut new_pos = current_pos;

            if p < 0.5 {
                if fabsf(coeff_a) < 1.0 {
                    // Encircling prey (Update position towards best solution)
                    // D = |C * X_best - X(t)|
                    let dx = fabsf(coeff_c * best_pos.x - current_pos.x);
                    let dy = fabsf(coeff_c * best_pos.y - current_pos.y);
                    let dz = fabsf(coeff_c * best_pos.z - current_pos.z);

                    // X(t+1) = X_best - A * D
                    new_pos.x = best_pos.x - coeff_a * dx;
                    new_pos.y = best_pos.y - coeff_a * dy;
                    new_pos.z = best_pos.z - coeff_a * dz;
                } else {
                    // Search for prey (Exploration phase)
                    // Select a random whale
                    let rand_idx = (self.rng.next_u32() as usize) % self.whales.len();
                    let rand_whale_pos = self.whales[rand_idx].position;

                    // D = |C * X_rand - X(t)|
                    let dx = fabsf(coeff_c * rand_whale_pos.x - current_pos.x);
                    let dy = fabsf(coeff_c * rand_whale_pos.y - current_pos.y);
                    let dz = fabsf(coeff_c * rand_whale_pos.z - current_pos.z);

                    // X(t+1) = X_rand - A * D
                    new_pos.x = rand_whale_pos.x - coeff_a * dx;
                    new_pos.y = rand_whale_pos.y - coeff_a * dy;
                    new_pos.z = rand_whale_pos.z - coeff_a * dz;
                }
            } else {
                // Bubble-net attacking method (Spiral updating position)
                // D' = |X_best - X(t)|
                let dx = fabsf(best_pos.x - current_pos.x);
                let dy = fabsf(best_pos.y - current_pos.y);
                let dz = fabsf(best_pos.z - current_pos.z);

                let l = (self.rng.next_f32() * 2.0) - 1.0; // Random number in [-1, 1]
                let b = self.config.spiral_param;

                // X(t+1) = D' * e^(bl) * cos(2pi*l) + X_best
                let factor = expf(b * l) * cosf(2.0 * core::f32::consts::PI * l);

                new_pos.x = dx * factor + best_pos.x;
                new_pos.y = dy * factor + best_pos.y;
                new_pos.z = dz * factor + best_pos.z;
            }

            self.whales[i].position = new_pos;
        }

        Ok(self.best_whale.unwrap().fitness)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initialization() {
        let config = WoaConfig::default();
        let mut woa = WhaleOptimizer::new(config, 42);

        let min = Position {
            x: -10.0,
            y: -10.0,
            z: 0.0,
        };
        let max = Position {
            x: 10.0,
            y: 10.0,
            z: 20.0,
        };

        woa.initialize(min, max).unwrap();
        assert_eq!(woa.whales.len(), 30);
    }

    #[test]
    fn test_optimization_sphere() {
        // Minimize sphere function: f(x) = x^2 + y^2 + z^2
        let config = WoaConfig {
            max_iterations: 50,
            population_size: 20,
            spiral_param: 1.0,
        };
        let mut woa = WhaleOptimizer::new(config, 12345);

        let min = Position {
            x: -5.0,
            y: -5.0,
            z: -5.0,
        };
        let max = Position {
            x: 5.0,
            y: 5.0,
            z: 5.0,
        };

        woa.initialize(min, max).unwrap();

        let cost_fn = |p: &Position| -> f32 { p.x * p.x + p.y * p.y + p.z * p.z };

        for i in 0..50 {
            woa.step(i, &cost_fn).unwrap();
        }

        let best = woa.best_whale.unwrap();
        // Should be close to 0
        assert!(
            best.fitness < 0.1,
            "WOA failed to converge on sphere function. Cost: {}",
            best.fitness
        );
    }
}
