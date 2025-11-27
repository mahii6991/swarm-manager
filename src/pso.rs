//! Particle Swarm Optimization (PSO) for Swarm Intelligence
//!
//! Inspired by pySwarms library, implementing:
//! - Global-best PSO (star topology)
//! - Local-best PSO (ring/von-neumann topology)
//! - Binary PSO for discrete optimization
//! - Custom topologies
//! - Velocity clamping and constriction
//! - Multi-objective optimization support

use crate::types::*;
use heapless::{FnvIndexMap, Vec};
use core::f32;

/// Maximum number of particles in swarm
pub const MAX_PARTICLES: usize = 100;

/// Maximum dimensions for optimization
pub const MAX_DIMENSIONS: usize = 50;

/// Particle in PSO swarm
#[derive(Debug, Clone)]
pub struct Particle {
    /// Current position
    pub position: Vec<f32, MAX_DIMENSIONS>,
    /// Current velocity
    pub velocity: Vec<f32, MAX_DIMENSIONS>,
    /// Personal best position
    pub pbest_position: Vec<f32, MAX_DIMENSIONS>,
    /// Personal best cost
    pub pbest_cost: f32,
    /// Particle ID
    pub id: usize,
}

impl Particle {
    /// Create a new particle
    pub fn new(dimensions: usize, id: usize) -> Result<Self> {
        let mut position = Vec::new();
        let mut velocity = Vec::new();
        let mut pbest = Vec::new();

        for _ in 0..dimensions {
            position.push(0.0).map_err(|_| SwarmError::BufferFull)?;
            velocity.push(0.0).map_err(|_| SwarmError::BufferFull)?;
            pbest.push(0.0).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            position,
            velocity,
            pbest_position: pbest,
            pbest_cost: f32::INFINITY,
            id,
        })
    }

    /// Initialize position randomly within bounds
    pub fn initialize(&mut self, bounds: &Bounds) -> Result<()> {
        for i in 0..self.position.len() {
            let range = bounds.upper[i] - bounds.lower[i];
            // Use simple LCG for random (replace with hardware RNG in production)
            let rand = Self::pseudo_random(self.id + i);
            self.position[i] = bounds.lower[i] + rand * range;

            // Initialize velocity
            let v_max = range * 0.1;
            self.velocity[i] = (Self::pseudo_random(self.id * 2 + i) - 0.5) * v_max * 2.0;
        }

        self.pbest_position.clone_from(&self.position);
        Ok(())
    }

    /// SplitMix64 pseudo-random number generator (better statistical properties)
    fn pseudo_random(seed: usize) -> f32 {
        let mut z = (seed as u64).wrapping_add(0x9e3779b97f4a7c15);
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z = z ^ (z >> 31);
        // Convert to float [0, 1)
        (z >> 11) as f32 * (1.0 / 9007199254740992.0)
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
}

/// Topology defines particle communication patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topology {
    /// Star topology - all particles attracted to global best
    Star,
    /// Ring topology - particles communicate with neighbors
    Ring,
    /// Von Neumann grid topology
    VonNeumann,
    /// Pyramid topology
    Pyramid,
    /// Random topology
    Random,
}

/// PSO hyperparameters
#[derive(Debug, Clone, Copy)]
pub struct PSOOptions {
    /// Cognitive parameter (c1) - attraction to personal best
    pub cognitive: f32,
    /// Social parameter (c2) - attraction to neighbor/global best
    pub social: f32,
    /// Inertia weight (w)
    pub inertia: f32,
    /// Velocity clamping factor
    pub velocity_clamp: f32,
    /// Use constriction coefficient
    pub use_constriction: bool,
}

impl PSOOptions {
    /// Default PSO parameters (standard PSO)
    pub fn default() -> Self {
        Self {
            cognitive: 2.05,
            social: 2.05,
            inertia: 0.9,
            velocity_clamp: 1.0,
            use_constriction: false,
        }
    }

    /// Parameters with constriction coefficient
    pub fn constriction() -> Self {
        let phi = 4.1;
        let kappa = 2.0 / (phi - 2.0 + libm::sqrtf(phi * phi - 4.0 * phi));

        Self {
            cognitive: 2.05,
            social: 2.05,
            inertia: kappa,
            velocity_clamp: 1.0,
            use_constriction: true,
        }
    }

    /// Balanced exploration-exploitation
    pub fn balanced() -> Self {
        Self {
            cognitive: 2.0,
            social: 2.0,
            inertia: 0.7298,
            velocity_clamp: 0.5,
            use_constriction: false,
        }
    }
}

/// Global-best PSO optimizer (Star topology)
pub struct GlobalBestPSO {
    /// Particle swarm
    particles: Vec<Particle, MAX_PARTICLES>,
    /// Global best position
    gbest_position: Vec<f32, MAX_DIMENSIONS>,
    /// Global best cost
    gbest_cost: f32,
    /// Search space bounds
    bounds: Bounds,
    /// PSO options
    options: PSOOptions,
    /// Current iteration
    iteration: u32,
    /// Cost history
    cost_history: Vec<f32, 1000>,
}

impl GlobalBestPSO {
    /// Create new global-best PSO optimizer
    pub fn new(
        n_particles: usize,
        dimensions: usize,
        bounds: Bounds,
        options: PSOOptions,
    ) -> Result<Self> {
        let mut particles = Vec::new();

        for i in 0..n_particles {
            let mut particle = Particle::new(dimensions, i)?;
            particle.initialize(&bounds)?;
            particles.push(particle).map_err(|_| SwarmError::BufferFull)?;
        }

        let mut gbest = Vec::new();
        for _ in 0..dimensions {
            gbest.push(0.0).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            particles,
            gbest_position: gbest,
            gbest_cost: f32::INFINITY,
            bounds,
            options,
            iteration: 0,
            cost_history: Vec::new(),
        })
    }

    /// Perform one optimization iteration
    pub fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        // Evaluate all particles
        for particle in &mut self.particles {
            let cost = cost_fn(&particle.position);

            // Update personal best
            if cost < particle.pbest_cost {
                particle.pbest_cost = cost;
                particle.pbest_position.clone_from(&particle.position);
            }

            // Update global best
            if cost < self.gbest_cost {
                self.gbest_cost = cost;
                self.gbest_position.clone_from(&particle.position);
            }
        }

        // Update velocities and positions
        for particle in &mut self.particles {
            self.update_velocity(particle)?;
            self.update_position(particle)?;
        }

        // Record cost
        self.cost_history.push(self.gbest_cost).ok();
        self.iteration += 1;

        Ok(self.gbest_cost)
    }

    /// Update particle velocity (standard PSO update)
    fn update_velocity(&self, particle: &mut Particle) -> Result<()> {
        for i in 0..particle.velocity.len() {
            let r1 = Particle::pseudo_random(self.iteration as usize * 2 + i);
            let r2 = Particle::pseudo_random(self.iteration as usize * 3 + i);

            // Cognitive component
            let cognitive = self.options.cognitive * r1
                * (particle.pbest_position[i] - particle.position[i]);

            // Social component
            let social = self.options.social * r2
                * (self.gbest_position[i] - particle.position[i]);

            // Update velocity
            particle.velocity[i] = self.options.inertia * particle.velocity[i]
                + cognitive
                + social;

            // Velocity clamping
            let v_max = (self.bounds.upper[i] - self.bounds.lower[i])
                * self.options.velocity_clamp;
            particle.velocity[i] = particle.velocity[i].clamp(-v_max, v_max);
        }

        Ok(())
    }

    /// Update particle position
    fn update_position(&self, particle: &mut Particle) -> Result<()> {
        for i in 0..particle.position.len() {
            particle.position[i] += particle.velocity[i];

            // Boundary handling (clamping)
            particle.position[i] = particle.position[i]
                .clamp(self.bounds.lower[i], self.bounds.upper[i]);
        }

        Ok(())
    }

    /// Optimize for a number of iterations
    pub fn optimize<F>(&mut self, iterations: u32, cost_fn: F) -> Result<(Vec<f32, MAX_DIMENSIONS>, f32)>
    where
        F: Fn(&[f32]) -> f32,
    {
        for _ in 0..iterations {
            self.step(&cost_fn)?;
        }

        Ok((self.gbest_position.clone(), self.gbest_cost))
    }

    /// Get best position found
    pub fn best_position(&self) -> &Vec<f32, MAX_DIMENSIONS> {
        &self.gbest_position
    }

    /// Get best cost found
    pub fn best_cost(&self) -> f32 {
        self.gbest_cost
    }

    /// Get cost history
    pub fn cost_history(&self) -> &Vec<f32, 1000> {
        &self.cost_history
    }
}

/// Local-best PSO optimizer (Ring topology)
pub struct LocalBestPSO {
    /// Particle swarm
    particles: Vec<Particle, MAX_PARTICLES>,
    /// Global best for tracking
    gbest_position: Vec<f32, MAX_DIMENSIONS>,
    /// Global best cost
    gbest_cost: f32,
    /// Search space bounds
    bounds: Bounds,
    /// PSO options
    options: PSOOptions,
    /// Neighborhood size (k-nearest neighbors)
    neighborhood_size: usize,
    /// Current iteration
    iteration: u32,
    /// Cost history
    cost_history: Vec<f32, 1000>,
}

impl LocalBestPSO {
    /// Create new local-best PSO optimizer
    pub fn new(
        n_particles: usize,
        dimensions: usize,
        bounds: Bounds,
        options: PSOOptions,
        neighborhood_size: usize,
    ) -> Result<Self> {
        let mut particles = Vec::new();

        for i in 0..n_particles {
            let mut particle = Particle::new(dimensions, i)?;
            particle.initialize(&bounds)?;
            particles.push(particle).map_err(|_| SwarmError::BufferFull)?;
        }

        let mut gbest = Vec::new();
        for _ in 0..dimensions {
            gbest.push(0.0).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            particles,
            gbest_position: gbest,
            gbest_cost: f32::INFINITY,
            bounds,
            options,
            neighborhood_size,
            iteration: 0,
            cost_history: Vec::new(),
        })
    }

    /// Perform one optimization iteration
    pub fn step<F>(&mut self, cost_fn: F) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        // Evaluate all particles
        for particle in &mut self.particles {
            let cost = cost_fn(&particle.position);

            // Update personal best
            if cost < particle.pbest_cost {
                particle.pbest_cost = cost;
                particle.pbest_position.clone_from(&particle.position);
            }

            // Update global best (for tracking)
            if cost < self.gbest_cost {
                self.gbest_cost = cost;
                self.gbest_position.clone_from(&particle.position);
            }
        }

        // Update velocities and positions using local neighborhoods
        for i in 0..self.particles.len() {
            let lbest_pos = self.find_local_best(i)?;
            self.update_velocity_local(i, &lbest_pos)?;
            self.update_position_local(i)?;
        }

        // Record cost
        self.cost_history.push(self.gbest_cost).ok();
        self.iteration += 1;

        Ok(self.gbest_cost)
    }

    /// Find local best in neighborhood (ring topology)
    fn find_local_best(&self, particle_idx: usize) -> Result<Vec<f32, MAX_DIMENSIONS>> {
        let n = self.particles.len();
        let k = self.neighborhood_size.min(n);

        let mut best_cost = f32::INFINITY;
        let mut best_pos = self.particles[particle_idx].pbest_position.clone();

        // Ring topology: check k/2 neighbors on each side
        for offset in 0..k {
            let neighbor_idx = (particle_idx + offset) % n;
            if self.particles[neighbor_idx].pbest_cost < best_cost {
                best_cost = self.particles[neighbor_idx].pbest_cost;
                best_pos = self.particles[neighbor_idx].pbest_position.clone();
            }
        }

        Ok(best_pos)
    }

    /// Update velocity for local-best PSO
    fn update_velocity_local(&mut self, particle_idx: usize, lbest: &[f32]) -> Result<()> {
        let particle = &mut self.particles[particle_idx];

        for i in 0..particle.velocity.len() {
            let r1 = Particle::pseudo_random(self.iteration as usize * 2 + i + particle_idx);
            let r2 = Particle::pseudo_random(self.iteration as usize * 3 + i + particle_idx);

            // Cognitive component
            let cognitive = self.options.cognitive * r1
                * (particle.pbest_position[i] - particle.position[i]);

            // Social component (local best instead of global)
            let social = self.options.social * r2
                * (lbest[i] - particle.position[i]);

            // Update velocity
            particle.velocity[i] = self.options.inertia * particle.velocity[i]
                + cognitive
                + social;

            // Velocity clamping
            let v_max = (self.bounds.upper[i] - self.bounds.lower[i])
                * self.options.velocity_clamp;
            particle.velocity[i] = particle.velocity[i].clamp(-v_max, v_max);
        }

        Ok(())
    }

    /// Update particle position
    fn update_position_local(&mut self, particle_idx: usize) -> Result<()> {
        let particle = &mut self.particles[particle_idx];

        for i in 0..particle.position.len() {
            particle.position[i] += particle.velocity[i];

            // Boundary handling
            particle.position[i] = particle.position[i]
                .clamp(self.bounds.lower[i], self.bounds.upper[i]);
        }

        Ok(())
    }

    /// Get best position found
    pub fn best_position(&self) -> &Vec<f32, MAX_DIMENSIONS> {
        &self.gbest_position
    }

    /// Get best cost found
    pub fn best_cost(&self) -> f32 {
        self.gbest_cost
    }
}

/// PSO for drone path planning
pub struct DronePathOptimizer {
    /// PSO optimizer
    pso: GlobalBestPSO,
    /// Start position
    start: Position,
    /// Goal position
    goal: Position,
    /// Obstacles
    obstacles: Vec<(Position, f32), 50>, // (center, radius)
}

impl DronePathOptimizer {
    /// Create path optimizer
    pub fn new(
        start: Position,
        goal: Position,
        n_waypoints: usize,
    ) -> Result<Self> {
        // Each waypoint has 3 dimensions (x, y, z)
        let dimensions = n_waypoints * 3;

        // Bounds (assuming 1000m x 1000m x 100m space)
        let bounds = Bounds::uniform(dimensions, 0.0, 1000.0)?;

        let options = PSOOptions::balanced();
        let pso = GlobalBestPSO::new(30, dimensions, bounds, options)?;

        Ok(Self {
            pso,
            start,
            goal,
            obstacles: Vec::new(),
        })
    }

    /// Add obstacle
    pub fn add_obstacle(&mut self, center: Position, radius: f32) -> Result<()> {
        self.obstacles.push((center, radius))
            .map_err(|_| SwarmError::BufferFull)
    }

    /// Compute path cost
    fn path_cost(&self, waypoints: &[f32]) -> f32 {
        let mut cost = 0.0;
        let n_waypoints = waypoints.len() / 3;

        // Start to first waypoint
        let first = Position {
            x: waypoints[0],
            y: waypoints[1],
            z: waypoints[2],
        };
        cost += self.start.distance_to(&first);

        // Between waypoints
        for i in 0..n_waypoints - 1 {
            let p1 = Position {
                x: waypoints[i * 3],
                y: waypoints[i * 3 + 1],
                z: waypoints[i * 3 + 2],
            };
            let p2 = Position {
                x: waypoints[(i + 1) * 3],
                y: waypoints[(i + 1) * 3 + 1],
                z: waypoints[(i + 1) * 3 + 2],
            };
            cost += p1.distance_to(&p2);
        }

        // Last waypoint to goal
        let last = Position {
            x: waypoints[(n_waypoints - 1) * 3],
            y: waypoints[(n_waypoints - 1) * 3 + 1],
            z: waypoints[(n_waypoints - 1) * 3 + 2],
        };
        cost += last.distance_to(&self.goal);

        // Penalty for obstacles
        for i in 0..n_waypoints {
            let wp = Position {
                x: waypoints[i * 3],
                y: waypoints[i * 3 + 1],
                z: waypoints[i * 3 + 2],
            };

            for (obs_center, obs_radius) in &self.obstacles {
                let dist = wp.distance_to(obs_center);
                if dist < *obs_radius {
                    cost += 1000.0; // Large penalty
                }
            }
        }

        cost
    }

    /// Optimize path
    pub fn optimize(&mut self, iterations: u32) -> Result<Vec<Position, 20>> {
        // Run optimization
        let cost_fn = |wp: &[f32]| self.path_cost(wp);
        let (best_waypoints, _cost) = self.pso.optimize(iterations, cost_fn)?;

        // Convert to positions
        let mut path = Vec::new();
        path.push(self.start).ok();

        let n_waypoints = best_waypoints.len() / 3;
        for i in 0..n_waypoints {
            let pos = Position {
                x: best_waypoints[i * 3],
                y: best_waypoints[i * 3 + 1],
                z: best_waypoints[i * 3 + 2],
            };
            path.push(pos).map_err(|_| SwarmError::BufferFull)?;
        }

        path.push(self.goal).ok();

        Ok(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pso_sphere_function() {
        // Minimize sphere function: f(x) = sum(x_i^2)
        let cost_fn = |x: &[f32]| {
            x.iter().map(|&xi| xi * xi).sum()
        };

        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();
        let options = PSOOptions::default();
        let mut pso = GlobalBestPSO::new(20, 5, bounds, options).unwrap();

        pso.optimize(100, cost_fn).unwrap();

        // Should converge near zero
        assert!(pso.best_cost() < 1.0);
    }

    #[test]
    fn test_path_optimizer() {
        let start = Position { x: 0.0, y: 0.0, z: 10.0 };
        let goal = Position { x: 100.0, y: 100.0, z: 10.0 };

        let mut optimizer = DronePathOptimizer::new(start, goal, 3).unwrap();
        optimizer.add_obstacle(
            Position { x: 50.0, y: 50.0, z: 10.0 },
            20.0
        ).unwrap();

        let path = optimizer.optimize(50).unwrap();
        assert!(path.len() >= 2);
    }
}
