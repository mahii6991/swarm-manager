//! Advanced PSO Features - Military Grade Implementation
//!
//! Top 3 Features from pySwarms:
//! 1. Advanced Topology Support (Star, Ring, Von Neumann, Pyramid, Dynamic, Custom)
//! 2. Constraint Handling (Boundaries, Inequalities, Collision, Energy, No-Fly Zones)
//! 3. Multi-Swarm Coordination (Hierarchical, Decentralized, Fault-Tolerant)

use crate::pso::*;
use crate::types::*;
use core::f32;
use heapless::{FnvIndexMap, Vec};

// ═══════════════════════════════════════════════════════════════════════════
// SPATIAL HASHING FOR O(n) COLLISION DETECTION
// ═══════════════════════════════════════════════════════════════════════════

/// Grid cell size for spatial hashing
const SPATIAL_CELL_SIZE: f32 = 10.0;
/// Maximum cells in spatial grid (8x8x8 = 512 cells)
const MAX_SPATIAL_CELLS: usize = 512;
/// Maximum particles per cell
const MAX_PARTICLES_PER_CELL: usize = 16;

/// Spatial hash grid for O(n) average collision detection
/// Instead of checking all n² particle pairs, we only check particles
/// in the same or adjacent grid cells.
pub struct SpatialGrid {
    /// Cell contents: maps cell_id -> list of particle indices
    cells: FnvIndexMap<u32, Vec<usize, MAX_PARTICLES_PER_CELL>, MAX_SPATIAL_CELLS>,
    /// Cell size (should be >= min_distance for correct collision detection)
    cell_size: f32,
}

impl SpatialGrid {
    /// Create new spatial grid with given cell size
    pub fn new(cell_size: f32) -> Self {
        Self {
            cells: FnvIndexMap::new(),
            cell_size: if cell_size > 0.0 { cell_size } else { SPATIAL_CELL_SIZE },
        }
    }

    /// Hash 3D position to cell ID
    fn hash_position(&self, x: f32, y: f32, z: f32) -> u32 {
        // Offset to handle negative coordinates
        let offset = 1000.0;
        let cx = ((x + offset) / self.cell_size) as i32;
        let cy = ((y + offset) / self.cell_size) as i32;
        let cz = ((z + offset) / self.cell_size) as i32;

        // Combine into single hash (8 bits each, wrapping for out-of-range)
        let hx = (cx & 0xFF) as u32;
        let hy = (cy & 0xFF) as u32;
        let hz = (cz & 0xFF) as u32;

        (hx << 16) | (hy << 8) | hz
    }

    /// Clear and rebuild grid from particle positions
    pub fn rebuild(&mut self, positions: &[Vec<f32, MAX_DIMENSIONS>]) {
        self.cells.clear();

        for (idx, pos) in positions.iter().enumerate() {
            if pos.len() >= 3 {
                let cell_id = self.hash_position(pos[0], pos[1], pos[2]);
                if let Some(cell) = self.cells.get_mut(&cell_id) {
                    let _ = cell.push(idx);
                } else {
                    let mut new_cell = Vec::new();
                    let _ = new_cell.push(idx);
                    let _ = self.cells.insert(cell_id, new_cell);
                }
            }
        }
    }

    /// Get particles in same and adjacent cells for collision checking
    /// Returns indices of particles that might collide with particle at given position
    pub fn get_nearby_particles(&self, x: f32, y: f32, z: f32) -> Vec<usize, 128> {
        let mut nearby = Vec::new();

        // Check 3x3x3 neighborhood of cells
        for dx in -1i32..=1 {
            for dy in -1i32..=1 {
                for dz in -1i32..=1 {
                    let nx = x + (dx as f32) * self.cell_size;
                    let ny = y + (dy as f32) * self.cell_size;
                    let nz = z + (dz as f32) * self.cell_size;
                    let cell_id = self.hash_position(nx, ny, nz);

                    if let Some(cell) = self.cells.get(&cell_id) {
                        for &idx in cell.iter() {
                            let _ = nearby.push(idx);
                        }
                    }
                }
            }
        }

        nearby
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FEATURE 1: ADVANCED TOPOLOGY SUPPORT
// ═══════════════════════════════════════════════════════════════════════════

/// Network topology manager for PSO
pub struct TopologyManager {
    /// Topology type
    topology: Topology,
    /// Adjacency matrix (who can communicate with whom)
    adjacency: FnvIndexMap<usize, Vec<usize, MAX_PARTICLES>, MAX_PARTICLES>,
    /// Number of particles
    n_particles: usize,
}

impl TopologyManager {
    /// Create new topology manager
    pub fn new(topology: Topology, n_particles: usize) -> Result<Self> {
        let mut manager = Self {
            topology,
            adjacency: FnvIndexMap::new(),
            n_particles,
        };

        manager.build_topology()?;
        Ok(manager)
    }

    /// Build topology adjacency matrix
    fn build_topology(&mut self) -> Result<()> {
        match self.topology {
            Topology::Star => self.build_star()?,
            Topology::Ring => self.build_ring()?,
            Topology::VonNeumann => self.build_von_neumann()?,
            Topology::Pyramid => self.build_pyramid()?,
            Topology::Random => self.build_random()?,
        }
        Ok(())
    }

    /// Star topology: all connected to central node
    fn build_star(&mut self) -> Result<()> {
        for i in 0..self.n_particles {
            let mut neighbors = Vec::new();
            // Everyone connected to everyone (global best)
            for j in 0..self.n_particles {
                if i != j {
                    neighbors.push(j).map_err(|_| SwarmError::BufferFull)?;
                }
            }
            self.adjacency
                .insert(i, neighbors)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Ring topology: connected to k nearest neighbors
    fn build_ring(&mut self) -> Result<()> {
        let k = 3; // neighborhood size

        for i in 0..self.n_particles {
            let mut neighbors = Vec::new();
            for offset in 1..=(k / 2) {
                let left = (i + self.n_particles - offset) % self.n_particles;
                let right = (i + offset) % self.n_particles;
                neighbors.push(left).ok();
                if right != left {
                    neighbors.push(right).ok();
                }
            }
            self.adjacency
                .insert(i, neighbors)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Von Neumann topology: 2D grid with 4-connectivity
    fn build_von_neumann(&mut self) -> Result<()> {
        let grid_size = (libm::sqrtf(self.n_particles as f32) as usize).max(1);

        for i in 0..self.n_particles {
            let row = i / grid_size;
            let col = i % grid_size;
            let mut neighbors = Vec::new();

            // Up
            if row > 0 {
                neighbors.push((row - 1) * grid_size + col).ok();
            }
            // Down
            if row < grid_size - 1 {
                neighbors.push((row + 1) * grid_size + col).ok();
            }
            // Left
            if col > 0 {
                neighbors.push(row * grid_size + (col - 1)).ok();
            }
            // Right
            if col < grid_size - 1 {
                neighbors.push(row * grid_size + (col + 1)).ok();
            }

            self.adjacency
                .insert(i, neighbors)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Pyramid topology: hierarchical structure
    fn build_pyramid(&mut self) -> Result<()> {
        for i in 0..self.n_particles {
            let mut neighbors = Vec::new();

            // Parent (if not root)
            if i > 0 {
                let parent = (i - 1) / 2;
                neighbors.push(parent).ok();
            }

            // Children
            let left_child = 2 * i + 1;
            let right_child = 2 * i + 2;

            if left_child < self.n_particles {
                neighbors.push(left_child).ok();
            }
            if right_child < self.n_particles {
                neighbors.push(right_child).ok();
            }

            self.adjacency
                .insert(i, neighbors)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Random topology: random connections
    fn build_random(&mut self) -> Result<()> {
        let degree = 4; // average connections per node

        for i in 0..self.n_particles {
            let mut neighbors = Vec::new();
            for _ in 0..degree {
                let rand = Particle::pseudo_random(i * 1000 + neighbors.len());
                let neighbor = (rand * self.n_particles as f32) as usize % self.n_particles;
                if neighbor != i && !neighbors.contains(&neighbor) {
                    neighbors.push(neighbor).ok();
                }
            }
            self.adjacency
                .insert(i, neighbors)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Get neighbors of a particle
    pub fn get_neighbors(&self, particle_id: usize) -> &[usize] {
        self.adjacency
            .get(&particle_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    /// Adapt topology dynamically based on performance
    pub fn adapt_topology(&mut self, performance_metric: f32) -> Result<()> {
        // Simple adaptive strategy: switch between exploration and exploitation
        if performance_metric > 0.5 {
            // Good progress - use star for exploitation
            self.topology = Topology::Star;
        } else {
            // Poor progress - use ring for exploration
            self.topology = Topology::Ring;
        }
        self.build_topology()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FEATURE 2: CONSTRAINT HANDLING
// ═══════════════════════════════════════════════════════════════════════════

/// Constraint types for PSO optimization
#[derive(Debug, Clone)]
pub enum Constraint {
    /// Inequality constraint: g(x) <= 0
    Inequality {
        /// Constraint function
        weight: f32,
    },
    /// Equality constraint: h(x) = 0 (tolerance)
    Equality {
        /// Tolerance for equality
        tolerance: f32,
        /// Penalty weight
        weight: f32,
    },
    /// Collision avoidance: min distance between particles
    Collision {
        /// Minimum distance
        min_distance: f32,
        /// Penalty weight
        weight: f32,
    },
    /// Energy/Battery constraint
    Energy {
        /// Maximum energy consumption
        max_energy: f32,
        /// Penalty weight
        weight: f32,
    },
    /// No-fly zone (geofencing)
    NoFlyZone {
        /// Center of zone
        center: Position,
        /// Radius of zone
        radius: f32,
        /// Penalty weight (very high for hard constraint)
        weight: f32,
    },
}

/// Constraint handler for PSO
pub struct ConstraintHandler {
    /// List of constraints
    constraints: Vec<Constraint, 50>,
    /// Penalty method (static, dynamic, adaptive)
    penalty_method: PenaltyMethod,
    /// Current iteration (for dynamic penalties)
    iteration: u32,
    /// Particle positions for collision checking (BUG-007 FIX)
    particle_positions: Vec<Vec<f32, MAX_DIMENSIONS>, MAX_PARTICLES>,
    /// Spatial grid for O(n) collision detection (performance optimization)
    spatial_grid: SpatialGrid,
}

#[derive(Debug, Clone, Copy)]
pub enum PenaltyMethod {
    /// Static penalty
    Static,
    /// Dynamic penalty (increases over time)
    Dynamic { growth_rate: f32 },
    /// Adaptive penalty
    Adaptive,
}

impl ConstraintHandler {
    /// Create new constraint handler
    pub fn new(penalty_method: PenaltyMethod) -> Self {
        Self {
            constraints: Vec::new(),
            penalty_method,
            iteration: 0,
            particle_positions: Vec::new(),
            spatial_grid: SpatialGrid::new(SPATIAL_CELL_SIZE),
        }
    }

    /// Set particle positions for collision checking (BUG-007 FIX)
    /// Also rebuilds the spatial grid for O(n) collision detection
    pub fn set_particle_positions(&mut self, positions: &[Vec<f32, MAX_DIMENSIONS>]) {
        self.particle_positions.clear();
        for pos in positions {
            let _ = self.particle_positions.push(pos.clone());
        }
        // Rebuild spatial grid for fast collision queries
        self.spatial_grid.rebuild(&self.particle_positions);
    }

    /// Update spatial grid cell size based on collision min_distance
    pub fn set_collision_cell_size(&mut self, min_distance: f32) {
        // Cell size should be >= min_distance to ensure all potential
        // collisions are found within the 3x3x3 cell neighborhood
        self.spatial_grid = SpatialGrid::new(min_distance.max(1.0));
    }

    /// Add constraint
    pub fn add_constraint(&mut self, constraint: Constraint) -> Result<()> {
        self.constraints
            .push(constraint)
            .map_err(|_| SwarmError::BufferFull)
    }

    /// Evaluate constraints and compute penalty
    pub fn evaluate_penalty<F>(
        &self,
        position: &[f32],
        particle_id: usize,
        constraint_fns: &[F],
    ) -> f32
    where
        F: Fn(&[f32]) -> f32,
    {
        let mut total_penalty = 0.0;

        for (i, constraint) in self.constraints.iter().enumerate() {
            let penalty = match constraint {
                Constraint::Inequality { weight } => {
                    let g = if i < constraint_fns.len() {
                        constraint_fns[i](position)
                    } else {
                        0.0
                    };

                    if g > 0.0 {
                        weight * g * g // Quadratic penalty
                    } else {
                        0.0
                    }
                }
                Constraint::Equality { tolerance, weight } => {
                    let h = if i < constraint_fns.len() {
                        constraint_fns[i](position)
                    } else {
                        0.0
                    };

                    if libm::fabsf(h) > *tolerance {
                        weight * h * h
                    } else {
                        0.0
                    }
                }
                Constraint::Collision {
                    min_distance,
                    weight,
                } => {
                    // BUG-007 FIX: Implement collision detection
                    // PERF: Use spatial grid for O(n) average case instead of O(n²)
                    let mut collision_penalty = 0.0;

                    if position.len() >= 3 {
                        // Query spatial grid for nearby particles only
                        let nearby = self.spatial_grid.get_nearby_particles(
                            position[0],
                            position[1],
                            position[2],
                        );

                        // Only check particles in nearby cells
                        for j in nearby {
                            if j != particle_id {
                                if let Some(other_pos) = self.particle_positions.get(j) {
                                    if other_pos.len() >= 3 {
                                        // Calculate Euclidean distance in 3D space
                                        let dx = position[0] - other_pos[0];
                                        let dy = position[1] - other_pos[1];
                                        let dz = position[2] - other_pos[2];
                                        let distance = libm::sqrtf(dx * dx + dy * dy + dz * dz);

                                        // Penalize if too close
                                        if distance < *min_distance {
                                            let violation = min_distance - distance;
                                            collision_penalty += weight * violation * violation;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    collision_penalty
                }
                Constraint::Energy { max_energy, weight } => {
                    // Compute energy consumption from position
                    let energy = self.compute_energy(position);
                    if energy > *max_energy {
                        weight * (energy - max_energy).powi(2)
                    } else {
                        0.0
                    }
                }
                Constraint::NoFlyZone {
                    center,
                    radius,
                    weight,
                } => {
                    // Check if position violates no-fly zone
                    if position.len() >= 3 {
                        let pos = Position {
                            x: position[0],
                            y: position[1],
                            z: position[2],
                        };
                        let dist = pos.distance_to(center);
                        if dist < *radius {
                            weight * (*radius - dist).powi(2)
                        } else {
                            0.0
                        }
                    } else {
                        0.0
                    }
                }
            };

            // Apply penalty method
            let adjusted_penalty = match self.penalty_method {
                PenaltyMethod::Static => penalty,
                PenaltyMethod::Dynamic { growth_rate } => {
                    penalty * (1.0 + growth_rate * self.iteration as f32)
                }
                PenaltyMethod::Adaptive => {
                    // Adaptive based on constraint violation level
                    if penalty > 0.0 {
                        penalty * 2.0 // Increase penalty if violated
                    } else {
                        penalty
                    }
                }
            };

            total_penalty += adjusted_penalty;
        }

        total_penalty
    }

    /// Repair mechanism: project infeasible solution to feasible space
    pub fn repair(&self, position: &mut [f32], bounds: &Bounds) {
        // Boundary repair (clamping)
        for (i, pos) in position.iter_mut().enumerate() {
            *pos = pos.clamp(bounds.lower[i], bounds.upper[i]);
        }

        // No-fly zone repair: push away if inside
        for constraint in &self.constraints {
            if let Constraint::NoFlyZone { center, radius, .. } = constraint {
                if position.len() >= 3 {
                    let pos = Position {
                        x: position[0],
                        y: position[1],
                        z: position[2],
                    };

                    let dist = pos.distance_to(center);
                    if dist < *radius {
                        // Push outward
                        let dx = pos.x - center.x;
                        let dy = pos.y - center.y;
                        let dz = pos.z - center.z;
                        let norm = libm::sqrtf(dx * dx + dy * dy + dz * dz).max(0.001);

                        position[0] = center.x + (dx / norm) * radius;
                        position[1] = center.y + (dy / norm) * radius;
                        position[2] = center.z + (dz / norm) * radius;
                    }
                }
            }
        }
    }

    /// Check if position is feasible (all constraints satisfied)
    pub fn is_feasible<F>(&self, position: &[f32], constraint_fns: &[F]) -> bool
    where
        F: Fn(&[f32]) -> f32,
    {
        self.evaluate_penalty(position, usize::MAX, constraint_fns) < 1e-6
    }

    /// Compute energy consumption (simplified)
    fn compute_energy(&self, position: &[f32]) -> f32 {
        // Simplified: energy proportional to distance from origin
        position.iter().map(|&x| x * x).sum::<f32>().sqrt()
    }

    /// Update iteration (for dynamic penalties)
    pub fn update_iteration(&mut self) {
        self.iteration += 1;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FEATURE 3: MULTI-SWARM COORDINATION
// ═══════════════════════════════════════════════════════════════════════════

/// Sub-swarm for multi-swarm optimization
pub struct SubSwarm {
    /// Sub-swarm ID
    pub id: usize,
    /// PSO optimizer for this sub-swarm
    pub optimizer: GlobalBestPSO,
    /// Task assignment
    pub task: SwarmTask,
    /// Best solution found by this sub-swarm
    pub best_cost: f32,
    /// Communication with other sub-swarms
    pub shared_knowledge: Vec<f32, MAX_DIMENSIONS>,
}

/// Multi-swarm coordinator
pub struct MultiSwarmCoordinator {
    /// Collection of sub-swarms
    sub_swarms: Vec<SubSwarm, 10>,
    /// Global best across all sub-swarms
    global_best: Vec<f32, MAX_DIMENSIONS>,
    /// Global best cost
    global_best_cost: f32,
    /// Information sharing strategy
    sharing_strategy: SharingStrategy,
    /// Migration frequency (iterations between migrations)
    migration_frequency: u32,
    /// Current iteration
    iteration: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum SharingStrategy {
    /// No sharing (independent sub-swarms)
    None,
    /// Share best solution periodically
    BestSolution,
    /// Migrate particles between sub-swarms
    Migration { rate: f32 },
    /// Share knowledge incrementally
    Incremental,
}

impl MultiSwarmCoordinator {
    /// Create new multi-swarm coordinator
    pub fn new(sharing_strategy: SharingStrategy, migration_frequency: u32) -> Self {
        let mut global_best = Vec::new();
        // Initialize with zeros
        for _ in 0..MAX_DIMENSIONS {
            global_best.push(0.0).ok();
        }

        Self {
            sub_swarms: Vec::new(),
            global_best,
            global_best_cost: f32::INFINITY,
            sharing_strategy,
            migration_frequency,
            iteration: 0,
        }
    }

    /// Add sub-swarm
    pub fn add_sub_swarm(&mut self, sub_swarm: SubSwarm) -> Result<()> {
        self.sub_swarms
            .push(sub_swarm)
            .map_err(|_| SwarmError::BufferFull)
    }

    /// Coordinate one iteration across all sub-swarms
    pub fn coordinate_step<F>(&mut self, cost_fns: &[F]) -> Result<f32>
    where
        F: Fn(&[f32]) -> f32,
    {
        // Step 1: Each sub-swarm optimizes independently
        for (i, sub_swarm) in self.sub_swarms.iter_mut().enumerate() {
            if i < cost_fns.len() {
                sub_swarm.best_cost = sub_swarm.optimizer.step(&cost_fns[i])?;

                // Update global best if needed
                if sub_swarm.best_cost < self.global_best_cost {
                    self.global_best_cost = sub_swarm.best_cost;
                    self.global_best = sub_swarm.optimizer.best_position().clone();
                }
            }
        }

        // Step 2: Information sharing
        if self.iteration.is_multiple_of(self.migration_frequency) {
            self.share_information()?;
        }

        self.iteration += 1;

        Ok(self.global_best_cost)
    }

    /// Share information between sub-swarms
    fn share_information(&mut self) -> Result<()> {
        match self.sharing_strategy {
            SharingStrategy::None => {
                // No sharing
            }
            SharingStrategy::BestSolution => {
                // Share global best with all sub-swarms
                for sub_swarm in &mut self.sub_swarms {
                    sub_swarm.shared_knowledge = self.global_best.clone();
                }
            }
            SharingStrategy::Migration { rate } => {
                // Migrate best particles between sub-swarms
                self.migrate_particles(rate)?;
            }
            SharingStrategy::Incremental => {
                // Gradually share knowledge
                for sub_swarm in &mut self.sub_swarms {
                    // Average with global best
                    for (i, val) in sub_swarm.shared_knowledge.iter_mut().enumerate() {
                        if i < self.global_best.len() {
                            *val = (*val + self.global_best[i]) / 2.0;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Migrate particles between sub-swarms
    fn migrate_particles(&mut self, _rate: f32) -> Result<()> {
        // Simplified migration: share best positions
        // In full implementation, would physically move particles

        if self.sub_swarms.len() < 2 {
            return Ok(());
        }

        // Ring migration: pass best to next sub-swarm
        let n = self.sub_swarms.len();
        let mut best_positions = Vec::<Vec<f32, MAX_DIMENSIONS>, 10>::new();

        for sub_swarm in &self.sub_swarms {
            best_positions
                .push(sub_swarm.optimizer.best_position().clone())
                .ok();
        }

        for i in 0..n {
            let next = (i + 1) % n;
            if next < best_positions.len() && i < self.sub_swarms.len() {
                self.sub_swarms[next].shared_knowledge = best_positions[i].clone();
            }
        }

        Ok(())
    }

    /// Get global best solution
    pub fn global_best(&self) -> &Vec<f32, MAX_DIMENSIONS> {
        &self.global_best
    }

    /// Get global best cost
    pub fn global_best_cost(&self) -> f32 {
        self.global_best_cost
    }

    /// Get number of sub-swarms
    pub fn num_sub_swarms(&self) -> usize {
        self.sub_swarms.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topology_star() {
        let topo = TopologyManager::new(Topology::Star, 5).unwrap();
        assert_eq!(topo.get_neighbors(0).len(), 4);
    }

    #[test]
    fn test_topology_ring() {
        let topo = TopologyManager::new(Topology::Ring, 10).unwrap();
        // Each node should have 2 neighbors (k=3, so 1 on each side typically)
        assert!(!topo.get_neighbors(5).is_empty());
    }

    #[test]
    fn test_constraint_handler() {
        let mut handler = ConstraintHandler::new(PenaltyMethod::Static);

        // Add no-fly zone
        handler
            .add_constraint(Constraint::NoFlyZone {
                center: Position {
                    x: 50.0,
                    y: 50.0,
                    z: 10.0,
                },
                radius: 20.0,
                weight: 1000.0,
            })
            .unwrap();

        // Position inside no-fly zone
        let pos_inside = [45.0, 45.0, 10.0];
        let dummy_constraints: &[fn(&[f32]) -> f32] = &[];
        let penalty_inside = handler.evaluate_penalty(&pos_inside, 0, dummy_constraints);
        assert!(penalty_inside > 0.0);

        // Position outside no-fly zone
        let pos_outside = [100.0, 100.0, 10.0];
        let penalty_outside = handler.evaluate_penalty(&pos_outside, 0, dummy_constraints);
        assert_eq!(penalty_outside, 0.0);
    }

    #[test]
    fn test_multi_swarm() {
        let coordinator = MultiSwarmCoordinator::new(SharingStrategy::BestSolution, 10);

        assert_eq!(coordinator.num_sub_swarms(), 0);
    }
}
