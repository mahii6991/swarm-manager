//! Swarm coordination and collective behavior
//!
//! Implements high-level swarm intelligence:
//! - Formation control
//! - Task allocation
//! - Collision avoidance
//! - Collective decision making
//! - Emergent behavior

use crate::types::*;
use crate::consensus::{ConsensusEngine, SwarmCommand};
use crate::network::MeshNetwork;
use crate::federated::FederatedCoordinator;
use heapless::{FnvIndexMap, Vec};

/// Swarm formation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Formation {
    /// Random distribution
    Random,
    /// Grid formation
    Grid { spacing: u32 },
    /// Line formation
    Line { spacing: u32 },
    /// Circle formation
    Circle { radius: u32 },
    /// V-formation (like birds)
    VFormation { spacing: u32 },
    /// Custom formation
    Custom,
}

/// Swarm behavior mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BehaviorMode {
    /// Exploration mode
    Exploration,
    /// Search and rescue
    SearchRescue,
    /// Surveillance
    Surveillance,
    /// Delivery
    Delivery,
    /// Collaborative mapping
    Mapping,
    /// Emergency response
    Emergency,
}

/// Main swarm controller integrating all subsystems
pub struct SwarmController {
    /// This drone's state
    local_state: DroneState,
    /// Known states of other drones
    swarm_states: FnvIndexMap<u64, DroneState, 100>,
    /// Current formation
    formation: Formation,
    /// Behavior mode
    behavior: BehaviorMode,
    /// Task queue
    tasks: Vec<SwarmTask, 100>,
    /// Target position (if any)
    target_position: Option<Position>,
}

impl SwarmController {
    /// Create a new swarm controller
    pub fn new(drone_id: DroneId, initial_position: Position) -> Self {
        Self {
            local_state: DroneState {
                id: drone_id,
                position: initial_position,
                velocity: Velocity {
                    vx: 0.0,
                    vy: 0.0,
                    vz: 0.0,
                },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            },
            swarm_states: FnvIndexMap::new(),
            formation: Formation::Random,
            behavior: BehaviorMode::Exploration,
            tasks: Vec::new(),
            target_position: None,
        }
    }

    /// Update local state
    pub fn update_state(
        &mut self,
        position: Position,
        velocity: Velocity,
        battery: u8,
        status: MissionStatus,
    ) {
        self.local_state.position = position;
        self.local_state.velocity = velocity;
        self.local_state.battery = battery;
        self.local_state.status = status;
        self.local_state.timestamp = Self::get_time();
    }

    /// Update state of another drone
    pub fn update_peer_state(&mut self, state: DroneState) -> Result<()> {
        self.swarm_states
            .insert(state.id.as_u64(), state)
            .map_err(|_| SwarmError::ResourceExhausted)?;
        Ok(())
    }

    /// Get local state
    pub fn local_state(&self) -> &DroneState {
        &self.local_state
    }

    /// Set formation
    pub fn set_formation(&mut self, formation: Formation) {
        self.formation = formation;
    }

    /// Set behavior mode
    pub fn set_behavior(&mut self, behavior: BehaviorMode) {
        self.behavior = behavior;
    }

    /// Calculate desired position based on formation
    pub fn compute_formation_position(&self) -> Position {
        match self.formation {
            Formation::Random => self.local_state.position,
            Formation::Grid { spacing } => self.compute_grid_position(spacing),
            Formation::Line { spacing } => self.compute_line_position(spacing),
            Formation::Circle { radius } => self.compute_circle_position(radius),
            Formation::VFormation { spacing } => self.compute_v_formation_position(spacing),
            Formation::Custom => self.local_state.position,
        }
    }

    /// Compute grid formation position
    fn compute_grid_position(&self, spacing: u32) -> Position {
        let id = self.local_state.id.as_u64();
        let grid_size = (libm::sqrtf(self.swarm_states.len() as f32) as u64) + 1;
        let row = id / grid_size;
        let col = id % grid_size;

        Position {
            x: col as f32 * spacing as f32,
            y: row as f32 * spacing as f32,
            z: self.local_state.position.z,
        }
    }

    /// Compute line formation position
    fn compute_line_position(&self, spacing: u32) -> Position {
        let id = self.local_state.id.as_u64();
        Position {
            x: id as f32 * spacing as f32,
            y: 0.0,
            z: self.local_state.position.z,
        }
    }

    /// Compute circle formation position
    fn compute_circle_position(&self, radius: u32) -> Position {
        let id = self.local_state.id.as_u64();
        let total = self.swarm_states.len() as f32 + 1.0;
        let angle = (id as f32 / total) * 2.0 * core::f32::consts::PI;

        Position {
            x: libm::cosf(angle) * radius as f32,
            y: libm::sinf(angle) * radius as f32,
            z: self.local_state.position.z,
        }
    }

    /// Compute V-formation position
    fn compute_v_formation_position(&self, spacing: u32) -> Position {
        let id = self.local_state.id.as_u64();
        if id == 0 {
            // Leader at front
            Position {
                x: 0.0,
                y: 0.0,
                z: self.local_state.position.z,
            }
        } else {
            let side = if id % 2 == 0 { -1.0 } else { 1.0 };
            let row = (id + 1) / 2;
            Position {
                x: side * row as f32 * spacing as f32,
                y: -(row as f32 * spacing as f32),
                z: self.local_state.position.z,
            }
        }
    }

    /// Collision avoidance using artificial potential fields
    pub fn compute_collision_avoidance(&self) -> Velocity {
        const REPULSION_DISTANCE: f32 = 10.0; // meters
        const REPULSION_STRENGTH: f32 = 5.0;

        let mut avoidance = Velocity {
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };

        // Compute repulsion from nearby drones
        for peer_state in self.swarm_states.values() {
            let distance = self.local_state.position.distance_to(&peer_state.position);

            if distance < REPULSION_DISTANCE && distance > 0.01 {
                // Repulsion force (inverse square law)
                let force = REPULSION_STRENGTH / (distance * distance);

                // Direction away from peer
                let dx = self.local_state.position.x - peer_state.position.x;
                let dy = self.local_state.position.y - peer_state.position.y;
                let dz = self.local_state.position.z - peer_state.position.z;

                // Normalize and apply force
                let norm = libm::sqrtf(dx * dx + dy * dy + dz * dz);
                avoidance.vx += (dx / norm) * force;
                avoidance.vy += (dy / norm) * force;
                avoidance.vz += (dz / norm) * force;
            }
        }

        avoidance
    }

    /// Compute desired velocity to reach target
    pub fn compute_target_velocity(&self, target: Position, max_speed: f32) -> Velocity {
        let dx = target.x - self.local_state.position.x;
        let dy = target.y - self.local_state.position.y;
        let dz = target.z - self.local_state.position.z;

        let distance = libm::sqrtf(dx * dx + dy * dy + dz * dz);

        if distance < 0.01 {
            return Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            };
        }

        // Proportional control
        let speed = libm::fminf(distance * 0.5, max_speed);

        Velocity {
            vx: (dx / distance) * speed,
            vy: (dy / distance) * speed,
            vz: (dz / distance) * speed,
        }
    }

    /// Combine velocities for final control output
    pub fn compute_control_velocity(&self, max_speed: f32) -> Velocity {
        // Get formation position
        let formation_pos = self.compute_formation_position();

        // Attraction to formation position
        let formation_vel = self.compute_target_velocity(formation_pos, max_speed);

        // Collision avoidance
        let avoidance_vel = self.compute_collision_avoidance();

        // Target tracking (if target exists)
        let target_vel = if let Some(target) = self.target_position {
            self.compute_target_velocity(target, max_speed)
        } else {
            Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            }
        };

        // Weighted combination
        const FORMATION_WEIGHT: f32 = 0.4;
        const AVOIDANCE_WEIGHT: f32 = 0.4;
        const TARGET_WEIGHT: f32 = 0.2;

        let mut final_vel = Velocity {
            vx: (formation_vel.vx * FORMATION_WEIGHT
                + avoidance_vel.vx * AVOIDANCE_WEIGHT
                + target_vel.vx * TARGET_WEIGHT).clamp(-100.0, 100.0),
            vy: (formation_vel.vy * FORMATION_WEIGHT
                + avoidance_vel.vy * AVOIDANCE_WEIGHT
                + target_vel.vy * TARGET_WEIGHT).clamp(-100.0, 100.0),
            vz: (formation_vel.vz * FORMATION_WEIGHT
                + avoidance_vel.vz * AVOIDANCE_WEIGHT
                + target_vel.vz * TARGET_WEIGHT).clamp(-100.0, 100.0),
        };

        // Limit to max speed
        let speed = libm::sqrtf(
            final_vel.vx * final_vel.vx + final_vel.vy * final_vel.vy + final_vel.vz * final_vel.vz,
        );
        if speed > max_speed {
            let scale = max_speed / speed;
            final_vel.vx *= scale;
            final_vel.vy *= scale;
            final_vel.vz *= scale;
        }

        final_vel
    }

    /// Add task to queue
    pub fn add_task(&mut self, task: SwarmTask) -> Result<()> {
        self.tasks
            .push(task)
            .map_err(|_| SwarmError::ResourceExhausted)
    }

    /// Get next task
    pub fn next_task(&mut self) -> Option<SwarmTask> {
        if self.tasks.is_empty() {
            None
        } else {
            Some(self.tasks.swap_remove(0))
        }
    }

    /// Set target position
    pub fn set_target(&mut self, target: Option<Position>) {
        self.target_position = target;
    }

    /// Get swarm size
    pub fn swarm_size(&self) -> usize {
        self.swarm_states.len() + 1 // +1 for self
    }

    /// Get swarm centroid (center of mass)
    pub fn swarm_centroid(&self) -> Position {
        let mut sum_x = self.local_state.position.x;
        let mut sum_y = self.local_state.position.y;
        let mut sum_z = self.local_state.position.z;
        let count = self.swarm_states.len() + 1;

        for state in self.swarm_states.values() {
            sum_x += state.position.x;
            sum_y += state.position.y;
            sum_z += state.position.z;
        }

        Position {
            x: sum_x / count as f32,
            y: sum_y / count as f32,
            z: sum_z / count as f32,
        }
    }

    /// Check if swarm is in formation
    pub fn is_in_formation(&self, tolerance: f32) -> bool {
        for state in self.swarm_states.values() {
            // Each drone should be near its formation position
            // This is a simplified check
            let distance = state.position.distance_to(&self.compute_formation_position());
            if distance > tolerance {
                return false;
            }
        }
        true
    }

    /// Get time (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

/// Task allocator for distributing work among swarm
pub struct TaskAllocator {
    /// Available tasks
    tasks: Vec<SwarmTask, 100>,
    /// Task assignments
    assignments: FnvIndexMap<u64, u64, 100>, // task_id -> drone_id
}

impl TaskAllocator {
    /// Create a new task allocator
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            assignments: FnvIndexMap::new(),
        }
    }

    /// Add task
    pub fn add_task(&mut self, task: SwarmTask) -> Result<()> {
        self.tasks
            .push(task)
            .map_err(|_| SwarmError::ResourceExhausted)
    }

    /// Allocate tasks to drones (greedy nearest-neighbor)
    pub fn allocate_tasks(&mut self, drone_states: &[DroneState]) -> Result<()> {
        // Sort tasks by priority
        self.tasks.sort_by(|a, b| b.priority.cmp(&a.priority));

        // Allocate each task to nearest available drone
        for task in &mut self.tasks {
            if !task.completed {
                let nearest = self.find_nearest_drone(&task.target, drone_states);
                if let Some(drone_id) = nearest {
                    task.assigned_drones.clear();
                    task.assigned_drones.push(drone_id).ok();
                    self.assignments.insert(task.task_id, drone_id.as_u64()).ok();
                }
            }
        }

        Ok(())
    }

    /// Find nearest drone to a position
    fn find_nearest_drone(&self, target: &Position, drones: &[DroneState]) -> Option<DroneId> {
        let mut nearest: Option<(DroneId, f32)> = None;

        for drone in drones {
            let distance = drone.position.distance_to(target);
            if let Some((_, min_dist)) = nearest {
                if distance < min_dist {
                    nearest = Some((drone.id, distance));
                }
            } else {
                nearest = Some((drone.id, distance));
            }
        }

        nearest.map(|(id, _)| id)
    }

    /// Get task assignment for drone
    pub fn get_assignment(&self, drone_id: DroneId) -> Option<&SwarmTask> {
        for task in &self.tasks {
            if task.assigned_drones.contains(&drone_id) && !task.completed {
                return Some(task);
            }
        }
        None
    }
}

impl Default for TaskAllocator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_swarm_controller() {
        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        let controller = SwarmController::new(DroneId::new(1), pos);
        assert_eq!(controller.swarm_size(), 1);
    }

    #[test]
    fn test_formation_position() {
        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        let mut controller = SwarmController::new(DroneId::new(0), pos);
        controller.set_formation(Formation::Line { spacing: 10 });

        let formation_pos = controller.compute_formation_position();
        assert_eq!(formation_pos.x, 0.0);
    }

    #[test]
    fn test_collision_avoidance() {
        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        let controller = SwarmController::new(DroneId::new(1), pos);

        let avoidance = controller.compute_collision_avoidance();
        // Should be zero with no nearby drones
        assert_eq!(avoidance.vx, 0.0);
    }
}
