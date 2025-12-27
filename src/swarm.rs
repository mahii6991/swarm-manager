//! Swarm coordination and collective behavior
//!
//! Implements high-level swarm intelligence:
//! - Formation control
//! - Task allocation
//! - Collision avoidance
//! - Collective decision making
//! - Emergent behavior

// Consensus, federated, and network types available for integration
use crate::types::*;
use crate::collision_avoidance::{CollisionAvoidance, AvoidanceConfig};
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
    swarm_states: FnvIndexMap<u64, DroneState, 128>,
    /// Current formation
    formation: Formation,
    /// Behavior mode
    behavior: BehaviorMode,
    /// Task queue
    tasks: Vec<SwarmTask, 100>,
    /// Target position (if any)
    target_position: Option<Position>,
    /// Collision avoidance system
    collision_avoidance: CollisionAvoidance,
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
            collision_avoidance: CollisionAvoidance::new(AvoidanceConfig::default()),
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
            let side = if id.is_multiple_of(2) { -1.0 } else { 1.0 };
            let row = id.div_ceil(2);
            Position {
                x: side * row as f32 * spacing as f32,
                y: -(row as f32 * spacing as f32),
                z: self.local_state.position.z,
            }
        }
    }

    /// Update collision avoidance obstacles from swarm state
    fn update_avoidance_obstacles(&mut self) {
        self.collision_avoidance.clear_obstacles();
        for state in self.swarm_states.values() {
            let _ = self.collision_avoidance.add_obstacle(
                [state.position.x, state.position.y, -state.position.z],
                [state.velocity.vx, state.velocity.vy, -state.velocity.vz],
                0.5 // Default drone radius, could be configurable
            );
        }
    }

    /// Compute collision avoidance velocity (deprecated, wraps new system)
    /// Returns escape velocity if in danger, otherwise zero
    pub fn compute_collision_avoidance(&mut self) -> Velocity {
        self.update_avoidance_obstacles();
        
        let current_pos = [
            self.local_state.position.x,
            self.local_state.position.y,
            -self.local_state.position.z
        ];
        
        let safe_vel = self.collision_avoidance.compute_safe_velocity(
            current_pos,
            [0.0, 0.0, 0.0] // Zero desired velocity
        );

        Velocity {
            vx: safe_vel[0],
            vy: safe_vel[1],
            vz: -safe_vel[2],
        }
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
    pub fn compute_control_velocity(&mut self, max_speed: f32) -> Velocity {
        // Update obstacles first
        self.update_avoidance_obstacles();

        // Get formation position
        let formation_pos = self.compute_formation_position();

        // Attraction to formation position
        let formation_vel = self.compute_target_velocity(formation_pos, max_speed);

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

        // Combine desired velocities
        // Prioritize formation and target
        let desired_vx = (formation_vel.vx + target_vel.vx).clamp(-max_speed, max_speed);
        let desired_vy = (formation_vel.vy + target_vel.vy).clamp(-max_speed, max_speed);
        let desired_vz = (formation_vel.vz + target_vel.vz).clamp(-max_speed, max_speed);

        let current_pos = [
            self.local_state.position.x,
            self.local_state.position.y,
            -self.local_state.position.z
        ];

        // Apply advanced collision avoidance
        let safe_vel_array = self.collision_avoidance.compute_safe_velocity(
            current_pos,
            [desired_vx, desired_vy, -desired_vz]
        );

        let mut final_vel = Velocity {
            vx: safe_vel_array[0],
            vy: safe_vel_array[1],
            vz: -safe_vel_array[2],
        };

        // Limit to max speed (safety check)
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
            let distance = state
                .position
                .distance_to(&self.compute_formation_position());
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

// Re-export TaskAllocator which was moved to task_allocation module
pub use crate::task_allocation::TaskAllocator;

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
        let mut controller = SwarmController::new(DroneId::new(1), pos);

        let avoidance = controller.compute_collision_avoidance();
        // Should be zero with no nearby drones
        assert_eq!(avoidance.vx, 0.0);
    }
}