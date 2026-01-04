//! Task allocation logic for the swarm

use crate::types::*;
use heapless::{FnvIndexMap, Vec};

/// Task allocator for distributing work among swarm
pub struct TaskAllocator {
    /// Available tasks
    tasks: Vec<SwarmTask, 100>,
    /// Task assignments
    assignments: FnvIndexMap<u64, u64, 128>, // task_id -> drone_id
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
    ///
    /// Returns Ok(()) if all tasks were successfully allocated,
    /// or Err if resource exhaustion prevented allocation.
    pub fn allocate_tasks(&mut self, drone_states: &[DroneState]) -> Result<()> {
        // Sort tasks by priority
        self.tasks.sort_by(|a, b| b.priority.cmp(&a.priority));

        // Allocate each task to nearest available drone
        // Use index-based iteration to avoid borrow checker issues
        let task_count = self.tasks.len();
        for i in 0..task_count {
            if !self.tasks[i].completed {
                let target = self.tasks[i].target;
                let task_id = self.tasks[i].task_id;
                let nearest = self.find_nearest_drone(&target, drone_states);
                if let Some(drone_id) = nearest {
                    self.tasks[i].assigned_drones.clear();
                    // Propagate errors - task assignment is mission-critical
                    self.tasks[i]
                        .assigned_drones
                        .push(drone_id)
                        .map_err(|_| SwarmError::ResourceExhausted)?;
                    self.assignments
                        .insert(task_id, drone_id.as_u64())
                        .map_err(|_| SwarmError::ResourceExhausted)?;
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
        self.tasks
            .iter()
            .find(|task| task.assigned_drones.contains(&drone_id) && !task.completed)
    }
}

impl Default for TaskAllocator {
    fn default() -> Self {
        Self::new()
    }
}
