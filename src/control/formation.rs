//! Formation Calculator for Swarm Coordination
//!
//! Computes target positions for various formation patterns.

use crate::types::Position;
use core::f32::consts::PI;

/// Swarm formation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Formation {
    /// Random distribution (no specific formation)
    Random,
    /// Grid formation
    Grid { spacing: u32 },
    /// Line formation
    Line { spacing: u32 },
    /// Circle formation
    Circle { radius: u32 },
    /// V-formation (like birds)
    VFormation { spacing: u32 },
    /// Diamond formation
    Diamond { spacing: u32 },
    /// Wedge formation (military)
    Wedge { spacing: u32, angle_deg: u32 },
    /// Custom formation (positions defined externally)
    Custom,
}

impl Default for Formation {
    fn default() -> Self {
        Formation::Random
    }
}

/// Calculates formation positions for drones
#[derive(Debug, Clone)]
pub struct FormationCalculator {
    /// Current formation type
    formation: Formation,
    /// Formation center/reference point
    center: Position,
    /// Formation heading (radians)
    heading: f32,
}

impl FormationCalculator {
    /// Create new formation calculator
    pub fn new() -> Self {
        Self {
            formation: Formation::Random,
            center: Position { x: 0.0, y: 0.0, z: 0.0 },
            heading: 0.0,
        }
    }

    /// Create with initial formation
    pub fn with_formation(formation: Formation) -> Self {
        Self {
            formation,
            center: Position { x: 0.0, y: 0.0, z: 0.0 },
            heading: 0.0,
        }
    }

    /// Set formation type
    pub fn set_formation(&mut self, formation: Formation) {
        self.formation = formation;
    }

    /// Get current formation
    pub fn formation(&self) -> Formation {
        self.formation
    }

    /// Set formation center
    pub fn set_center(&mut self, center: Position) {
        self.center = center;
    }

    /// Set formation heading (radians)
    pub fn set_heading(&mut self, heading: f32) {
        self.heading = heading;
    }

    /// Compute position for a drone in the formation
    ///
    /// # Arguments
    /// * `drone_index` - Index of the drone in the swarm (0 = leader)
    /// * `swarm_size` - Total size of the swarm
    /// * `current_altitude` - Current altitude to maintain (z coordinate)
    pub fn compute_position(
        &self,
        drone_index: u64,
        swarm_size: usize,
        current_altitude: f32,
    ) -> Position {
        let (local_x, local_y) = match self.formation {
            Formation::Random => return Position {
                x: self.center.x,
                y: self.center.y,
                z: current_altitude,
            },
            Formation::Grid { spacing } => self.compute_grid(drone_index, swarm_size, spacing),
            Formation::Line { spacing } => self.compute_line(drone_index, spacing),
            Formation::Circle { radius } => self.compute_circle(drone_index, swarm_size, radius),
            Formation::VFormation { spacing } => self.compute_v_formation(drone_index, spacing),
            Formation::Diamond { spacing } => self.compute_diamond(drone_index, spacing),
            Formation::Wedge { spacing, angle_deg } => {
                self.compute_wedge(drone_index, spacing, angle_deg)
            }
            Formation::Custom => (0.0, 0.0),
        };

        // Apply rotation based on heading
        let (rotated_x, rotated_y) = self.rotate_point(local_x, local_y);

        Position {
            x: self.center.x + rotated_x,
            y: self.center.y + rotated_y,
            z: current_altitude,
        }
    }

    /// Compute grid formation position
    fn compute_grid(&self, drone_index: u64, swarm_size: usize, spacing: u32) -> (f32, f32) {
        let grid_size = (libm::sqrtf(swarm_size as f32).ceil() as u64).max(1);
        let row = drone_index / grid_size;
        let col = drone_index % grid_size;

        // Center the grid
        let offset_x = (grid_size as f32 - 1.0) / 2.0 * spacing as f32;
        let offset_y = ((swarm_size as u64 / grid_size) as f32 - 1.0) / 2.0 * spacing as f32;

        (
            col as f32 * spacing as f32 - offset_x,
            row as f32 * spacing as f32 - offset_y,
        )
    }

    /// Compute line formation position
    fn compute_line(&self, drone_index: u64, spacing: u32) -> (f32, f32) {
        (drone_index as f32 * spacing as f32, 0.0)
    }

    /// Compute circle formation position
    fn compute_circle(&self, drone_index: u64, swarm_size: usize, radius: u32) -> (f32, f32) {
        let total = swarm_size.max(1) as f32;
        let angle = (drone_index as f32 / total) * 2.0 * PI;

        (
            libm::cosf(angle) * radius as f32,
            libm::sinf(angle) * radius as f32,
        )
    }

    /// Compute V-formation position
    fn compute_v_formation(&self, drone_index: u64, spacing: u32) -> (f32, f32) {
        if drone_index == 0 {
            // Leader at front
            (0.0, 0.0)
        } else {
            // Alternate left and right
            let side = if drone_index % 2 == 1 { -1.0 } else { 1.0 };
            let row = (drone_index + 1) / 2;
            (
                side * row as f32 * spacing as f32,
                -(row as f32 * spacing as f32),
            )
        }
    }

    /// Compute diamond formation position
    fn compute_diamond(&self, drone_index: u64, spacing: u32) -> (f32, f32) {
        match drone_index {
            0 => (0.0, spacing as f32), // Front
            1 => (-(spacing as f32), 0.0), // Left
            2 => (spacing as f32, 0.0), // Right
            3 => (0.0, -(spacing as f32)), // Back
            _ => {
                // Additional drones form outer diamond
                let ring = (drone_index - 4) / 4 + 1;
                let pos_in_ring = (drone_index - 4) % 4;
                let dist = (ring + 1) as f32 * spacing as f32;
                match pos_in_ring {
                    0 => (0.0, dist),
                    1 => (-dist, 0.0),
                    2 => (dist, 0.0),
                    _ => (0.0, -dist),
                }
            }
        }
    }

    /// Compute wedge formation position
    fn compute_wedge(&self, drone_index: u64, spacing: u32, angle_deg: u32) -> (f32, f32) {
        if drone_index == 0 {
            (0.0, 0.0)
        } else {
            let angle_rad = (angle_deg as f32).to_radians() / 2.0;
            let side = if drone_index % 2 == 1 { -1.0 } else { 1.0 };
            let row = (drone_index + 1) / 2;

            let x = side * row as f32 * spacing as f32 * libm::cosf(angle_rad);
            let y = -(row as f32 * spacing as f32 * libm::sinf(angle_rad));
            (x, y)
        }
    }

    /// Rotate point around origin by heading angle
    fn rotate_point(&self, x: f32, y: f32) -> (f32, f32) {
        let cos_h = libm::cosf(self.heading);
        let sin_h = libm::sinf(self.heading);
        (
            x * cos_h - y * sin_h,
            x * sin_h + y * cos_h,
        )
    }

    /// Check if a position is within formation tolerance
    pub fn is_in_position(&self, actual: &Position, target: &Position, tolerance: f32) -> bool {
        let dx = actual.x - target.x;
        let dy = actual.y - target.y;
        let dz = actual.z - target.z;
        let dist_sq = dx * dx + dy * dy + dz * dz;
        dist_sq <= tolerance * tolerance
    }
}

impl Default for FormationCalculator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_formation() {
        let calc = FormationCalculator::with_formation(Formation::Line { spacing: 10 });

        let pos0 = calc.compute_position(0, 5, 10.0);
        let pos1 = calc.compute_position(1, 5, 10.0);
        let pos2 = calc.compute_position(2, 5, 10.0);

        assert_eq!(pos0.x, 0.0);
        assert_eq!(pos1.x, 10.0);
        assert_eq!(pos2.x, 20.0);
    }

    #[test]
    fn test_circle_formation() {
        let calc = FormationCalculator::with_formation(Formation::Circle { radius: 10 });

        let pos0 = calc.compute_position(0, 4, 10.0);
        // First drone should be at angle 0 (right side)
        assert!((pos0.x - 10.0).abs() < 0.01);
        assert!(pos0.y.abs() < 0.01);
    }

    #[test]
    fn test_v_formation() {
        let calc = FormationCalculator::with_formation(Formation::VFormation { spacing: 10 });

        let pos0 = calc.compute_position(0, 5, 10.0);
        let pos1 = calc.compute_position(1, 5, 10.0);
        let pos2 = calc.compute_position(2, 5, 10.0);

        // Leader at front
        assert_eq!(pos0.x, 0.0);
        assert_eq!(pos0.y, 0.0);

        // First follower on left
        assert_eq!(pos1.x, -10.0);

        // Second follower on right
        assert_eq!(pos2.x, 10.0);
    }

    #[test]
    fn test_position_tolerance() {
        let calc = FormationCalculator::new();
        let actual = Position { x: 10.1, y: 10.0, z: 10.0 };
        let target = Position { x: 10.0, y: 10.0, z: 10.0 };

        assert!(calc.is_in_position(&actual, &target, 0.5));
        assert!(!calc.is_in_position(&actual, &target, 0.05));
    }
}
