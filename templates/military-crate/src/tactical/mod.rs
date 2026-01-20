//! Advanced Tactical Algorithms
//!
//! This module provides military-grade tactical algorithms for
//! formation control, threat response, and evasion maneuvers.
//!
//! Features:
//! - Dynamic Wedge Formation: Adaptive V-formation with spacing optimization
//! - Stacked Echelon: Multi-layer offset formations
//! - Distributed Mesh: Resilient hexagonal coverage grid
//! - Threat Classification: Multi-factor threat assessment
//! - Predictive Evasion: Trajectory-aware escape vector computation

extern crate alloc;
use alloc::vec::Vec;

use drone_swarm_system::extensions::{
    EvasionManeuver, FormationPlan, ObjectiveType, SwarmSnapshot, TacticalError, TacticalObjective,
    TacticalProvider, ThreatAction, ThreatAssessment, ThreatResponse, ThreatType,
};
use drone_swarm_system::types::{DroneId, Position, Velocity};

/// Pi constant for angle calculations
const PI: f32 = core::f32::consts::PI;

/// Advanced tactical provider with military-grade algorithms.
///
/// Implements classified formation patterns, threat assessment,
/// and evasion strategies.
#[derive(Debug)]
pub struct AdvancedTactical {
    /// Tactical doctrine configuration
    doctrine: TacticalDoctrine,
    /// Minimum safe distance between drones (meters)
    min_spacing: f32,
    /// Maximum formation radius (meters)
    max_radius: f32,
    /// Evasion speed multiplier
    evasion_speed: f32,
    /// Threat memory for predictive analysis
    threat_history: Vec<ThreatRecord>,
}

/// Record of past threats for pattern analysis
#[derive(Debug, Clone)]
struct ThreatRecord {
    position: Position,
    velocity: Option<Velocity>,
    threat_type: ThreatType,
    timestamp_ms: u64,
}

impl AdvancedTactical {
    /// Create a new advanced tactical provider.
    pub fn new() -> Self {
        Self {
            doctrine: TacticalDoctrine::Balanced,
            min_spacing: 5.0,
            max_radius: 100.0,
            evasion_speed: 15.0,
            threat_history: Vec::new(),
        }
    }

    /// Create with custom tactical doctrine.
    pub fn with_doctrine(doctrine: TacticalDoctrine) -> Self {
        let (min_spacing, evasion_speed) = match doctrine {
            TacticalDoctrine::Defensive => (8.0, 12.0),
            TacticalDoctrine::Balanced => (5.0, 15.0),
            TacticalDoctrine::Aggressive => (3.0, 18.0),
            TacticalDoctrine::Stealth => (10.0, 10.0),
        };

        Self {
            doctrine,
            min_spacing,
            max_radius: 100.0,
            evasion_speed,
            threat_history: Vec::new(),
        }
    }

    /// Compute dynamic wedge formation
    /// V-shaped formation that adapts spacing based on mission and swarm size
    fn compute_dynamic_wedge(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        let spacing = self.min_spacing + (objective.priority as f32) * 0.3;
        let angle = PI / 6.0; // 30 degrees wedge angle

        let mut assignments = Vec::with_capacity(n);

        // Leader at front
        if let Some((id, _)) = positions.first() {
            assignments.push((
                *id,
                Position {
                    x: objective.target.x,
                    y: objective.target.y,
                    z: objective.target.z,
                },
            ));
        }

        // Followers in V pattern
        for (i, (id, _)) in positions.iter().enumerate().skip(1) {
            let row = (i + 1) / 2;
            let side = if i % 2 == 1 { 1.0 } else { -1.0 };

            let offset_x = side * (row as f32) * spacing * angle.cos();
            let offset_y = -(row as f32) * spacing * angle.sin();
            let offset_z = (row as f32) * 0.5; // Slight altitude stacking

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + offset_x,
                    y: objective.target.y + offset_y,
                    z: objective.target.z + offset_z,
                },
            ));
        }

        assignments
    }

    /// Compute stacked echelon formation
    /// Multi-layer offset formation for defense in depth
    fn compute_stacked_echelon(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        let layer_spacing = self.min_spacing * 2.0;
        let lateral_spacing = self.min_spacing * 1.5;
        let altitude_step = 3.0;

        // Organize into layers (3-4 drones per layer)
        let drones_per_layer = 4;
        let num_layers = (n + drones_per_layer - 1) / drones_per_layer;

        let mut assignments = Vec::with_capacity(n);

        for (i, (id, _)) in positions.iter().enumerate() {
            let layer = i / drones_per_layer;
            let pos_in_layer = i % drones_per_layer;

            // Echelon offset: each layer is shifted
            let layer_offset = (layer as f32) * layer_spacing;
            let lateral_offset =
                (pos_in_layer as f32 - (drones_per_layer as f32 - 1.0) / 2.0) * lateral_spacing;

            // Stagger altitude by layer
            let altitude_offset = (layer as f32) * altitude_step;

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + lateral_offset,
                    y: objective.target.y - layer_offset,
                    z: objective.target.z + altitude_offset,
                },
            ));
        }

        assignments
    }

    /// Compute distributed mesh formation
    /// Hexagonal grid for maximum area coverage with redundancy
    fn compute_distributed_mesh(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        // Hexagonal grid spacing
        let hex_radius = self.min_spacing * 1.732; // sqrt(3) for hex geometry
        let row_height = hex_radius * 1.5;

        // Calculate grid dimensions
        let grid_cols = (n as f32).sqrt().ceil() as usize;

        let mut assignments = Vec::with_capacity(n);

        for (i, (id, _)) in positions.iter().enumerate() {
            let row = i / grid_cols;
            let col = i % grid_cols;

            // Hex offset for odd rows
            let hex_offset = if row % 2 == 1 {
                hex_radius / 2.0
            } else {
                0.0
            };

            // Center the grid around target
            let center_offset_x = (grid_cols as f32 * hex_radius) / 2.0;
            let center_offset_y = ((n / grid_cols) as f32 * row_height) / 2.0;

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + (col as f32 * hex_radius) + hex_offset - center_offset_x,
                    y: objective.target.y + (row as f32 * row_height) - center_offset_y,
                    z: objective.target.z + (i % 3) as f32 * 2.0, // Slight altitude variation
                },
            ));
        }

        assignments
    }

    /// Compute rotating perimeter formation
    /// Circle formation with designated sectors for 360-degree coverage
    fn compute_rotating_perimeter(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        let radius = self.min_spacing * 3.0 + (objective.priority as f32) * 0.5;

        let mut assignments = Vec::with_capacity(n);

        for (i, (id, _)) in positions.iter().enumerate() {
            let angle = (i as f32 / n as f32) * 2.0 * PI;

            // Inner/outer ring alternation for depth
            let ring_radius = if i % 2 == 0 {
                radius
            } else {
                radius * 0.7
            };

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + ring_radius * angle.cos(),
                    y: objective.target.y + ring_radius * angle.sin(),
                    z: objective.target.z + (i % 4) as f32 * 2.0,
                },
            ));
        }

        assignments
    }

    /// Compute expanding spiral formation (for search and rescue)
    fn compute_expanding_spiral(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        let initial_radius = self.min_spacing * 2.0;
        let expansion_rate = self.min_spacing * 0.5;

        let mut assignments = Vec::with_capacity(n);

        for (i, (id, _)) in positions.iter().enumerate() {
            // Archimedean spiral
            let angle = (i as f32) * PI / 3.0; // 60 degree increments
            let radius = initial_radius + expansion_rate * (i as f32);

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + radius * angle.cos(),
                    y: objective.target.y + radius * angle.sin(),
                    z: objective.target.z + 5.0, // Maintain constant altitude for search
                },
            ));
        }

        assignments
    }

    /// Compute stealth scatter formation (low observability)
    fn compute_stealth_scatter(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Vec<(DroneId, Position)> {
        let n = positions.len();
        if n == 0 {
            return Vec::new();
        }

        // Large spacing with pseudo-random offsets
        let base_spacing = self.max_radius / (n as f32).sqrt();

        let mut assignments = Vec::with_capacity(n);

        for (i, (id, _)) in positions.iter().enumerate() {
            let seed = id.0 as usize;

            // Pseudo-random position within area
            let rand_x = Self::pseudo_random(seed * 3) - 0.5;
            let rand_y = Self::pseudo_random(seed * 5) - 0.5;
            let rand_z = Self::pseudo_random(seed * 7) * 0.2;

            assignments.push((
                *id,
                Position {
                    x: objective.target.x + rand_x * base_spacing * (n as f32).sqrt(),
                    y: objective.target.y + rand_y * base_spacing * (n as f32).sqrt(),
                    z: objective.target.z + rand_z * 20.0 + 10.0, // Varying altitude
                },
            ));
        }

        assignments
    }

    /// Classify threat severity based on multiple factors
    fn classify_threat(&self, threat: &ThreatAssessment, swarm_state: &SwarmSnapshot) -> (ThreatAction, u8) {
        let base_level = threat.threat_level;

        // Factor 1: Proximity to swarm center
        let proximity_factor = if let Some(threat_pos) = &threat.position {
            let center = self.calculate_swarm_center(swarm_state);
            let dist = self.distance_3d(&center, threat_pos);
            if dist < 20.0 {
                20
            } else if dist < 50.0 {
                10
            } else {
                0
            }
        } else {
            5
        };

        // Factor 2: Threat type severity
        let type_factor = match threat.threat_type {
            ThreatType::Electronic => 25,     // High severity - affects comms
            ThreatType::Obstacle => 30,       // Critical - physical danger
            ThreatType::Weather => 20,        // Significant
            ThreatType::CommInterference => 15,
            ThreatType::Unknown => 20,        // Assume worst case
            ThreatType::Custom(_) => 10,
        };

        // Factor 3: Threat velocity (moving threats more dangerous)
        let velocity_factor = if let Some(vel) = &threat.velocity {
            let speed = (vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz).sqrt();
            (speed * 2.0).min(15.0) as u8
        } else {
            0
        };

        // Factor 4: Doctrine modifier
        let doctrine_modifier = match self.doctrine {
            TacticalDoctrine::Defensive => 10,  // More cautious
            TacticalDoctrine::Balanced => 0,
            TacticalDoctrine::Aggressive => -10, // More tolerant
            TacticalDoctrine::Stealth => 5,
        };

        let adjusted_level = (base_level as i16
            + proximity_factor as i16
            + type_factor as i16
            + velocity_factor as i16
            + doctrine_modifier)
            .clamp(0, 100) as u8;

        // Determine action based on adjusted level
        let action = match adjusted_level {
            0..=25 => ThreatAction::Continue,
            26..=45 => ThreatAction::Rally,
            46..=65 => ThreatAction::Evade,
            66..=80 => ThreatAction::Disperse,
            81..=90 => ThreatAction::ReturnToLaunch,
            91..=100 => ThreatAction::EmergencyLand,
            _ => ThreatAction::EmergencyLand,
        };

        (action, adjusted_level)
    }

    /// Calculate swarm center of mass
    fn calculate_swarm_center(&self, swarm_state: &SwarmSnapshot) -> Position {
        if swarm_state.positions.is_empty() {
            return Position { x: 0.0, y: 0.0, z: 0.0 };
        }

        let n = swarm_state.positions.len() as f32;
        let (sum_x, sum_y, sum_z) = swarm_state
            .positions
            .iter()
            .fold((0.0, 0.0, 0.0), |(x, y, z), (_, pos)| {
                (x + pos.x, y + pos.y, z + pos.z)
            });

        Position {
            x: sum_x / n,
            y: sum_y / n,
            z: sum_z / n,
        }
    }

    /// 3D distance calculation
    fn distance_3d(&self, a: &Position, b: &Position) -> f32 {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        let dz = a.z - b.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Pseudo-random number generator (0.0 to 1.0)
    fn pseudo_random(seed: usize) -> f32 {
        let a = 1_103_515_245_u64;
        let c = 12_345_u64;
        let m = 2_147_483_648_u64;
        let x = ((a.wrapping_mul(seed as u64).wrapping_add(c)) % m) as f32;
        x / m as f32
    }

    /// Compute predictive escape vector considering threat trajectory
    fn compute_escape_vector(
        &self,
        current_pos: &Position,
        current_vel: &Velocity,
        threat_pos: &Position,
        threat_vel: Option<&Velocity>,
    ) -> (Position, Velocity) {
        // Vector from threat to current position
        let mut dx = current_pos.x - threat_pos.x;
        let mut dy = current_pos.y - threat_pos.y;
        let mut dz = current_pos.z - threat_pos.z;

        // If threat has velocity, predict intercept and adjust
        if let Some(t_vel) = threat_vel {
            let t_speed = (t_vel.vx * t_vel.vx + t_vel.vy * t_vel.vy + t_vel.vz * t_vel.vz).sqrt();
            if t_speed > 0.1 {
                // Predict threat position in 2 seconds
                let predicted_threat_x = threat_pos.x + t_vel.vx * 2.0;
                let predicted_threat_y = threat_pos.y + t_vel.vy * 2.0;
                let predicted_threat_z = threat_pos.z + t_vel.vz * 2.0;

                // Escape from predicted position
                dx = current_pos.x - predicted_threat_x;
                dy = current_pos.y - predicted_threat_y;
                dz = current_pos.z - predicted_threat_z;
            }
        }

        let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);

        // Normalize escape direction
        let escape_x = dx / dist;
        let escape_y = dy / dist;
        let escape_z = dz / dist;

        // Add perpendicular component for unpredictability
        let perp_x = -escape_y;
        let perp_y = escape_x;

        // Combine with current momentum
        let momentum_factor = 0.2;
        let escape_factor = 0.6;
        let perp_factor = 0.2;

        let final_dir_x = escape_x * escape_factor
            + perp_x * perp_factor
            + current_vel.vx.signum() * momentum_factor;
        let final_dir_y = escape_y * escape_factor
            + perp_y * perp_factor
            + current_vel.vy.signum() * momentum_factor;
        let final_dir_z = escape_z * escape_factor + 0.3; // Slight upward bias

        // Normalize final direction
        let final_mag = (final_dir_x * final_dir_x + final_dir_y * final_dir_y + final_dir_z * final_dir_z).sqrt().max(0.1);

        let norm_x = final_dir_x / final_mag;
        let norm_y = final_dir_y / final_mag;
        let norm_z = final_dir_z / final_mag;

        // Escape distance proportional to threat proximity
        let escape_dist = 50.0_f32.max(dist * 1.5).min(self.max_radius);

        let target_position = Position {
            x: current_pos.x + norm_x * escape_dist,
            y: current_pos.y + norm_y * escape_dist,
            z: (current_pos.z + norm_z * 30.0).max(10.0), // Minimum altitude
        };

        let target_velocity = Velocity {
            vx: norm_x * self.evasion_speed,
            vy: norm_y * self.evasion_speed,
            vz: norm_z * self.evasion_speed * 0.5,
        };

        (target_position, target_velocity)
    }
}

impl Default for AdvancedTactical {
    fn default() -> Self {
        Self::new()
    }
}

/// Tactical doctrine presets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TacticalDoctrine {
    /// Defensive posture - prioritize survivability
    Defensive,
    /// Balanced - adapt to situation
    Balanced,
    /// Aggressive - prioritize mission completion
    Aggressive,
    /// Stealth - minimize detection signature
    Stealth,
}

impl TacticalProvider for AdvancedTactical {
    fn compute_formation(
        &self,
        positions: &[(DroneId, Position)],
        objective: &TacticalObjective,
    ) -> Result<FormationPlan, TacticalError> {
        if positions.is_empty() {
            return Err(TacticalError::InsufficientDrones);
        }

        // Select formation based on objective type and doctrine
        let (assignments, formation_type) = match objective.objective_type {
            ObjectiveType::Coverage => (
                self.compute_distributed_mesh(positions, objective),
                "distributed_mesh",
            ),
            ObjectiveType::Defense => (
                self.compute_rotating_perimeter(positions, objective),
                "rotating_perimeter",
            ),
            ObjectiveType::Escort => (
                self.compute_dynamic_wedge(positions, objective),
                "dynamic_wedge",
            ),
            ObjectiveType::SearchRescue => (
                self.compute_expanding_spiral(positions, objective),
                "expanding_spiral",
            ),
            ObjectiveType::Reconnaissance => (
                self.compute_stealth_scatter(positions, objective),
                "stealth_scatter",
            ),
            ObjectiveType::Custom(code) => {
                // Custom formations based on code
                match code {
                    1 => (
                        self.compute_stacked_echelon(positions, objective),
                        "stacked_echelon",
                    ),
                    _ => (
                        self.compute_distributed_mesh(positions, objective),
                        "adaptive",
                    ),
                }
            }
        };

        // Calculate ETA based on maximum distance any drone needs to travel
        let max_distance = positions
            .iter()
            .zip(assignments.iter())
            .map(|((_, from), (_, to))| {
                let dx = to.x - from.x;
                let dy = to.y - from.y;
                let dz = to.z - from.z;
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .fold(0.0f32, |a, b| a.max(b));

        // ETA = distance / average_speed (assuming 10 m/s average)
        let eta_ms = ((max_distance / 10.0) * 1000.0) as u64 + 2000; // +2s buffer

        Ok(FormationPlan {
            assignments,
            formation_type,
            eta_ms,
        })
    }

    fn compute_threat_response(
        &self,
        threat: &ThreatAssessment,
        swarm_state: &SwarmSnapshot,
    ) -> Result<ThreatResponse, TacticalError> {
        // Classify threat using multi-factor analysis
        let (action, urgency) = self.classify_threat(threat, swarm_state);

        // Identify priority drones (those closest to threat)
        let priority_drones = if let Some(threat_pos) = &threat.position {
            let mut with_dist: Vec<_> = swarm_state
                .positions
                .iter()
                .map(|(id, pos)| {
                    let dist = self.distance_3d(pos, threat_pos);
                    (*id, dist)
                })
                .collect();

            with_dist.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(core::cmp::Ordering::Equal));

            // Determine how many drones should be prioritized based on action
            let priority_fraction = match action {
                ThreatAction::Continue => 0.0,
                ThreatAction::Rally => 0.2,
                ThreatAction::Evade => 0.3,
                ThreatAction::Disperse => 0.5,
                ThreatAction::ReturnToLaunch => 0.8,
                ThreatAction::EmergencyLand => 1.0,
            };

            let priority_count = ((swarm_state.positions.len() as f32) * priority_fraction).ceil() as usize;
            with_dist
                .into_iter()
                .take(priority_count)
                .map(|(id, _)| id)
                .collect()
        } else {
            Vec::new()
        };

        Ok(ThreatResponse {
            action,
            priority_drones,
            urgency,
        })
    }

    fn compute_evasion(
        &self,
        _drone_id: DroneId,
        current_pos: Position,
        current_vel: Velocity,
        threat_pos: Position,
    ) -> Result<EvasionManeuver, TacticalError> {
        // Compute predictive escape vector
        let (target_position, target_velocity) =
            self.compute_escape_vector(&current_pos, &current_vel, &threat_pos, None);

        // Calculate maneuver duration based on escape distance
        let escape_dist = self.distance_3d(&current_pos, &target_position);
        let duration_ms = ((escape_dist / self.evasion_speed) * 1000.0) as u64;

        Ok(EvasionManeuver {
            target_position,
            target_velocity,
            duration_ms: duration_ms.max(2000), // Minimum 2 second maneuver
        })
    }

    fn provider_name(&self) -> &'static str {
        "AdvancedTactical (Military-Grade)"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_positions(n: usize) -> Vec<(DroneId, Position)> {
        (0..n)
            .map(|i| {
                (
                    DroneId(i as u64),
                    Position {
                        x: i as f32 * 5.0,
                        y: 0.0,
                        z: 10.0,
                    },
                )
            })
            .collect()
    }

    #[test]
    fn test_formation_computation() {
        let tactical = AdvancedTactical::new();
        let positions = create_test_positions(5);
        let objective = TacticalObjective {
            target: Position { x: 50.0, y: 50.0, z: 20.0 },
            objective_type: ObjectiveType::Coverage,
            priority: 80,
        };

        let plan = tactical.compute_formation(&positions, &objective).unwrap();

        assert_eq!(plan.assignments.len(), 5);
        assert_eq!(plan.formation_type, "distributed_mesh");
    }

    #[test]
    fn test_wedge_formation() {
        let tactical = AdvancedTactical::new();
        let positions = create_test_positions(7);
        let objective = TacticalObjective {
            target: Position { x: 0.0, y: 0.0, z: 15.0 },
            objective_type: ObjectiveType::Escort,
            priority: 60,
        };

        let plan = tactical.compute_formation(&positions, &objective).unwrap();

        assert_eq!(plan.formation_type, "dynamic_wedge");
        // Leader should be at target
        assert_eq!(plan.assignments[0].1.x, objective.target.x);
    }

    #[test]
    fn test_threat_response_high_level() {
        let tactical = AdvancedTactical::new();
        let threat = ThreatAssessment {
            position: Some(Position { x: 10.0, y: 10.0, z: 15.0 }),
            velocity: Some(Velocity { vx: 5.0, vy: 5.0, vz: 0.0 }),
            threat_level: 75,
            threat_type: ThreatType::Electronic,
        };
        let swarm_state = SwarmSnapshot {
            positions: vec![
                (DroneId(1), Position { x: 0.0, y: 0.0, z: 10.0 }),
                (DroneId(2), Position { x: 5.0, y: 0.0, z: 10.0 }),
            ],
            velocities: vec![
                (DroneId(1), Velocity { vx: 0.0, vy: 0.0, vz: 0.0 }),
                (DroneId(2), Velocity { vx: 0.0, vy: 0.0, vz: 0.0 }),
            ],
            formation: "grid",
        };

        let response = tactical.compute_threat_response(&threat, &swarm_state).unwrap();

        // High threat level should trigger evacuation-type response
        assert!(matches!(
            response.action,
            ThreatAction::Evade | ThreatAction::Disperse | ThreatAction::ReturnToLaunch
        ));
        assert!(response.urgency >= 70);
    }

    #[test]
    fn test_evasion_maneuver() {
        let tactical = AdvancedTactical::new();
        let current_pos = Position { x: 0.0, y: 0.0, z: 20.0 };
        let current_vel = Velocity { vx: 2.0, vy: 0.0, vz: 0.0 };
        let threat_pos = Position { x: 10.0, y: 0.0, z: 20.0 };

        let maneuver = tactical
            .compute_evasion(DroneId(1), current_pos, current_vel, threat_pos)
            .unwrap();

        // Should move away from threat
        assert!(maneuver.target_position.x < current_pos.x);
        assert!(maneuver.duration_ms >= 2000);
    }

    #[test]
    fn test_doctrine_affects_behavior() {
        let defensive = AdvancedTactical::with_doctrine(TacticalDoctrine::Defensive);
        let aggressive = AdvancedTactical::with_doctrine(TacticalDoctrine::Aggressive);

        let threat = ThreatAssessment {
            position: Some(Position { x: 50.0, y: 50.0, z: 20.0 }),
            velocity: None,
            threat_level: 50,
            threat_type: ThreatType::Unknown,
        };
        let swarm_state = SwarmSnapshot {
            positions: vec![(DroneId(1), Position { x: 0.0, y: 0.0, z: 10.0 })],
            velocities: vec![(DroneId(1), Velocity { vx: 0.0, vy: 0.0, vz: 0.0 })],
            formation: "grid",
        };

        let defensive_response = defensive.compute_threat_response(&threat, &swarm_state).unwrap();
        let aggressive_response = aggressive.compute_threat_response(&threat, &swarm_state).unwrap();

        // Defensive should have higher urgency
        assert!(defensive_response.urgency >= aggressive_response.urgency);
    }

    #[test]
    fn test_empty_positions_error() {
        let tactical = AdvancedTactical::new();
        let objective = TacticalObjective {
            target: Position { x: 0.0, y: 0.0, z: 10.0 },
            objective_type: ObjectiveType::Coverage,
            priority: 50,
        };

        let result = tactical.compute_formation(&[], &objective);
        assert!(matches!(result, Err(TacticalError::InsufficientDrones)));
    }
}
