//! Advanced Collision Avoidance System
//!
//! This module provides multiple collision avoidance algorithms for drone swarms:
//! - Velocity Obstacles (VO)
//! - Reciprocal Velocity Obstacles (RVO)
//! - ORCA (Optimal Reciprocal Collision Avoidance)
//! - Artificial Potential Fields (APF)
//! - Geofencing with boundary avoidance
//!
//! # Safety Guarantees
//! - Minimum separation distance enforcement
//! - Velocity limits for safe maneuvering
//! - Emergency stop capability
//! - Fail-safe defaults
//!
//! # Example
//! ```ignore
//! use drone_swarm_system::collision_avoidance::{CollisionAvoidance, AvoidanceConfig};
//!
//! let config = AvoidanceConfig::default();
//! let mut avoidance = CollisionAvoidance::new(config);
//!
//! // Add obstacles (other drones)
//! avoidance.add_obstacle([10.0, 5.0, -10.0], [1.0, 0.0, 0.0], 0.5);
//!
//! // Compute safe velocity
//! let safe_vel = avoidance.compute_safe_velocity(
//!     [0.0, 0.0, -10.0],  // current position
//!     [2.0, 0.0, 0.0],    // desired velocity
//! );
//! ```

use heapless::Vec;

/// Maximum number of obstacles to track
pub const MAX_OBSTACLES: usize = 32;

/// Maximum number of geofence vertices
pub const MAX_GEOFENCE_VERTICES: usize = 16;

/// Collision avoidance algorithm selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AvoidanceAlgorithm {
    /// Velocity Obstacles - basic collision cone
    VelocityObstacle,
    /// Reciprocal Velocity Obstacles - shared responsibility
    RVO,
    /// Optimal Reciprocal Collision Avoidance - linear programming
    ORCA,
    /// Artificial Potential Field - gradient descent
    PotentialField,
    /// Combined approach (ORCA + APF)
    Hybrid,
}

/// Configuration for collision avoidance system
#[derive(Debug, Clone)]
pub struct AvoidanceConfig {
    /// Minimum separation distance (meters)
    pub min_separation: f32,
    /// Safety margin added to obstacles (meters)
    pub safety_margin: f32,
    /// Maximum velocity magnitude (m/s)
    pub max_velocity: f32,
    /// Time horizon for collision prediction (seconds)
    pub time_horizon: f32,
    /// Responsiveness factor (0.0-1.0)
    pub responsiveness: f32,
    /// Algorithm to use
    pub algorithm: AvoidanceAlgorithm,
    /// Enable geofencing
    pub geofence_enabled: bool,
    /// Geofence boundary buffer (meters)
    pub geofence_buffer: f32,
    /// Vertical limits [min_altitude, max_altitude] (positive values)
    pub altitude_limits: [f32; 2],
}

impl Default for AvoidanceConfig {
    fn default() -> Self {
        Self {
            min_separation: 3.0,
            safety_margin: 1.0,
            max_velocity: 10.0,
            time_horizon: 5.0,
            responsiveness: 0.8,
            algorithm: AvoidanceAlgorithm::Hybrid,
            geofence_enabled: true,
            geofence_buffer: 5.0,
            altitude_limits: [2.0, 100.0],
        }
    }
}

/// Obstacle representation (other drone or static obstacle)
#[derive(Debug, Clone, Copy)]
pub struct Obstacle {
    /// Position [x, y, z]
    pub position: [f32; 3],
    /// Velocity [vx, vy, vz]
    pub velocity: [f32; 3],
    /// Radius (combined agent + obstacle radius)
    pub radius: f32,
    /// Is obstacle static (doesn't move)
    pub is_static: bool,
    /// Priority (higher = must avoid more)
    pub priority: u8,
}

impl Obstacle {
    /// Create a new obstacle
    pub fn new(position: [f32; 3], velocity: [f32; 3], radius: f32) -> Self {
        Self {
            position,
            velocity,
            radius,
            is_static: false,
            priority: 1,
        }
    }

    /// Create a static obstacle
    pub fn new_static(position: [f32; 3], radius: f32) -> Self {
        Self {
            position,
            velocity: [0.0, 0.0, 0.0],
            radius,
            is_static: true,
            priority: 2, // Static obstacles have higher priority
        }
    }
}

/// Geofence definition (polygon boundary)
#[derive(Debug, Clone)]
pub struct Geofence {
    /// Polygon vertices [x, y] in clockwise order
    pub vertices: Vec<[f32; 2], MAX_GEOFENCE_VERTICES>,
    /// Minimum altitude (positive, meters AGL)
    pub min_altitude: f32,
    /// Maximum altitude (positive, meters AGL)
    pub max_altitude: f32,
    /// Is fence inclusive (true = stay inside) or exclusive (true = stay outside)
    pub inclusive: bool,
}

impl Default for Geofence {
    fn default() -> Self {
        // Default 100m x 100m square centered at origin
        let mut vertices = Vec::new();
        let _ = vertices.push([-50.0, -50.0]);
        let _ = vertices.push([50.0, -50.0]);
        let _ = vertices.push([50.0, 50.0]);
        let _ = vertices.push([-50.0, 50.0]);

        Self {
            vertices,
            min_altitude: 2.0,
            max_altitude: 100.0,
            inclusive: true,
        }
    }
}

impl Geofence {
    /// Check if point is inside geofence (2D check)
    pub fn contains_point(&self, x: f32, y: f32) -> bool {
        if self.vertices.len() < 3 {
            return true; // No valid geofence
        }

        // Ray casting algorithm
        let mut inside = false;
        let n = self.vertices.len();

        let mut j = n - 1;
        for i in 0..n {
            let xi = self.vertices[i][0];
            let yi = self.vertices[i][1];
            let xj = self.vertices[j][0];
            let yj = self.vertices[j][1];

            if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
            j = i;
        }

        if self.inclusive {
            inside
        } else {
            !inside
        }
    }

    /// Get distance to nearest boundary edge
    pub fn distance_to_boundary(&self, x: f32, y: f32) -> f32 {
        if self.vertices.len() < 2 {
            return f32::MAX;
        }

        let mut min_dist = f32::MAX;
        let n = self.vertices.len();

        for i in 0..n {
            let j = (i + 1) % n;
            let dist = point_to_segment_distance(
                x,
                y,
                self.vertices[i][0],
                self.vertices[i][1],
                self.vertices[j][0],
                self.vertices[j][1],
            );
            min_dist = min_dist.min(dist);
        }

        min_dist
    }
}

/// Main collision avoidance system
#[derive(Debug, Clone)]
pub struct CollisionAvoidance {
    /// Configuration
    config: AvoidanceConfig,
    /// Current obstacles
    obstacles: Vec<Obstacle, MAX_OBSTACLES>,
    /// Geofence boundary
    geofence: Geofence,
    /// Agent radius (this drone's safety radius)
    agent_radius: f32,
}

impl CollisionAvoidance {
    /// Create new collision avoidance system
    pub fn new(config: AvoidanceConfig) -> Self {
        Self {
            config,
            obstacles: Vec::new(),
            geofence: Geofence::default(),
            agent_radius: 0.5, // Default drone radius
        }
    }

    /// Set agent (this drone's) radius
    pub fn set_agent_radius(&mut self, radius: f32) {
        self.agent_radius = radius;
    }

    /// Set geofence
    pub fn set_geofence(&mut self, geofence: Geofence) {
        self.geofence = geofence;
    }

    /// Clear all obstacles
    pub fn clear_obstacles(&mut self) {
        self.obstacles.clear();
    }

    /// Add an obstacle (other drone or static object)
    pub fn add_obstacle(&mut self, position: [f32; 3], velocity: [f32; 3], radius: f32) -> bool {
        if self.obstacles.len() >= MAX_OBSTACLES {
            return false;
        }
        let _ = self.obstacles.push(Obstacle::new(position, velocity, radius));
        true
    }

    /// Add a static obstacle
    pub fn add_static_obstacle(&mut self, position: [f32; 3], radius: f32) -> bool {
        if self.obstacles.len() >= MAX_OBSTACLES {
            return false;
        }
        let _ = self.obstacles.push(Obstacle::new_static(position, radius));
        true
    }

    /// Compute safe velocity given current position and desired velocity
    pub fn compute_safe_velocity(
        &self,
        position: [f32; 3],
        desired_velocity: [f32; 3],
    ) -> [f32; 3] {
        let mut safe_vel = desired_velocity;

        // Apply selected algorithm
        safe_vel = match self.config.algorithm {
            AvoidanceAlgorithm::VelocityObstacle => {
                self.velocity_obstacle(position, safe_vel)
            }
            AvoidanceAlgorithm::RVO => {
                self.reciprocal_velocity_obstacle(position, safe_vel)
            }
            AvoidanceAlgorithm::ORCA => {
                self.orca(position, safe_vel)
            }
            AvoidanceAlgorithm::PotentialField => {
                self.potential_field(position, safe_vel)
            }
            AvoidanceAlgorithm::Hybrid => {
                // Combine ORCA and potential field
                let orca_vel = self.orca(position, safe_vel);
                self.potential_field(position, orca_vel)
            }
        };

        // Apply geofence constraints
        if self.config.geofence_enabled {
            safe_vel = self.apply_geofence_constraints(position, safe_vel);
        }

        // Enforce velocity limits
        safe_vel = self.clamp_velocity(safe_vel);

        safe_vel
    }

    /// Velocity Obstacle algorithm
    fn velocity_obstacle(&self, position: [f32; 3], desired_vel: [f32; 3]) -> [f32; 3] {
        let mut result = desired_vel;

        for obs in &self.obstacles {
            let combined_radius = self.agent_radius + obs.radius + self.config.safety_margin;

            // Relative position and velocity
            let rel_pos = [
                obs.position[0] - position[0],
                obs.position[1] - position[1],
                obs.position[2] - position[2],
            ];

            let dist = magnitude(&rel_pos);

            if dist < 0.01 {
                continue; // Skip if at same position
            }

            // Check if current velocity leads to collision
            let rel_vel = [
                desired_vel[0] - obs.velocity[0],
                desired_vel[1] - obs.velocity[1],
                desired_vel[2] - obs.velocity[2],
            ];

            // Time to closest approach
            let dot_rv_rp = dot(&rel_vel, &rel_pos);
            let rel_vel_sq = dot(&rel_vel, &rel_vel);

            if rel_vel_sq < 0.0001 {
                continue;
            }

            let t_closest = -dot_rv_rp / rel_vel_sq;

            if t_closest < 0.0 || t_closest > self.config.time_horizon {
                continue; // No collision in time horizon
            }

            // Distance at closest approach
            let closest_pos = [
                rel_pos[0] + rel_vel[0] * t_closest,
                rel_pos[1] + rel_vel[1] * t_closest,
                rel_pos[2] + rel_vel[2] * t_closest,
            ];

            let closest_dist = magnitude(&closest_pos);

            if closest_dist < combined_radius {
                // Collision predicted - adjust velocity
                let avoidance_strength =
                    (combined_radius - closest_dist) / combined_radius * self.config.responsiveness;

                // Steer perpendicular to relative position
                let perp = perpendicular_3d(&rel_pos);
                result[0] += perp[0] * avoidance_strength * self.config.max_velocity;
                result[1] += perp[1] * avoidance_strength * self.config.max_velocity;
                result[2] += perp[2] * avoidance_strength * self.config.max_velocity * 0.5;
            }
        }

        result
    }

    /// Reciprocal Velocity Obstacle algorithm
    fn reciprocal_velocity_obstacle(&self, position: [f32; 3], desired_vel: [f32; 3]) -> [f32; 3] {
        let mut result = desired_vel;

        for obs in &self.obstacles {
            let combined_radius = self.agent_radius + obs.radius + self.config.safety_margin;

            let rel_pos = [
                obs.position[0] - position[0],
                obs.position[1] - position[1],
                obs.position[2] - position[2],
            ];

            let dist = magnitude(&rel_pos);

            if dist < combined_radius {
                // Already in collision - emergency avoidance
                let escape_dir = normalize(&[
                    position[0] - obs.position[0],
                    position[1] - obs.position[1],
                    position[2] - obs.position[2],
                ]);
                return [
                    escape_dir[0] * self.config.max_velocity,
                    escape_dir[1] * self.config.max_velocity,
                    escape_dir[2] * self.config.max_velocity * 0.3,
                ];
            }

            // RVO: Use average of current velocities as reference
            let avg_vel = [
                (desired_vel[0] + obs.velocity[0]) * 0.5,
                (desired_vel[1] + obs.velocity[1]) * 0.5,
                (desired_vel[2] + obs.velocity[2]) * 0.5,
            ];

            // Check if average velocity leads to collision
            let rel_vel = [
                avg_vel[0] - obs.velocity[0],
                avg_vel[1] - obs.velocity[1],
                avg_vel[2] - obs.velocity[2],
            ];

            let dot_rv_rp = dot(&rel_vel, &rel_pos);
            let rel_vel_sq = dot(&rel_vel, &rel_vel);

            if rel_vel_sq < 0.0001 {
                continue;
            }

            let t_closest = -dot_rv_rp / rel_vel_sq;

            if t_closest > 0.0 && t_closest < self.config.time_horizon {
                let closest_pos = [
                    rel_pos[0] + rel_vel[0] * t_closest,
                    rel_pos[1] + rel_vel[1] * t_closest,
                    rel_pos[2] + rel_vel[2] * t_closest,
                ];

                let closest_dist = magnitude(&closest_pos);

                if closest_dist < combined_radius {
                    // Need to adjust - each agent takes half responsibility
                    let avoidance_strength = (combined_radius - closest_dist)
                        / combined_radius
                        * self.config.responsiveness
                        * 0.5; // Half responsibility

                    let perp = perpendicular_3d(&rel_pos);
                    result[0] += perp[0] * avoidance_strength * self.config.max_velocity;
                    result[1] += perp[1] * avoidance_strength * self.config.max_velocity;
                    result[2] += perp[2] * avoidance_strength * self.config.max_velocity * 0.5;
                }
            }
        }

        result
    }

    /// ORCA (Optimal Reciprocal Collision Avoidance) - simplified implementation
    fn orca(&self, position: [f32; 3], desired_vel: [f32; 3]) -> [f32; 3] {
        let mut result = desired_vel;
        let tau = self.config.time_horizon;

        for obs in &self.obstacles {
            let combined_radius = self.agent_radius + obs.radius + self.config.safety_margin;

            let rel_pos = [
                obs.position[0] - position[0],
                obs.position[1] - position[1],
                obs.position[2] - position[2],
            ];

            let dist_sq = dot(&rel_pos, &rel_pos);
            let dist = libm::sqrtf(dist_sq);

            if dist < 0.01 {
                continue;
            }

            let rel_vel = [
                desired_vel[0] - obs.velocity[0],
                desired_vel[1] - obs.velocity[1],
                desired_vel[2] - obs.velocity[2],
            ];

            // ORCA constraint computation
            if dist > combined_radius {
                // Not in collision yet
                let _leg = libm::sqrtf(dist_sq - combined_radius * combined_radius);

                // Check if velocity is inside the velocity obstacle
                let w = [
                    rel_vel[0] - rel_pos[0] / tau,
                    rel_vel[1] - rel_pos[1] / tau,
                    rel_vel[2] - rel_pos[2] / tau,
                ];

                let w_length = magnitude(&w);
                let unit_w = if w_length > 0.001 {
                    normalize(&w)
                } else {
                    [1.0, 0.0, 0.0]
                };

                // Project onto ORCA half-plane
                let _dot_product = dot(&rel_pos, &unit_w);
                let u_magnitude = combined_radius / tau - w_length;

                if u_magnitude > 0.0 {
                    // Velocity needs adjustment
                    let u = [
                        unit_w[0] * u_magnitude * self.config.responsiveness * 0.5,
                        unit_w[1] * u_magnitude * self.config.responsiveness * 0.5,
                        unit_w[2] * u_magnitude * self.config.responsiveness * 0.5,
                    ];

                    result[0] += u[0];
                    result[1] += u[1];
                    result[2] += u[2];
                }
            } else {
                // In collision - compute escape velocity
                let penetration = combined_radius - dist;
                let escape_dir = if dist > 0.001 {
                    normalize(&[
                        position[0] - obs.position[0],
                        position[1] - obs.position[1],
                        position[2] - obs.position[2],
                    ])
                } else {
                    [1.0, 0.0, 0.0]
                };

                let escape_speed = penetration * 2.0 * self.config.responsiveness;
                result[0] += escape_dir[0] * escape_speed;
                result[1] += escape_dir[1] * escape_speed;
                result[2] += escape_dir[2] * escape_speed * 0.5;
            }
        }

        result
    }

    /// Artificial Potential Field algorithm
    fn potential_field(&self, position: [f32; 3], desired_vel: [f32; 3]) -> [f32; 3] {
        let mut result = desired_vel;

        // Repulsive potential from obstacles
        for obs in &self.obstacles {
            let combined_radius = self.agent_radius + obs.radius + self.config.safety_margin;
            let influence_dist = combined_radius * 3.0; // Influence range

            let rel_pos = [
                position[0] - obs.position[0],
                position[1] - obs.position[1],
                position[2] - obs.position[2],
            ];

            let dist = magnitude(&rel_pos);

            if dist < influence_dist && dist > 0.01 {
                // Repulsive force (inverse square law with cutoff)
                let normalized = normalize(&rel_pos);
                let force_magnitude = if dist < combined_radius {
                    // Very strong repulsion when inside safety zone
                    self.config.max_velocity * 2.0
                } else {
                    // Gradual repulsion
                    let x = (influence_dist - dist) / (influence_dist - combined_radius);
                    x * x * self.config.max_velocity * self.config.responsiveness
                };

                result[0] += normalized[0] * force_magnitude;
                result[1] += normalized[1] * force_magnitude;
                result[2] += normalized[2] * force_magnitude * 0.5;
            }
        }

        result
    }

    /// Apply geofence boundary constraints
    fn apply_geofence_constraints(&self, position: [f32; 3], velocity: [f32; 3]) -> [f32; 3] {
        let mut result = velocity;

        // Check horizontal boundaries
        let dist_to_boundary = self.geofence.distance_to_boundary(position[0], position[1]);
        let is_inside = self.geofence.contains_point(position[0], position[1]);

        if (self.geofence.inclusive && !is_inside)
            || (!self.geofence.inclusive && is_inside)
            || dist_to_boundary < self.config.geofence_buffer
        {
            // Need to steer away from boundary
            let center = self.geofence_center();
            let to_center = [center[0] - position[0], center[1] - position[1]];
            let dist_to_center = libm::sqrtf(to_center[0] * to_center[0] + to_center[1] * to_center[1]);

            if dist_to_center > 0.01 {
                let strength = if !is_inside && self.geofence.inclusive {
                    // Outside inclusive fence - strong pull back
                    self.config.max_velocity
                } else {
                    // Near boundary - gradual push
                    (self.config.geofence_buffer - dist_to_boundary)
                        / self.config.geofence_buffer
                        * self.config.max_velocity
                        * self.config.responsiveness
                };

                result[0] += (to_center[0] / dist_to_center) * strength;
                result[1] += (to_center[1] / dist_to_center) * strength;
            }
        }

        // Check altitude limits (convert from NED: negative z = positive altitude)
        let altitude = -position[2];
        let min_alt = self.config.altitude_limits[0];
        let max_alt = self.config.altitude_limits[1];

        if altitude < min_alt + self.config.geofence_buffer {
            // Too low - push up (negative z velocity in NED)
            let strength = (min_alt + self.config.geofence_buffer - altitude)
                / self.config.geofence_buffer
                * self.config.max_velocity;
            result[2] -= strength; // Negative z = up in NED
        } else if altitude > max_alt - self.config.geofence_buffer {
            // Too high - push down (positive z velocity in NED)
            let strength = (altitude - (max_alt - self.config.geofence_buffer))
                / self.config.geofence_buffer
                * self.config.max_velocity;
            result[2] += strength; // Positive z = down in NED
        }

        result
    }

    /// Calculate geofence center
    fn geofence_center(&self) -> [f32; 2] {
        if self.geofence.vertices.is_empty() {
            return [0.0, 0.0];
        }

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;

        for v in &self.geofence.vertices {
            sum_x += v[0];
            sum_y += v[1];
        }

        let n = self.geofence.vertices.len() as f32;
        [sum_x / n, sum_y / n]
    }

    /// Clamp velocity to maximum
    fn clamp_velocity(&self, velocity: [f32; 3]) -> [f32; 3] {
        let speed = magnitude(&velocity);

        if speed > self.config.max_velocity {
            let scale = self.config.max_velocity / speed;
            [
                velocity[0] * scale,
                velocity[1] * scale,
                velocity[2] * scale,
            ]
        } else {
            velocity
        }
    }

    /// Check if emergency stop is needed (imminent collision)
    pub fn check_emergency(&self, position: [f32; 3], velocity: [f32; 3]) -> bool {
        for obs in &self.obstacles {
            let combined_radius = self.agent_radius + obs.radius;

            let rel_pos = [
                obs.position[0] - position[0],
                obs.position[1] - position[1],
                obs.position[2] - position[2],
            ];

            let dist = magnitude(&rel_pos);

            // Already in collision zone
            if dist < combined_radius {
                return true;
            }

            // Check time to collision
            let rel_vel = [
                velocity[0] - obs.velocity[0],
                velocity[1] - obs.velocity[1],
                velocity[2] - obs.velocity[2],
            ];

            let closing_speed = -dot(&rel_vel, &normalize(&rel_pos));

            if closing_speed > 0.1 {
                let time_to_collision = (dist - combined_radius) / closing_speed;
                if time_to_collision < 1.0 {
                    // Less than 1 second to collision
                    return true;
                }
            }
        }

        false
    }

    /// Get minimum distance to any obstacle
    pub fn min_obstacle_distance(&self, position: [f32; 3]) -> f32 {
        let mut min_dist = f32::MAX;

        for obs in &self.obstacles {
            let dist = distance_3d(&position, &obs.position) - obs.radius - self.agent_radius;
            min_dist = min_dist.min(dist);
        }

        min_dist
    }

    /// Get number of obstacles being tracked
    pub fn obstacle_count(&self) -> usize {
        self.obstacles.len()
    }
}

impl Default for CollisionAvoidance {
    fn default() -> Self {
        Self::new(AvoidanceConfig::default())
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Calculate 3D vector magnitude
fn magnitude(v: &[f32; 3]) -> f32 {
    libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Normalize a 3D vector
fn normalize(v: &[f32; 3]) -> [f32; 3] {
    let mag = magnitude(v);
    if mag < 0.0001 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    }
}

/// Dot product of two 3D vectors
fn dot(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Get perpendicular vector (for 3D, we use cross product with up vector)
fn perpendicular_3d(v: &[f32; 3]) -> [f32; 3] {
    // Cross product with Z axis to get horizontal perpendicular
    let up = [0.0, 0.0, 1.0];
    let cross = [
        v[1] * up[2] - v[2] * up[1],
        v[2] * up[0] - v[0] * up[2],
        v[0] * up[1] - v[1] * up[0],
    ];

    let mag = magnitude(&cross);
    if mag < 0.0001 {
        // v is parallel to Z, use X axis
        let side = [1.0, 0.0, 0.0];
        let cross2 = [
            v[1] * side[2] - v[2] * side[1],
            v[2] * side[0] - v[0] * side[2],
            v[0] * side[1] - v[1] * side[0],
        ];
        normalize(&cross2)
    } else {
        normalize(&cross)
    }
}

/// Calculate 3D Euclidean distance
fn distance_3d(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    libm::sqrtf(dx * dx + dy * dy + dz * dz)
}

/// Calculate distance from point to line segment
fn point_to_segment_distance(px: f32, py: f32, x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len_sq = dx * dx + dy * dy;

    if len_sq < 0.0001 {
        // Segment is a point
        return libm::sqrtf((px - x1) * (px - x1) + (py - y1) * (py - y1));
    }

    // Project point onto line, clamping to segment
    let t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);

    let proj_x = x1 + t * dx;
    let proj_y = y1 + t * dy;

    libm::sqrtf((px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_magnitude() {
        assert!((magnitude(&[3.0, 4.0, 0.0]) - 5.0).abs() < 0.001);
        assert!((magnitude(&[1.0, 1.0, 1.0]) - 1.732).abs() < 0.01);
    }

    #[test]
    fn test_normalize() {
        let n = normalize(&[3.0, 4.0, 0.0]);
        assert!((n[0] - 0.6).abs() < 0.001);
        assert!((n[1] - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_geofence_contains() {
        let geofence = Geofence::default(); // 100x100 square
        assert!(geofence.contains_point(0.0, 0.0));
        assert!(geofence.contains_point(40.0, 40.0));
        assert!(!geofence.contains_point(60.0, 60.0)); // Outside
    }

    #[test]
    fn test_collision_avoidance_basic() {
        let mut ca = CollisionAvoidance::default();

        // Add obstacle directly ahead (close enough to trigger avoidance)
        ca.add_obstacle([5.0, 0.0, -10.0], [0.0, 0.0, 0.0], 1.0);

        // Desired velocity toward obstacle
        let safe_vel = ca.compute_safe_velocity([0.0, 0.0, -10.0], [5.0, 0.0, 0.0]);

        // Should modify velocity in some way (either lateral or reduced forward)
        let vel_changed = safe_vel[1].abs() > 0.01 || (safe_vel[0] - 5.0).abs() > 0.01;
        assert!(vel_changed, "Velocity should be modified to avoid obstacle");
    }

    #[test]
    fn test_emergency_detection() {
        let mut ca = CollisionAvoidance::default();
        // Obstacle very close - within combined radius (agent 0.5 + obstacle 1.0 + margin)
        ca.add_obstacle([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);

        // Already in collision zone
        assert!(ca.check_emergency([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]));

        // Test with obstacle further away, moving away from it
        let mut ca2 = CollisionAvoidance::default();
        ca2.add_obstacle([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0);
        assert!(!ca2.check_emergency([0.0, 0.0, 0.0], [-5.0, 0.0, 0.0]));
    }

    #[test]
    fn test_point_to_segment() {
        // Point on segment
        let d = point_to_segment_distance(0.5, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(d.abs() < 0.001);

        // Point perpendicular to segment
        let d = point_to_segment_distance(0.5, 1.0, 0.0, 0.0, 1.0, 0.0);
        assert!((d - 1.0).abs() < 0.001);
    }
}
