//! Velocity Controller for Drone Movement
//!
//! Computes desired velocities for navigation and formation keeping.

use crate::types::{Position, Velocity};

/// Velocity controller for drone navigation
#[derive(Debug, Clone)]
pub struct VelocityController {
    /// Maximum speed (m/s)
    max_speed: f32,
    /// Proportional gain for position control
    position_gain: f32,
    /// Velocity damping factor
    damping: f32,
    /// Minimum speed threshold (below this, set to zero)
    min_speed_threshold: f32,
}

impl VelocityController {
    /// Create new velocity controller with default parameters
    pub fn new(max_speed: f32) -> Self {
        Self {
            max_speed,
            position_gain: 0.5,
            damping: 0.8,
            min_speed_threshold: 0.01,
        }
    }

    /// Create with custom parameters
    pub fn with_params(
        max_speed: f32,
        position_gain: f32,
        damping: f32,
        min_speed_threshold: f32,
    ) -> Self {
        Self {
            max_speed,
            position_gain,
            damping,
            min_speed_threshold,
        }
    }

    /// Set maximum speed
    pub fn set_max_speed(&mut self, max_speed: f32) {
        self.max_speed = max_speed;
    }

    /// Get maximum speed
    pub fn max_speed(&self) -> f32 {
        self.max_speed
    }

    /// Set position gain
    pub fn set_position_gain(&mut self, gain: f32) {
        self.position_gain = gain;
    }

    /// Compute velocity to reach destination
    pub fn compute_destination_velocity(
        &self,
        current: &Position,
        destination: &Position,
    ) -> Velocity {
        let dx = destination.x - current.x;
        let dy = destination.y - current.y;
        let dz = destination.z - current.z;

        let distance = libm::sqrtf(dx * dx + dy * dy + dz * dz);

        if distance < self.min_speed_threshold {
            return Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            };
        }

        // Proportional control with saturation
        let speed = (distance * self.position_gain).min(self.max_speed);

        Velocity {
            vx: (dx / distance) * speed,
            vy: (dy / distance) * speed,
            vz: (dz / distance) * speed,
        }
    }

    /// Combine multiple velocity commands with weighting
    pub fn combine_velocities(&self, velocities: &[(Velocity, f32)]) -> Velocity {
        let mut total_vx = 0.0f32;
        let mut total_vy = 0.0f32;
        let mut total_vz = 0.0f32;
        let mut total_weight = 0.0f32;

        for (vel, weight) in velocities {
            total_vx += vel.vx * weight;
            total_vy += vel.vy * weight;
            total_vz += vel.vz * weight;
            total_weight += weight;
        }

        if total_weight < 0.001 {
            return Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            };
        }

        let vx = total_vx / total_weight;
        let vy = total_vy / total_weight;
        let vz = total_vz / total_weight;

        self.clamp_velocity(Velocity { vx, vy, vz })
    }

    /// Clamp velocity to maximum speed
    pub fn clamp_velocity(&self, vel: Velocity) -> Velocity {
        let speed = libm::sqrtf(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz);

        if speed > self.max_speed {
            let scale = self.max_speed / speed;
            Velocity {
                vx: vel.vx * scale,
                vy: vel.vy * scale,
                vz: vel.vz * scale,
            }
        } else if speed < self.min_speed_threshold {
            Velocity {
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
            }
        } else {
            vel
        }
    }

    /// Apply damping to current velocity
    pub fn apply_damping(&self, vel: Velocity) -> Velocity {
        Velocity {
            vx: vel.vx * self.damping,
            vy: vel.vy * self.damping,
            vz: vel.vz * self.damping,
        }
    }

    /// Compute velocity with smooth acceleration
    pub fn smooth_velocity(
        &self,
        current_vel: &Velocity,
        target_vel: &Velocity,
        dt: f32,
        max_accel: f32,
    ) -> Velocity {
        let dvx = target_vel.vx - current_vel.vx;
        let dvy = target_vel.vy - current_vel.vy;
        let dvz = target_vel.vz - current_vel.vz;

        let dv_mag = libm::sqrtf(dvx * dvx + dvy * dvy + dvz * dvz);
        let max_dv = max_accel * dt;

        if dv_mag <= max_dv || dv_mag < 0.001 {
            // Can reach target velocity directly
            self.clamp_velocity(*target_vel)
        } else {
            // Limit acceleration
            let scale = max_dv / dv_mag;
            self.clamp_velocity(Velocity {
                vx: current_vel.vx + dvx * scale,
                vy: current_vel.vy + dvy * scale,
                vz: current_vel.vz + dvz * scale,
            })
        }
    }

    /// Compute escape velocity away from a position
    pub fn compute_escape_velocity(
        &self,
        current: &Position,
        threat: &Position,
        escape_speed: f32,
    ) -> Velocity {
        let dx = current.x - threat.x;
        let dy = current.y - threat.y;
        let dz = current.z - threat.z;

        let distance = libm::sqrtf(dx * dx + dy * dy + dz * dz);

        if distance < 0.01 {
            // At threat position, escape in positive x direction
            return Velocity {
                vx: escape_speed,
                vy: 0.0,
                vz: 0.0,
            };
        }

        let speed = escape_speed.min(self.max_speed);

        Velocity {
            vx: (dx / distance) * speed,
            vy: (dy / distance) * speed,
            vz: (dz / distance) * speed,
        }
    }

    /// Calculate velocity magnitude
    pub fn velocity_magnitude(vel: &Velocity) -> f32 {
        libm::sqrtf(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz)
    }

    /// Check if velocity is effectively zero
    pub fn is_stationary(&self, vel: &Velocity) -> bool {
        Self::velocity_magnitude(vel) < self.min_speed_threshold
    }
}

impl Default for VelocityController {
    fn default() -> Self {
        Self::new(10.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_destination_velocity() {
        let ctrl = VelocityController::new(10.0);
        let current = Position { x: 0.0, y: 0.0, z: 10.0 };
        let dest = Position { x: 10.0, y: 0.0, z: 10.0 };

        let vel = ctrl.compute_destination_velocity(&current, &dest);

        assert!(vel.vx > 0.0);
        assert!(vel.vy.abs() < 0.01);
        assert!(vel.vz.abs() < 0.01);
    }

    #[test]
    fn test_velocity_clamping() {
        let ctrl = VelocityController::new(5.0);
        let vel = Velocity { vx: 10.0, vy: 0.0, vz: 0.0 };

        let clamped = ctrl.clamp_velocity(vel);

        assert!((clamped.vx - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_combine_velocities() {
        let ctrl = VelocityController::new(10.0);
        let velocities = [
            (Velocity { vx: 5.0, vy: 0.0, vz: 0.0 }, 1.0),
            (Velocity { vx: 0.0, vy: 5.0, vz: 0.0 }, 1.0),
        ];

        let combined = ctrl.combine_velocities(&velocities);

        assert!((combined.vx - 2.5).abs() < 0.01);
        assert!((combined.vy - 2.5).abs() < 0.01);
    }

    #[test]
    fn test_escape_velocity() {
        let ctrl = VelocityController::new(10.0);
        let current = Position { x: 10.0, y: 0.0, z: 10.0 };
        let threat = Position { x: 0.0, y: 0.0, z: 10.0 };

        let vel = ctrl.compute_escape_velocity(&current, &threat, 5.0);

        // Should escape in positive x direction (away from threat)
        assert!(vel.vx > 0.0);
    }

    #[test]
    fn test_stationary_check() {
        let ctrl = VelocityController::new(10.0);

        assert!(ctrl.is_stationary(&Velocity { vx: 0.0, vy: 0.0, vz: 0.0 }));
        assert!(!ctrl.is_stationary(&Velocity { vx: 1.0, vy: 0.0, vz: 0.0 }));
    }
}
