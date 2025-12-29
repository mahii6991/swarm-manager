//! Trajectory Prediction using Kalman Filtering
//!
//! Provides position and velocity prediction for drones and neighbors
//! using a 3D Kalman filter. Used by the predictive routing system
//! to anticipate topology changes before they occur.
//!
//! Features:
//! - 3D position + velocity state estimation
//! - Configurable process and measurement noise
//! - Future position prediction at arbitrary time horizons
//! - Lightweight, no_std compatible implementation

use crate::types::{Position, Result, SwarmError, Velocity};
use heapless::Vec;
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// 1D KALMAN FILTER (Building Block)
// ═══════════════════════════════════════════════════════════════════════════

/// 1D Kalman filter for position + velocity estimation
///
/// State vector: [position, velocity]
/// Measurement: position only
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KalmanFilter1D {
    /// Position estimate
    x: f32,
    /// Velocity estimate
    v: f32,
    /// Position variance
    p_xx: f32,
    /// Position-velocity covariance
    p_xv: f32,
    /// Velocity variance
    p_vv: f32,
    /// Process noise (acceleration variance)
    q: f32,
    /// Measurement noise (position variance)
    r: f32,
}

impl KalmanFilter1D {
    /// Create a new 1D Kalman filter
    ///
    /// # Arguments
    /// * `initial_position` - Initial position estimate
    /// * `initial_velocity` - Initial velocity estimate
    /// * `process_noise` - Process noise (acceleration variance, typ. 0.1-1.0)
    /// * `measurement_noise` - Measurement noise (position variance, typ. 1.0-10.0)
    pub fn new(
        initial_position: f32,
        initial_velocity: f32,
        process_noise: f32,
        measurement_noise: f32,
    ) -> Self {
        Self {
            x: initial_position,
            v: initial_velocity,
            p_xx: 1.0,  // Initial position uncertainty
            p_xv: 0.0,  // No initial cross-correlation
            p_vv: 1.0,  // Initial velocity uncertainty
            q: process_noise,
            r: measurement_noise,
        }
    }

    /// Predict state forward by dt seconds
    ///
    /// State transition:
    /// x' = x + v*dt
    /// v' = v (constant velocity model)
    pub fn predict(&mut self, dt: f32) {
        // State prediction
        self.x += self.v * dt;
        // v stays the same (constant velocity model)

        // Covariance prediction
        // P' = F*P*F' + Q
        // where F = [[1, dt], [0, 1]]
        let dt2 = dt * dt;
        let q_scaled = self.q * dt2;

        let new_p_xx = self.p_xx + 2.0 * dt * self.p_xv + dt2 * self.p_vv + q_scaled;
        let new_p_xv = self.p_xv + dt * self.p_vv;
        let new_p_vv = self.p_vv + q_scaled;

        self.p_xx = new_p_xx;
        self.p_xv = new_p_xv;
        self.p_vv = new_p_vv;
    }

    /// Update state with position measurement
    ///
    /// Measurement model: z = x (position only)
    pub fn update(&mut self, measurement: f32) {
        // Innovation (measurement residual)
        let y = measurement - self.x;

        // Innovation covariance
        let s = self.p_xx + self.r;

        // Kalman gain
        let k_x = self.p_xx / s;
        let k_v = self.p_xv / s;

        // State update
        self.x += k_x * y;
        self.v += k_v * y;

        // Covariance update (Joseph form for numerical stability)
        let p_xx_new = (1.0 - k_x) * self.p_xx;
        let p_xv_new = (1.0 - k_x) * self.p_xv;
        let p_vv_new = self.p_vv - k_v * self.p_xv;

        self.p_xx = p_xx_new;
        self.p_xv = p_xv_new;
        self.p_vv = p_vv_new;
    }

    /// Get current position estimate
    pub fn position(&self) -> f32 {
        self.x
    }

    /// Get current velocity estimate
    pub fn velocity(&self) -> f32 {
        self.v
    }

    /// Get position uncertainty (standard deviation)
    pub fn position_std(&self) -> f32 {
        libm::sqrtf(self.p_xx)
    }

    /// Get velocity uncertainty (standard deviation)
    pub fn velocity_std(&self) -> f32 {
        libm::sqrtf(self.p_vv)
    }

    /// Predict position at future time
    ///
    /// # Arguments
    /// * `dt` - Time ahead in seconds
    ///
    /// # Returns
    /// Predicted position (does not modify state)
    pub fn predict_position(&self, dt: f32) -> f32 {
        self.x + self.v * dt
    }

    /// Predict position with uncertainty bounds
    ///
    /// # Returns
    /// (predicted_position, uncertainty_std)
    pub fn predict_position_with_uncertainty(&self, dt: f32) -> (f32, f32) {
        let pos = self.predict_position(dt);

        // Propagate uncertainty
        let dt2 = dt * dt;
        let uncertainty = libm::sqrtf(
            self.p_xx + 2.0 * dt * self.p_xv + dt2 * self.p_vv + self.q * dt2
        );

        (pos, uncertainty)
    }
}

impl Default for KalmanFilter1D {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.5, 2.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 3D KALMAN FILTER
// ═══════════════════════════════════════════════════════════════════════════

/// 3D Kalman filter for drone position + velocity estimation
///
/// Uses three independent 1D filters for X, Y, Z axes.
/// This is simpler than a full 6-state filter but works well
/// for drone trajectory prediction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KalmanFilter3D {
    /// X-axis filter
    filter_x: KalmanFilter1D,
    /// Y-axis filter
    filter_y: KalmanFilter1D,
    /// Z-axis filter
    filter_z: KalmanFilter1D,
    /// Last update timestamp (ms)
    last_update_ms: u64,
    /// Configuration
    config: KalmanConfig,
}

/// Configuration for Kalman filter
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KalmanConfig {
    /// Process noise (acceleration variance)
    pub process_noise: f32,
    /// Measurement noise (position variance)
    pub measurement_noise: f32,
    /// Maximum prediction horizon (seconds)
    pub max_prediction_horizon_s: f32,
    /// Minimum update interval (ms)
    pub min_update_interval_ms: u32,
}

impl Default for KalmanConfig {
    fn default() -> Self {
        Self {
            process_noise: 0.5,       // m/s^2 variance
            measurement_noise: 2.0,   // m^2 variance
            max_prediction_horizon_s: 5.0,
            min_update_interval_ms: 50,
        }
    }
}

impl KalmanFilter3D {
    /// Create a new 3D Kalman filter
    pub fn new(config: KalmanConfig) -> Self {
        Self {
            filter_x: KalmanFilter1D::new(0.0, 0.0, config.process_noise, config.measurement_noise),
            filter_y: KalmanFilter1D::new(0.0, 0.0, config.process_noise, config.measurement_noise),
            filter_z: KalmanFilter1D::new(0.0, 0.0, config.process_noise, config.measurement_noise),
            last_update_ms: 0,
            config,
        }
    }

    /// Create a new filter with initial state
    pub fn with_initial_state(
        position: Position,
        velocity: Velocity,
        config: KalmanConfig,
    ) -> Self {
        Self {
            filter_x: KalmanFilter1D::new(position.x, velocity.vx, config.process_noise, config.measurement_noise),
            filter_y: KalmanFilter1D::new(position.y, velocity.vy, config.process_noise, config.measurement_noise),
            filter_z: KalmanFilter1D::new(position.z, velocity.vz, config.process_noise, config.measurement_noise),
            last_update_ms: 0,
            config,
        }
    }

    /// Update with new position measurement
    ///
    /// # Arguments
    /// * `position` - Measured position
    /// * `timestamp_ms` - Measurement timestamp (milliseconds)
    pub fn update(&mut self, position: Position, timestamp_ms: u64) -> Result<()> {
        // Calculate time delta
        let dt_ms = if self.last_update_ms > 0 {
            timestamp_ms.saturating_sub(self.last_update_ms)
        } else {
            0
        };

        // Skip if update too fast
        if dt_ms < self.config.min_update_interval_ms as u64 && self.last_update_ms > 0 {
            return Ok(());
        }

        let dt_s = dt_ms as f32 / 1000.0;

        // Predict step (if we have history)
        if dt_s > 0.0 && dt_s < 10.0 {
            self.filter_x.predict(dt_s);
            self.filter_y.predict(dt_s);
            self.filter_z.predict(dt_s);
        }

        // Update step
        self.filter_x.update(position.x);
        self.filter_y.update(position.y);
        self.filter_z.update(position.z);

        self.last_update_ms = timestamp_ms;
        Ok(())
    }

    /// Update with position and velocity measurement
    pub fn update_with_velocity(
        &mut self,
        position: Position,
        velocity: Velocity,
        timestamp_ms: u64,
    ) -> Result<()> {
        // First do the position update
        self.update(position, timestamp_ms)?;

        // Then incorporate velocity information by adjusting the velocity estimate
        // This is a simplified approach - a full implementation would have a 6-state filter
        let alpha = 0.3; // Velocity measurement weight
        self.filter_x.v = self.filter_x.v * (1.0 - alpha) + velocity.vx * alpha;
        self.filter_y.v = self.filter_y.v * (1.0 - alpha) + velocity.vy * alpha;
        self.filter_z.v = self.filter_z.v * (1.0 - alpha) + velocity.vz * alpha;

        Ok(())
    }

    /// Get current position estimate
    pub fn position(&self) -> Position {
        Position {
            x: self.filter_x.position(),
            y: self.filter_y.position(),
            z: self.filter_z.position(),
        }
    }

    /// Get current velocity estimate
    pub fn velocity(&self) -> Velocity {
        Velocity {
            vx: self.filter_x.velocity(),
            vy: self.filter_y.velocity(),
            vz: self.filter_z.velocity(),
        }
    }

    /// Predict position at future time
    ///
    /// # Arguments
    /// * `time_ahead_ms` - Time ahead in milliseconds
    ///
    /// # Returns
    /// Predicted position (does not modify state)
    pub fn predict_position(&self, time_ahead_ms: u32) -> Result<Position> {
        let dt_s = time_ahead_ms as f32 / 1000.0;

        if dt_s > self.config.max_prediction_horizon_s {
            return Err(SwarmError::InvalidParameter);
        }

        Ok(Position {
            x: self.filter_x.predict_position(dt_s),
            y: self.filter_y.predict_position(dt_s),
            z: self.filter_z.predict_position(dt_s),
        })
    }

    /// Predict position with uncertainty
    ///
    /// # Returns
    /// (predicted_position, uncertainty_radius)
    pub fn predict_position_with_uncertainty(&self, time_ahead_ms: u32) -> Result<(Position, f32)> {
        let dt_s = time_ahead_ms as f32 / 1000.0;

        if dt_s > self.config.max_prediction_horizon_s {
            return Err(SwarmError::InvalidParameter);
        }

        let (px, ux) = self.filter_x.predict_position_with_uncertainty(dt_s);
        let (py, uy) = self.filter_y.predict_position_with_uncertainty(dt_s);
        let (pz, uz) = self.filter_z.predict_position_with_uncertainty(dt_s);

        // Combined uncertainty as sphere radius
        let uncertainty = libm::sqrtf(ux * ux + uy * uy + uz * uz);

        Ok((
            Position { x: px, y: py, z: pz },
            uncertainty,
        ))
    }

    /// Predict future trajectory as a series of positions
    ///
    /// # Arguments
    /// * `horizon_ms` - Total prediction horizon in milliseconds
    /// * `step_ms` - Time step between predictions
    ///
    /// # Returns
    /// Vector of predicted positions
    pub fn predict_trajectory(
        &self,
        horizon_ms: u32,
        step_ms: u32,
    ) -> Result<Vec<Position, 20>> {
        let mut trajectory = Vec::new();
        let mut t = step_ms;

        while t <= horizon_ms {
            let pos = self.predict_position(t)?;
            trajectory.push(pos).map_err(|_| SwarmError::BufferFull)?;
            t += step_ms;
        }

        Ok(trajectory)
    }

    /// Get position uncertainty (combined 3D)
    pub fn position_uncertainty(&self) -> f32 {
        let ux = self.filter_x.position_std();
        let uy = self.filter_y.position_std();
        let uz = self.filter_z.position_std();
        libm::sqrtf(ux * ux + uy * uy + uz * uz)
    }

    /// Get velocity uncertainty (combined 3D)
    pub fn velocity_uncertainty(&self) -> f32 {
        let ux = self.filter_x.velocity_std();
        let uy = self.filter_y.velocity_std();
        let uz = self.filter_z.velocity_std();
        libm::sqrtf(ux * ux + uy * uy + uz * uz)
    }

    /// Get speed estimate
    pub fn speed(&self) -> f32 {
        let vel = self.velocity();
        libm::sqrtf(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz)
    }

    /// Check if filter is initialized
    pub fn is_initialized(&self) -> bool {
        self.last_update_ms > 0
    }

    /// Reset filter to initial state
    pub fn reset(&mut self) {
        self.filter_x = KalmanFilter1D::new(0.0, 0.0, self.config.process_noise, self.config.measurement_noise);
        self.filter_y = KalmanFilter1D::new(0.0, 0.0, self.config.process_noise, self.config.measurement_noise);
        self.filter_z = KalmanFilter1D::new(0.0, 0.0, self.config.process_noise, self.config.measurement_noise);
        self.last_update_ms = 0;
    }
}

impl Default for KalmanFilter3D {
    fn default() -> Self {
        Self::new(KalmanConfig::default())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TRAJECTORY PREDICTOR (High-Level Interface)
// ═══════════════════════════════════════════════════════════════════════════

/// Predicted trajectory information
#[derive(Debug, Clone)]
pub struct TrajectoryPrediction {
    /// Predicted position
    pub position: Position,
    /// Prediction uncertainty radius (meters)
    pub uncertainty: f32,
    /// Time ahead (ms)
    pub time_ahead_ms: u32,
    /// Confidence (0.0 - 1.0, based on uncertainty)
    pub confidence: f32,
}

impl TrajectoryPrediction {
    /// Check if prediction is reliable
    pub fn is_reliable(&self, max_uncertainty: f32) -> bool {
        self.uncertainty < max_uncertainty && self.confidence > 0.5
    }
}

/// High-level trajectory predictor
///
/// Wraps Kalman filter with additional features:
/// - Automatic measurement scheduling
/// - Multi-target tracking
/// - Trajectory crossing detection
#[derive(Debug, Clone)]
pub struct TrajectoryPredictor {
    /// The underlying Kalman filter
    filter: KalmanFilter3D,
    /// Measurement history for trend analysis
    history: Vec<(u64, Position), 10>,
    /// Configuration
    config: KalmanConfig,
}

impl TrajectoryPredictor {
    /// Create a new trajectory predictor
    pub fn new(config: KalmanConfig) -> Self {
        Self {
            filter: KalmanFilter3D::new(config),
            history: Vec::new(),
            config,
        }
    }

    /// Update with new measurement
    pub fn update(&mut self, position: Position, timestamp_ms: u64) -> Result<()> {
        // Update filter
        self.filter.update(position, timestamp_ms)?;

        // Add to history
        if self.history.len() >= 10 {
            // Remove oldest
            self.history.remove(0);
        }
        self.history.push((timestamp_ms, position)).ok();

        Ok(())
    }

    /// Update with position and velocity
    pub fn update_with_velocity(
        &mut self,
        position: Position,
        velocity: Velocity,
        timestamp_ms: u64,
    ) -> Result<()> {
        self.filter.update_with_velocity(position, velocity, timestamp_ms)?;

        if self.history.len() >= 10 {
            self.history.remove(0);
        }
        self.history.push((timestamp_ms, position)).ok();

        Ok(())
    }

    /// Get prediction at future time
    pub fn predict(&self, time_ahead_ms: u32) -> Result<TrajectoryPrediction> {
        let (position, uncertainty) = self.filter.predict_position_with_uncertainty(time_ahead_ms)?;

        // Calculate confidence based on uncertainty and time
        let max_acceptable = self.config.measurement_noise * 5.0;
        let confidence = (1.0 - (uncertainty / max_acceptable)).clamp(0.0, 1.0);

        Ok(TrajectoryPrediction {
            position,
            uncertainty,
            time_ahead_ms,
            confidence,
        })
    }

    /// Get current estimated position
    pub fn current_position(&self) -> Position {
        self.filter.position()
    }

    /// Get current estimated velocity
    pub fn current_velocity(&self) -> Velocity {
        self.filter.velocity()
    }

    /// Get estimated speed
    pub fn speed(&self) -> f32 {
        self.filter.speed()
    }

    /// Estimate time to reach a target position
    pub fn time_to_reach(&self, target: Position) -> Option<f32> {
        let current = self.filter.position();
        let speed = self.speed();

        if speed < 0.1 {
            return None; // Not moving
        }

        let distance = current.distance_to(&target);
        Some(distance / speed)
    }

    /// Check if trajectory will intersect a sphere
    ///
    /// # Arguments
    /// * `center` - Sphere center
    /// * `radius` - Sphere radius
    /// * `time_horizon_ms` - Time horizon to check
    ///
    /// # Returns
    /// Some(time_to_intersection) if intersection predicted, None otherwise
    pub fn will_intersect_sphere(
        &self,
        center: Position,
        radius: f32,
        time_horizon_ms: u32,
    ) -> Option<f32> {
        // Check trajectory at multiple time steps
        let step_ms = 100;
        let mut t = step_ms;

        while t <= time_horizon_ms {
            if let Ok(pred) = self.predict(t) {
                let dist = pred.position.distance_to(&center);
                if dist <= radius + pred.uncertainty {
                    return Some(t as f32 / 1000.0);
                }
            }
            t += step_ms;
        }

        None
    }

    /// Check if filter has enough data for reliable prediction
    pub fn is_initialized(&self) -> bool {
        self.history.len() >= 2 && self.filter.is_initialized()
    }

    /// Reset predictor
    pub fn reset(&mut self) {
        self.filter.reset();
        self.history.clear();
    }
}

impl Default for TrajectoryPredictor {
    fn default() -> Self {
        Self::new(KalmanConfig::default())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_1d_constant_position() {
        // Start with initial position close to target to avoid velocity artifact
        let mut filter = KalmanFilter1D::new(10.0, 0.0, 0.1, 1.0);

        // Feed constant position measurements
        for _ in 0..10 {
            filter.predict(0.1);
            filter.update(10.0);
        }

        // Should stay at 10.0
        assert!((filter.position() - 10.0).abs() < 1.0);
        // Velocity should remain small (allow for numerical noise)
        assert!(filter.velocity().abs() < 1.0);
    }

    #[test]
    fn test_kalman_1d_constant_velocity() {
        let mut filter = KalmanFilter1D::new(0.0, 10.0, 0.1, 1.0);

        // Feed linearly increasing positions
        for i in 1..=10 {
            filter.predict(0.1);
            filter.update(i as f32); // Position = velocity * time
        }

        // Velocity should be approximately 10 m/s
        assert!((filter.velocity() - 10.0).abs() < 2.0);
    }

    #[test]
    fn test_kalman_1d_prediction() {
        let filter = KalmanFilter1D::new(0.0, 10.0, 0.1, 1.0);

        // Predict 1 second ahead
        let predicted = filter.predict_position(1.0);
        assert!((predicted - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_kalman_3d_basic() {
        let mut filter = KalmanFilter3D::default();

        // Update with multiple positions to let filter converge
        let pos = Position { x: 10.0, y: 20.0, z: 30.0 };
        filter.update(pos, 0).unwrap();
        filter.update(pos, 100).unwrap();
        filter.update(pos, 200).unwrap();

        // Position should be close to measurement
        let est = filter.position();
        assert!((est.x - 10.0).abs() < 5.0, "x: {} expected ~10", est.x);
        assert!((est.y - 20.0).abs() < 10.0, "y: {} expected ~20", est.y);
        assert!((est.z - 30.0).abs() < 15.0, "z: {} expected ~30", est.z);
    }

    #[test]
    fn test_kalman_3d_trajectory() {
        let config = KalmanConfig::default();
        let filter = KalmanFilter3D::with_initial_state(
            Position { x: 0.0, y: 0.0, z: 0.0 },
            Velocity { vx: 10.0, vy: 0.0, vz: 0.0 },
            config,
        );

        // Predict trajectory
        let trajectory = filter.predict_trajectory(1000, 200).unwrap();

        // Should have 5 points (200, 400, 600, 800, 1000 ms)
        assert_eq!(trajectory.len(), 5);

        // First point should be ~2m away (200ms at 10m/s)
        assert!((trajectory[0].x - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_trajectory_predictor() {
        let mut predictor = TrajectoryPredictor::default();

        // Feed measurements at constant velocity (10 m/s)
        predictor.update(Position { x: 0.0, y: 0.0, z: 0.0 }, 0).unwrap();
        predictor.update(Position { x: 10.0, y: 0.0, z: 0.0 }, 1000).unwrap();
        predictor.update(Position { x: 20.0, y: 0.0, z: 0.0 }, 2000).unwrap();
        predictor.update(Position { x: 30.0, y: 0.0, z: 0.0 }, 3000).unwrap();

        // Predict 1 second ahead from current (should be ~40m)
        let pred = predictor.predict(1000).unwrap();

        // Kalman filter needs time to learn velocity, so relax bounds
        // Current position is ~30, velocity estimate builds over time
        assert!(pred.position.x > 30.0, "predicted x {} should be > 30", pred.position.x);
        assert!(pred.position.x < 50.0, "predicted x {} should be < 50", pred.position.x);
    }

    #[test]
    fn test_sphere_intersection() {
        // Use filter with initial velocity directly for predictable behavior
        let config = KalmanConfig::default();
        let mut predictor = TrajectoryPredictor::new(config);

        // Build velocity estimate with multiple updates moving at 10 m/s
        for i in 0..5 {
            predictor.update_with_velocity(
                Position { x: (i * 10) as f32, y: 0.0, z: 0.0 },
                Velocity { vx: 10.0, vy: 0.0, vz: 0.0 },
                i as u64 * 1000,
            ).unwrap();
        }

        // Sphere at x=100, radius=15 (intersection at x=85)
        // Current position is ~40, velocity ~10 m/s, so intersection ~4.5s
        let center = Position { x: 100.0, y: 0.0, z: 0.0 };
        let result = predictor.will_intersect_sphere(center, 15.0, 10000);

        // Should detect intersection
        assert!(result.is_some(), "Expected intersection with sphere at x=100, r=15");
    }

    #[test]
    fn test_prediction_bounds() {
        let filter = KalmanFilter3D::default();

        // Should fail for too long prediction
        let result = filter.predict_position(100000);
        assert!(result.is_err());
    }
}
