//! Link Quality Prediction
//!
//! Predicts wireless link quality degradation and disconnection
//! using trend analysis and exponential smoothing. Used by the
//! proactive routing system to pre-establish alternative routes
//! before link failures occur.
//!
//! Features:
//! - RSSI trend analysis
//! - Link quality extrapolation
//! - Disconnect time estimation
//! - History-based confidence scoring

use crate::types::{DroneId, Position, Result, SwarmError};
use heapless::Vec;
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for link prediction
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LinkPredictorConfig {
    /// History size for trend analysis
    pub history_size: usize,
    /// Exponential smoothing alpha (0.0-1.0)
    pub smoothing_alpha: f32,
    /// Minimum RSSI for viable link (dBm)
    pub min_rssi_dbm: i8,
    /// Minimum link quality threshold (0.0-1.0)
    pub min_quality_threshold: f32,
    /// Warning threshold for quality (0.0-1.0)
    pub warning_quality_threshold: f32,
    /// Maximum prediction horizon (ms)
    pub max_prediction_horizon_ms: u32,
}

impl Default for LinkPredictorConfig {
    fn default() -> Self {
        Self {
            history_size: 20,
            smoothing_alpha: 0.3,
            min_rssi_dbm: -85,
            min_quality_threshold: 0.2,
            warning_quality_threshold: 0.4,
            max_prediction_horizon_ms: 10000,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK QUALITY TREND
// ═══════════════════════════════════════════════════════════════════════════

/// Link quality trend direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinkTrend {
    /// Link quality is improving
    Improving,
    /// Link quality is stable
    Stable,
    /// Link quality is degrading
    Degrading,
    /// Link quality is rapidly degrading (critical)
    Critical,
    /// Unknown (insufficient data)
    Unknown,
}

impl LinkTrend {
    /// Get trend from rate of change
    pub fn from_rate(rate_per_second: f32) -> Self {
        if rate_per_second > 0.02 {
            LinkTrend::Improving
        } else if rate_per_second > -0.02 {
            LinkTrend::Stable
        } else if rate_per_second > -0.1 {
            LinkTrend::Degrading
        } else {
            LinkTrend::Critical
        }
    }

    /// Check if link is at risk
    pub fn is_at_risk(&self) -> bool {
        matches!(self, LinkTrend::Degrading | LinkTrend::Critical)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK MEASUREMENT
// ═══════════════════════════════════════════════════════════════════════════

/// Single link quality measurement
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LinkMeasurement {
    /// Timestamp (ms)
    pub timestamp_ms: u64,
    /// RSSI in dBm
    pub rssi_dbm: i8,
    /// Link quality (0.0-1.0)
    pub quality: f32,
    /// Neighbor position (optional)
    pub position: Option<Position>,
    /// Round-trip time (ms)
    pub rtt_ms: Option<u32>,
}

impl LinkMeasurement {
    /// Create from basic measurements
    pub fn new(timestamp_ms: u64, rssi_dbm: i8, quality: f32) -> Self {
        Self {
            timestamp_ms,
            rssi_dbm,
            quality,
            position: None,
            rtt_ms: None,
        }
    }

    /// Create with position
    pub fn with_position(mut self, position: Position) -> Self {
        self.position = Some(position);
        self
    }

    /// Create with RTT
    pub fn with_rtt(mut self, rtt_ms: u32) -> Self {
        self.rtt_ms = Some(rtt_ms);
        self
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK PREDICTOR
// ═══════════════════════════════════════════════════════════════════════════

/// Link quality predictor for a single neighbor
#[derive(Debug, Clone)]
pub struct LinkPredictor {
    /// Neighbor drone ID
    neighbor_id: DroneId,
    /// Measurement history
    history: Vec<LinkMeasurement, 20>,
    /// Smoothed RSSI value
    smoothed_rssi: f32,
    /// Smoothed quality value
    smoothed_quality: f32,
    /// RSSI trend (change per second)
    rssi_trend: f32,
    /// Quality trend (change per second)
    quality_trend: f32,
    /// Configuration
    config: LinkPredictorConfig,
    /// Last update timestamp
    last_update_ms: u64,
}

impl LinkPredictor {
    /// Create a new link predictor for a neighbor
    pub fn new(neighbor_id: DroneId, config: LinkPredictorConfig) -> Self {
        Self {
            neighbor_id,
            history: Vec::new(),
            smoothed_rssi: -50.0, // Assume good initial RSSI
            smoothed_quality: 1.0,
            rssi_trend: 0.0,
            quality_trend: 0.0,
            config,
            last_update_ms: 0,
        }
    }

    /// Create with default config
    pub fn new_default(neighbor_id: DroneId) -> Self {
        Self::new(neighbor_id, LinkPredictorConfig::default())
    }

    /// Get neighbor ID
    pub fn neighbor_id(&self) -> DroneId {
        self.neighbor_id
    }

    /// Update with new measurement
    pub fn update(&mut self, measurement: LinkMeasurement) -> Result<()> {
        let timestamp = measurement.timestamp_ms;
        let rssi = measurement.rssi_dbm as f32;
        let quality = measurement.quality;

        // Calculate time delta
        let dt_ms = if self.last_update_ms > 0 {
            timestamp.saturating_sub(self.last_update_ms)
        } else {
            0
        };
        let dt_s = dt_ms as f32 / 1000.0;

        // Update trends if we have history
        if dt_s > 0.0 && dt_s < 10.0 {
            // RSSI trend
            let rssi_delta = rssi - self.smoothed_rssi;
            let new_rssi_trend = rssi_delta / dt_s;
            self.rssi_trend = self.rssi_trend * (1.0 - self.config.smoothing_alpha)
                + new_rssi_trend * self.config.smoothing_alpha;

            // Quality trend
            let quality_delta = quality - self.smoothed_quality;
            let new_quality_trend = quality_delta / dt_s;
            self.quality_trend = self.quality_trend * (1.0 - self.config.smoothing_alpha)
                + new_quality_trend * self.config.smoothing_alpha;
        }

        // Update smoothed values
        self.smoothed_rssi = self.smoothed_rssi * (1.0 - self.config.smoothing_alpha)
            + rssi * self.config.smoothing_alpha;
        self.smoothed_quality = self.smoothed_quality * (1.0 - self.config.smoothing_alpha)
            + quality * self.config.smoothing_alpha;

        // Add to history
        if self.history.len() >= self.config.history_size {
            self.history.remove(0);
        }
        self.history.push(measurement).map_err(|_| SwarmError::BufferFull)?;

        self.last_update_ms = timestamp;
        Ok(())
    }

    /// Update with basic RSSI and quality values
    pub fn update_basic(&mut self, rssi_dbm: i8, quality: f32, timestamp_ms: u64) -> Result<()> {
        let measurement = LinkMeasurement::new(timestamp_ms, rssi_dbm, quality);
        self.update(measurement)
    }

    /// Get current smoothed RSSI
    pub fn current_rssi(&self) -> f32 {
        self.smoothed_rssi
    }

    /// Get current smoothed quality
    pub fn current_quality(&self) -> f32 {
        self.smoothed_quality
    }

    /// Get current trend
    pub fn trend(&self) -> LinkTrend {
        if self.history.len() < 3 {
            return LinkTrend::Unknown;
        }
        LinkTrend::from_rate(self.quality_trend)
    }

    /// Predict quality at future time
    ///
    /// # Arguments
    /// * `time_ahead_ms` - Time ahead in milliseconds
    ///
    /// # Returns
    /// Predicted quality (0.0-1.0)
    pub fn predict_quality(&self, time_ahead_ms: u32) -> Result<f32> {
        if time_ahead_ms > self.config.max_prediction_horizon_ms {
            return Err(SwarmError::InvalidParameter);
        }

        let dt_s = time_ahead_ms as f32 / 1000.0;
        let predicted = self.smoothed_quality + self.quality_trend * dt_s;

        // Clamp to valid range
        Ok(predicted.clamp(0.0, 1.0))
    }

    /// Predict RSSI at future time
    pub fn predict_rssi(&self, time_ahead_ms: u32) -> Result<f32> {
        if time_ahead_ms > self.config.max_prediction_horizon_ms {
            return Err(SwarmError::InvalidParameter);
        }

        let dt_s = time_ahead_ms as f32 / 1000.0;
        let predicted = self.smoothed_rssi + self.rssi_trend * dt_s;

        // Clamp to reasonable range
        Ok(predicted.clamp(-120.0, 0.0))
    }

    /// Estimate time until link quality drops below threshold
    ///
    /// # Returns
    /// Time in milliseconds until quality < threshold, or None if not predicted
    pub fn time_to_threshold(&self, threshold: f32) -> Option<u32> {
        if self.smoothed_quality <= threshold {
            return Some(0); // Already below threshold
        }

        if self.quality_trend >= 0.0 {
            return None; // Quality stable or improving
        }

        // Time to reach threshold: t = (threshold - current) / trend
        let time_s = (threshold - self.smoothed_quality) / self.quality_trend;

        if time_s < 0.0 {
            None
        } else {
            Some((time_s * 1000.0) as u32)
        }
    }

    /// Estimate time until disconnect
    ///
    /// Uses minimum quality threshold from config
    pub fn time_to_disconnect(&self) -> Option<u32> {
        self.time_to_threshold(self.config.min_quality_threshold)
    }

    /// Estimate time until warning threshold
    pub fn time_to_warning(&self) -> Option<u32> {
        self.time_to_threshold(self.config.warning_quality_threshold)
    }

    /// Check if link is at risk of disconnection
    pub fn is_at_risk(&self) -> bool {
        // At risk if:
        // 1. Quality is degrading
        // 2. Quality is below warning threshold
        // 3. Disconnect predicted within 5 seconds
        let trend_risk = self.trend().is_at_risk();
        let quality_risk = self.smoothed_quality < self.config.warning_quality_threshold;
        let time_risk = self.time_to_disconnect()
            .map(|t| t < 5000)
            .unwrap_or(false);

        trend_risk || quality_risk || time_risk
    }

    /// Get link health status
    pub fn health_status(&self) -> LinkHealthStatus {
        if self.smoothed_quality < self.config.min_quality_threshold {
            LinkHealthStatus::Critical
        } else if self.is_at_risk() {
            LinkHealthStatus::Warning
        } else if self.smoothed_quality > 0.8 && self.trend() != LinkTrend::Degrading {
            LinkHealthStatus::Excellent
        } else {
            LinkHealthStatus::Good
        }
    }

    /// Get prediction confidence based on history
    pub fn prediction_confidence(&self) -> f32 {
        // Confidence based on:
        // 1. Amount of history
        // 2. Consistency of measurements
        // 3. Recency of last update
        let history_factor = (self.history.len() as f32 / self.config.history_size as f32).min(1.0);

        // Calculate variance in recent measurements
        let variance = self.calculate_quality_variance();
        let consistency_factor = (1.0 / (1.0 + variance * 10.0)).clamp(0.0, 1.0);

        history_factor * consistency_factor
    }

    /// Calculate variance in quality measurements
    fn calculate_quality_variance(&self) -> f32 {
        if self.history.len() < 2 {
            return 1.0; // High variance (unknown)
        }

        let mean = self.history.iter().map(|m| m.quality).sum::<f32>() / self.history.len() as f32;

        let variance = self.history.iter()
            .map(|m| (m.quality - mean) * (m.quality - mean))
            .sum::<f32>() / self.history.len() as f32;

        variance
    }

    /// Get history length
    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// Check if predictor is initialized
    pub fn is_initialized(&self) -> bool {
        self.history.len() >= 3
    }

    /// Reset predictor
    pub fn reset(&mut self) {
        self.history.clear();
        self.smoothed_rssi = -50.0;
        self.smoothed_quality = 1.0;
        self.rssi_trend = 0.0;
        self.quality_trend = 0.0;
        self.last_update_ms = 0;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK HEALTH STATUS
// ═══════════════════════════════════════════════════════════════════════════

/// Link health status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinkHealthStatus {
    /// Excellent link quality (>80%, stable or improving)
    Excellent,
    /// Good link quality (>40%, not at risk)
    Good,
    /// Warning: link quality degrading or below threshold
    Warning,
    /// Critical: link quality below minimum, disconnect imminent
    Critical,
}

impl LinkHealthStatus {
    /// Check if action is needed
    pub fn needs_action(&self) -> bool {
        matches!(self, LinkHealthStatus::Warning | LinkHealthStatus::Critical)
    }

    /// Get priority for routing decisions
    pub fn routing_priority(&self) -> u8 {
        match self {
            LinkHealthStatus::Excellent => 0,
            LinkHealthStatus::Good => 1,
            LinkHealthStatus::Warning => 2,
            LinkHealthStatus::Critical => 3,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK PREDICTION RESULT
// ═══════════════════════════════════════════════════════════════════════════

/// Complete link prediction result
#[derive(Debug, Clone)]
pub struct LinkPrediction {
    /// Neighbor ID
    pub neighbor_id: DroneId,
    /// Current quality
    pub current_quality: f32,
    /// Current RSSI
    pub current_rssi: f32,
    /// Predicted quality at horizon
    pub predicted_quality: f32,
    /// Predicted RSSI at horizon
    pub predicted_rssi: f32,
    /// Time to warning threshold (ms)
    pub time_to_warning_ms: Option<u32>,
    /// Time to disconnect (ms)
    pub time_to_disconnect_ms: Option<u32>,
    /// Current trend
    pub trend: LinkTrend,
    /// Health status
    pub health: LinkHealthStatus,
    /// Prediction confidence
    pub confidence: f32,
}

impl LinkPredictor {
    /// Get complete prediction at specified horizon
    pub fn get_prediction(&self, horizon_ms: u32) -> Result<LinkPrediction> {
        Ok(LinkPrediction {
            neighbor_id: self.neighbor_id,
            current_quality: self.smoothed_quality,
            current_rssi: self.smoothed_rssi,
            predicted_quality: self.predict_quality(horizon_ms)?,
            predicted_rssi: self.predict_rssi(horizon_ms)?,
            time_to_warning_ms: self.time_to_warning(),
            time_to_disconnect_ms: self.time_to_disconnect(),
            trend: self.trend(),
            health: self.health_status(),
            confidence: self.prediction_confidence(),
        })
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_link_trend_from_rate() {
        assert_eq!(LinkTrend::from_rate(0.05), LinkTrend::Improving);
        assert_eq!(LinkTrend::from_rate(0.0), LinkTrend::Stable);
        assert_eq!(LinkTrend::from_rate(-0.05), LinkTrend::Degrading);
        assert_eq!(LinkTrend::from_rate(-0.2), LinkTrend::Critical);
    }

    #[test]
    fn test_link_predictor_basic() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // Add stable measurements with longer interval for trend to stabilize
        for i in 0..20 {
            predictor.update_basic(-60, 0.9, i * 500).unwrap();
        }

        assert!(predictor.is_initialized());
        assert!((predictor.current_quality() - 0.9).abs() < 0.15,
            "quality {} should be close to 0.9", predictor.current_quality());
        // After many stable measurements, trend should stabilize
        let trend = predictor.trend();
        assert!(!trend.is_at_risk(), "trend {:?} should not be at risk for stable measurements", trend);
    }

    #[test]
    fn test_link_predictor_degrading() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // Add degrading measurements
        for i in 0..10 {
            let quality = 1.0 - (i as f32 * 0.08);
            predictor.update_basic(-60 - i as i8, quality, i as u64 * 1000).unwrap();
        }

        assert!(predictor.trend().is_at_risk());
        assert!(predictor.is_at_risk());
    }

    #[test]
    fn test_time_to_disconnect() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // Simulate degrading link
        predictor.update_basic(-60, 0.8, 0).unwrap();
        predictor.update_basic(-65, 0.6, 1000).unwrap();
        predictor.update_basic(-70, 0.4, 2000).unwrap();

        // Should predict disconnect
        let ttd = predictor.time_to_disconnect();
        assert!(ttd.is_some());
        assert!(ttd.unwrap() > 0);
    }

    #[test]
    fn test_quality_prediction() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // Add measurements with known trend
        predictor.update_basic(-60, 0.9, 0).unwrap();
        predictor.update_basic(-60, 0.8, 1000).unwrap();
        predictor.update_basic(-60, 0.7, 2000).unwrap();

        // Predict 1 second ahead
        let predicted = predictor.predict_quality(1000).unwrap();

        // Should be lower than current
        assert!(predicted < predictor.current_quality());
    }

    #[test]
    fn test_health_status() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // Good link - need more measurements to let smoothed values converge
        for i in 0..15 {
            predictor.update_basic(-50, 0.95, i * 200).unwrap();
        }
        // After convergence, either Excellent or Good is acceptable for high quality
        let status = predictor.health_status();
        assert!(
            status == LinkHealthStatus::Excellent || status == LinkHealthStatus::Good,
            "status {:?} should be Excellent or Good for high quality link", status
        );

        // Degraded link
        predictor.reset();
        for i in 0..10 {
            predictor.update_basic(-80, 0.3, i * 200).unwrap();
        }
        assert!(predictor.health_status().needs_action(),
            "health {:?} should need action for degraded link", predictor.health_status());
    }

    #[test]
    fn test_prediction_confidence() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        // No history = low confidence
        assert!(predictor.prediction_confidence() < 0.5);

        // Build history
        for i in 0..20 {
            predictor.update_basic(-60, 0.8, i * 100).unwrap();
        }

        // Full history = high confidence
        assert!(predictor.prediction_confidence() > 0.7);
    }

    #[test]
    fn test_get_prediction() {
        let mut predictor = LinkPredictor::new_default(DroneId::new(1));

        for i in 0..5 {
            predictor.update_basic(-60, 0.8, i * 100).unwrap();
        }

        let prediction = predictor.get_prediction(1000).unwrap();

        assert_eq!(prediction.neighbor_id, DroneId::new(1));
        assert!(prediction.current_quality > 0.0);
        assert!(prediction.predicted_quality <= 1.0);
    }

    #[test]
    fn test_measurement_with_extras() {
        let measurement = LinkMeasurement::new(1000, -60, 0.8)
            .with_position(Position { x: 10.0, y: 20.0, z: 30.0 })
            .with_rtt(50);

        assert!(measurement.position.is_some());
        assert!(measurement.rtt_ms.is_some());
        assert_eq!(measurement.rtt_ms.unwrap(), 50);
    }
}
