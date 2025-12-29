//! Resilience Scoring for Chaos Engineering
//!
//! Measures system resilience based on fault detection times,
//! recovery times, cascade depth, and availability metrics.
//!
//! Features:
//! - MTTR (Mean Time to Recovery) calculation
//! - Recovery success rate tracking
//! - Cascade depth measurement
//! - Availability metrics
//! - Comprehensive resilience scoring

use crate::fault_injector::{FaultId, FaultSeverity, FaultType};
use crate::types::DroneId;
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// FAULT EVENT TRACKING
// ═══════════════════════════════════════════════════════════════════════════

/// Record of a fault event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultRecord {
    /// Fault ID
    pub fault_id: FaultId,
    /// Fault type
    pub fault_type: FaultType,
    /// Severity
    pub severity: FaultSeverity,
    /// Target drone (None = global)
    pub target: Option<DroneId>,
    /// Time fault was injected (ms)
    pub injected_at_ms: u64,
    /// Time fault was detected (ms)
    pub detected_at_ms: Option<u64>,
    /// Time recovery started (ms)
    pub recovery_started_ms: Option<u64>,
    /// Time recovery completed (ms)
    pub recovered_at_ms: Option<u64>,
    /// Was recovery successful?
    pub recovery_successful: Option<bool>,
    /// Cascade depth (how many other failures it caused)
    pub cascade_depth: u8,
    /// Secondary faults caused by this fault
    pub caused_faults: Vec<FaultId, 8>,
}

impl FaultRecord {
    /// Create a new fault record
    pub fn new(
        fault_id: FaultId,
        fault_type: FaultType,
        severity: FaultSeverity,
        target: Option<DroneId>,
        injected_at_ms: u64,
    ) -> Self {
        Self {
            fault_id,
            fault_type,
            severity,
            target,
            injected_at_ms,
            detected_at_ms: None,
            recovery_started_ms: None,
            recovered_at_ms: None,
            recovery_successful: None,
            cascade_depth: 0,
            caused_faults: Vec::new(),
        }
    }

    /// Get time to detection (ms)
    pub fn detection_time(&self) -> Option<u64> {
        self.detected_at_ms.map(|d| d.saturating_sub(self.injected_at_ms))
    }

    /// Get time to recovery (ms)
    pub fn recovery_time(&self) -> Option<u64> {
        match (self.detected_at_ms, self.recovered_at_ms) {
            (Some(d), Some(r)) => Some(r.saturating_sub(d)),
            _ => None,
        }
    }

    /// Get total fault duration (ms)
    pub fn total_duration(&self) -> Option<u64> {
        self.recovered_at_ms.map(|r| r.saturating_sub(self.injected_at_ms))
    }

    /// Is fault still active?
    pub fn is_active(&self) -> bool {
        self.recovered_at_ms.is_none()
    }

    /// Was recovery successful?
    pub fn was_successful(&self) -> bool {
        self.recovery_successful.unwrap_or(false)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RESILIENCE METRICS
// ═══════════════════════════════════════════════════════════════════════════

/// Aggregated resilience metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResilienceMetrics {
    /// Mean Time to Detection (ms)
    pub mttd_ms: f32,
    /// Mean Time to Recovery (ms)
    pub mttr_ms: f32,
    /// Mean Time Between Failures (ms) - only if enough data
    pub mtbf_ms: Option<f32>,
    /// Recovery success rate (0.0-1.0)
    pub recovery_success_rate: f32,
    /// Average cascade depth
    pub avg_cascade_depth: f32,
    /// Maximum cascade depth observed
    pub max_cascade_depth: u8,
    /// Total faults processed
    pub total_faults: u32,
    /// Successful recoveries
    pub successful_recoveries: u32,
    /// Failed recoveries
    pub failed_recoveries: u32,
    /// Faults still active
    pub active_faults: u32,
    /// System availability (uptime / total time)
    pub availability: f32,
    /// Overall resilience score (0-100)
    pub resilience_score: f32,
}

impl ResilienceMetrics {
    /// Get grade based on resilience score
    pub fn grade(&self) -> &'static str {
        match self.resilience_score as u32 {
            90..=100 => "A+ (Excellent)",
            80..=89 => "A (Very Good)",
            70..=79 => "B (Good)",
            60..=69 => "C (Acceptable)",
            50..=59 => "D (Poor)",
            _ => "F (Critical)",
        }
    }

    /// Is the system considered resilient?
    pub fn is_resilient(&self) -> bool {
        self.resilience_score >= 70.0
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RESILIENCE SCORER CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for resilience scoring
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResilienceScorerConfig {
    /// Target MTTD (ms) - used for scoring
    pub target_mttd_ms: u32,
    /// Target MTTR (ms) - used for scoring
    pub target_mttr_ms: u32,
    /// Maximum acceptable cascade depth
    pub max_acceptable_cascade: u8,
    /// Weight for detection time in score
    pub detection_weight: f32,
    /// Weight for recovery time in score
    pub recovery_weight: f32,
    /// Weight for cascade depth in score
    pub cascade_weight: f32,
    /// Weight for availability in score
    pub availability_weight: f32,
}

impl Default for ResilienceScorerConfig {
    fn default() -> Self {
        Self {
            target_mttd_ms: 1000,     // 1 second target detection
            target_mttr_ms: 5000,     // 5 second target recovery
            max_acceptable_cascade: 3,
            detection_weight: 0.2,
            recovery_weight: 0.3,
            cascade_weight: 0.2,
            availability_weight: 0.3,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RESILIENCE SCORER
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum number of fault records to track
const MAX_FAULT_RECORDS: usize = 128;

/// Resilience scorer for chaos engineering
#[derive(Debug)]
pub struct ResilienceScorer {
    /// Configuration
    config: ResilienceScorerConfig,
    /// Fault records
    records: FnvIndexMap<FaultId, FaultRecord, MAX_FAULT_RECORDS>,
    /// Total uptime (ms)
    total_uptime_ms: u64,
    /// Total downtime (ms)
    total_downtime_ms: u64,
    /// Start time of measurement period
    measurement_start_ms: u64,
    /// Last update time
    last_update_ms: u64,
    /// Number of drones being monitored
    drone_count: u32,
    /// Per-drone downtime tracking
    drone_downtime: FnvIndexMap<DroneId, u64, 64>,
}

impl ResilienceScorer {
    /// Create a new resilience scorer
    pub fn new(config: ResilienceScorerConfig) -> Self {
        Self {
            config,
            records: FnvIndexMap::new(),
            total_uptime_ms: 0,
            total_downtime_ms: 0,
            measurement_start_ms: 0,
            last_update_ms: 0,
            drone_count: 0,
            drone_downtime: FnvIndexMap::new(),
        }
    }

    /// Create with default configuration
    pub fn new_default() -> Self {
        Self::new(ResilienceScorerConfig::default())
    }

    /// Start measurement period
    pub fn start_measurement(&mut self, timestamp_ms: u64, drone_count: u32) {
        self.measurement_start_ms = timestamp_ms;
        self.last_update_ms = timestamp_ms;
        self.drone_count = drone_count;
        self.records.clear();
        self.drone_downtime.clear();
        self.total_uptime_ms = 0;
        self.total_downtime_ms = 0;
    }

    /// Record a fault being injected
    pub fn record_fault_injected(
        &mut self,
        fault_id: FaultId,
        fault_type: FaultType,
        severity: FaultSeverity,
        target: Option<DroneId>,
        timestamp_ms: u64,
    ) {
        let record = FaultRecord::new(fault_id, fault_type, severity, target, timestamp_ms);
        self.records.insert(fault_id, record).ok();
    }

    /// Record fault detection
    pub fn record_fault_detected(&mut self, fault_id: FaultId, timestamp_ms: u64) {
        if let Some(record) = self.records.get_mut(&fault_id) {
            record.detected_at_ms = Some(timestamp_ms);
        }
    }

    /// Record recovery started
    pub fn record_recovery_started(&mut self, fault_id: FaultId, timestamp_ms: u64) {
        if let Some(record) = self.records.get_mut(&fault_id) {
            record.recovery_started_ms = Some(timestamp_ms);
        }
    }

    /// Record recovery completed
    pub fn record_recovery(
        &mut self,
        fault_id: FaultId,
        timestamp_ms: u64,
        successful: bool,
    ) {
        if let Some(record) = self.records.get_mut(&fault_id) {
            record.recovered_at_ms = Some(timestamp_ms);
            record.recovery_successful = Some(successful);

            // Update downtime tracking
            if let Some(target) = record.target {
                let downtime = record.total_duration().unwrap_or(0);
                if let Some(d) = self.drone_downtime.get_mut(&target) {
                    *d += downtime;
                } else {
                    self.drone_downtime.insert(target, downtime).ok();
                }
            }
        }
    }

    /// Record a cascade (fault causing another fault)
    pub fn record_cascade(&mut self, cause_fault_id: FaultId, effect_fault_id: FaultId) {
        // Get effect's cascade depth
        let effect_depth = self.records
            .get(&effect_fault_id)
            .map(|r| r.cascade_depth)
            .unwrap_or(0);

        // Update cause's cascade info
        if let Some(record) = self.records.get_mut(&cause_fault_id) {
            record.caused_faults.push(effect_fault_id).ok();
            record.cascade_depth = record.cascade_depth.max(effect_depth + 1);
        }
    }

    /// Update time tracking
    pub fn update(&mut self, timestamp_ms: u64) {
        let elapsed = timestamp_ms.saturating_sub(self.last_update_ms);
        self.last_update_ms = timestamp_ms;

        // Count active faults
        let active_faults = self.records.values()
            .filter(|r| r.is_active())
            .count() as u32;

        // Update uptime/downtime
        if active_faults == 0 {
            self.total_uptime_ms += elapsed;
        } else {
            // Partial downtime based on affected drones
            let affected_drones = self.records.values()
                .filter(|r| r.is_active() && r.target.is_some())
                .count() as u32;

            if self.drone_count > 0 {
                let downtime_ratio = affected_drones as f32 / self.drone_count as f32;
                self.total_downtime_ms += (elapsed as f32 * downtime_ratio) as u64;
                self.total_uptime_ms += (elapsed as f32 * (1.0 - downtime_ratio)) as u64;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // METRIC CALCULATIONS
    // ═══════════════════════════════════════════════════════════════════════

    /// Calculate Mean Time to Detection
    pub fn calculate_mttd(&self) -> f32 {
        let detection_times: Vec<u64, MAX_FAULT_RECORDS> = self.records
            .values()
            .filter_map(|r| r.detection_time())
            .collect();

        if detection_times.is_empty() {
            return 0.0;
        }

        let sum: u64 = detection_times.iter().sum();
        sum as f32 / detection_times.len() as f32
    }

    /// Calculate Mean Time to Recovery
    pub fn calculate_mttr(&self) -> f32 {
        let recovery_times: Vec<u64, MAX_FAULT_RECORDS> = self.records
            .values()
            .filter_map(|r| r.recovery_time())
            .collect();

        if recovery_times.is_empty() {
            return 0.0;
        }

        let sum: u64 = recovery_times.iter().sum();
        sum as f32 / recovery_times.len() as f32
    }

    /// Calculate recovery success rate
    pub fn calculate_recovery_rate(&self) -> f32 {
        let total_recovered = self.records.values()
            .filter(|r| r.recovered_at_ms.is_some())
            .count();

        if total_recovered == 0 {
            return 1.0; // No faults to recover from = 100% success
        }

        let successful = self.records.values()
            .filter(|r| r.was_successful())
            .count();

        successful as f32 / total_recovered as f32
    }

    /// Calculate average cascade depth
    pub fn calculate_avg_cascade_depth(&self) -> f32 {
        if self.records.is_empty() {
            return 0.0;
        }

        let sum: u32 = self.records.values()
            .map(|r| r.cascade_depth as u32)
            .sum();

        sum as f32 / self.records.len() as f32
    }

    /// Get maximum cascade depth
    pub fn get_max_cascade_depth(&self) -> u8 {
        self.records.values()
            .map(|r| r.cascade_depth)
            .max()
            .unwrap_or(0)
    }

    /// Calculate system availability
    pub fn calculate_availability(&self) -> f32 {
        let total = self.total_uptime_ms + self.total_downtime_ms;
        if total == 0 {
            return 1.0;
        }
        self.total_uptime_ms as f32 / total as f32
    }

    /// Calculate overall resilience score (0-100)
    pub fn calculate_score(&self) -> f32 {
        let mttd = self.calculate_mttd();
        let mttr = self.calculate_mttr();
        let recovery_rate = self.calculate_recovery_rate();
        let cascade_depth = self.calculate_avg_cascade_depth();
        let availability = self.calculate_availability();

        // Detection score (100 if instant, 0 if 2x target)
        let detection_score = if mttd == 0.0 {
            100.0
        } else {
            let ratio = mttd / self.config.target_mttd_ms as f32;
            (100.0 * (1.0 - ratio.min(2.0) / 2.0)).max(0.0)
        };

        // Recovery score (100 if instant, 0 if 2x target)
        let recovery_score = if mttr == 0.0 {
            100.0
        } else {
            let ratio = mttr / self.config.target_mttr_ms as f32;
            (100.0 * (1.0 - ratio.min(2.0) / 2.0)).max(0.0)
        };

        // Cascade score (100 if no cascades, 0 if >= max acceptable)
        let cascade_score = {
            let ratio = cascade_depth / self.config.max_acceptable_cascade as f32;
            (100.0 * (1.0 - ratio.min(1.0))).max(0.0)
        };

        // Availability score (directly mapped to percentage)
        let availability_score = availability * 100.0;

        // Weighted combination
        let score = detection_score * self.config.detection_weight
            + recovery_score * self.config.recovery_weight
            + cascade_score * self.config.cascade_weight
            + availability_score * self.config.availability_weight;

        // Bonus for high recovery rate
        let recovery_bonus = recovery_rate * 10.0;

        (score + recovery_bonus).min(100.0)
    }

    /// Get comprehensive metrics
    pub fn get_metrics(&self) -> ResilienceMetrics {
        let total_faults = self.records.len() as u32;
        let successful_recoveries = self.records.values()
            .filter(|r| r.was_successful())
            .count() as u32;
        let failed_recoveries = self.records.values()
            .filter(|r| r.recovered_at_ms.is_some() && !r.was_successful())
            .count() as u32;
        let active_faults = self.records.values()
            .filter(|r| r.is_active())
            .count() as u32;

        ResilienceMetrics {
            mttd_ms: self.calculate_mttd(),
            mttr_ms: self.calculate_mttr(),
            mtbf_ms: None, // Would need longer-term tracking
            recovery_success_rate: self.calculate_recovery_rate(),
            avg_cascade_depth: self.calculate_avg_cascade_depth(),
            max_cascade_depth: self.get_max_cascade_depth(),
            total_faults,
            successful_recoveries,
            failed_recoveries,
            active_faults,
            availability: self.calculate_availability(),
            resilience_score: self.calculate_score(),
        }
    }

    /// Get detailed report
    pub fn get_report(&self) -> ResilienceReport {
        let metrics = self.get_metrics();

        // Find worst performing fault types
        let mut fault_type_stats: FnvIndexMap<FaultType, (u32, u64), 16> = FnvIndexMap::new();
        for record in self.records.values() {
            if let Some(recovery_time) = record.recovery_time() {
                if let Some(stats) = fault_type_stats.get_mut(&record.fault_type) {
                    stats.0 += 1;
                    stats.1 += recovery_time;
                } else {
                    fault_type_stats.insert(record.fault_type, (1, recovery_time)).ok();
                }
            }
        }

        let mut fault_breakdown: Vec<FaultTypeBreakdown, 10> = Vec::new();
        for (fault_type, (count, total_recovery)) in fault_type_stats.iter() {
            let avg_recovery = if *count > 0 {
                *total_recovery as f32 / *count as f32
            } else {
                0.0
            };

            fault_breakdown.push(FaultTypeBreakdown {
                fault_type: *fault_type,
                count: *count,
                avg_recovery_time_ms: avg_recovery,
            }).ok();
        }

        ResilienceReport {
            metrics,
            fault_breakdown,
            total_measurement_time_ms: self.total_uptime_ms + self.total_downtime_ms,
            drone_count: self.drone_count,
        }
    }

    /// Get fault record by ID
    pub fn get_record(&self, fault_id: FaultId) -> Option<&FaultRecord> {
        self.records.get(&fault_id)
    }

    /// Get all records
    pub fn all_records(&self) -> impl Iterator<Item = &FaultRecord> {
        self.records.values()
    }

    /// Reset all metrics
    pub fn reset(&mut self) {
        self.records.clear();
        self.drone_downtime.clear();
        self.total_uptime_ms = 0;
        self.total_downtime_ms = 0;
        self.measurement_start_ms = 0;
        self.last_update_ms = 0;
    }
}

impl Default for ResilienceScorer {
    fn default() -> Self {
        Self::new_default()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RESILIENCE REPORT
// ═══════════════════════════════════════════════════════════════════════════

/// Per-fault-type breakdown
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultTypeBreakdown {
    /// Fault type
    pub fault_type: FaultType,
    /// Number of occurrences
    pub count: u32,
    /// Average recovery time (ms)
    pub avg_recovery_time_ms: f32,
}

/// Comprehensive resilience report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResilienceReport {
    /// Overall metrics
    pub metrics: ResilienceMetrics,
    /// Per-fault-type breakdown
    pub fault_breakdown: Vec<FaultTypeBreakdown, 10>,
    /// Total measurement time
    pub total_measurement_time_ms: u64,
    /// Number of drones monitored
    pub drone_count: u32,
}

impl ResilienceReport {
    /// Get summary string
    pub fn summary(&self) -> &'static str {
        if self.metrics.resilience_score >= 90.0 {
            "System demonstrates excellent resilience"
        } else if self.metrics.resilience_score >= 70.0 {
            "System shows good resilience with minor areas for improvement"
        } else if self.metrics.resilience_score >= 50.0 {
            "System has moderate resilience, improvements recommended"
        } else {
            "System resilience is below acceptable levels, immediate attention required"
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scorer_creation() {
        let scorer = ResilienceScorer::new_default();
        let metrics = scorer.get_metrics();

        assert_eq!(metrics.total_faults, 0);
        assert!((metrics.resilience_score - 100.0).abs() < 0.1); // Perfect score with no faults
    }

    #[test]
    fn test_fault_recording() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        let fault_id = FaultId::new(1);
        scorer.record_fault_injected(
            fault_id,
            FaultType::CommunicationFailure,
            FaultSeverity::Major,
            Some(DroneId::new(1)),
            0,
        );

        assert_eq!(scorer.get_metrics().total_faults, 1);
        assert!(scorer.get_record(fault_id).is_some());
    }

    #[test]
    fn test_detection_time() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        let fault_id = FaultId::new(1);
        scorer.record_fault_injected(
            fault_id,
            FaultType::SensorMalfunction,
            FaultSeverity::Minor,
            None,
            0,
        );

        scorer.record_fault_detected(fault_id, 500);

        let record = scorer.get_record(fault_id).unwrap();
        assert_eq!(record.detection_time(), Some(500));
    }

    #[test]
    fn test_recovery_time() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        let fault_id = FaultId::new(1);
        scorer.record_fault_injected(
            fault_id,
            FaultType::CommunicationFailure,
            FaultSeverity::Major,
            Some(DroneId::new(1)),
            0,
        );

        scorer.record_fault_detected(fault_id, 100);
        scorer.record_recovery_started(fault_id, 200);
        scorer.record_recovery(fault_id, 1000, true);

        let record = scorer.get_record(fault_id).unwrap();
        assert_eq!(record.recovery_time(), Some(900)); // 1000 - 100
        assert!(record.was_successful());
    }

    #[test]
    fn test_mttr_calculation() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        // Record multiple faults with different recovery times
        for i in 1..=3 {
            let fault_id = FaultId::new(i);
            scorer.record_fault_injected(
                fault_id,
                FaultType::SensorMalfunction,
                FaultSeverity::Minor,
                None,
                0,
            );
            scorer.record_fault_detected(fault_id, 100);
            scorer.record_recovery(fault_id, 100 + (i as u64 * 1000), true);
        }

        let mttr = scorer.calculate_mttr();
        // Average of 1000, 2000, 3000 = 2000
        assert!((mttr - 2000.0).abs() < 1.0);
    }

    #[test]
    fn test_recovery_rate() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        // 2 successful, 1 failed
        for i in 1..=3 {
            let fault_id = FaultId::new(i);
            scorer.record_fault_injected(
                fault_id,
                FaultType::MotorFailure,
                FaultSeverity::Major,
                None,
                0,
            );
            scorer.record_fault_detected(fault_id, 100);
            scorer.record_recovery(fault_id, 1000, i <= 2); // First 2 successful
        }

        let rate = scorer.calculate_recovery_rate();
        assert!((rate - 0.666).abs() < 0.01);
    }

    #[test]
    fn test_cascade_tracking() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        let fault1 = FaultId::new(1);
        let fault2 = FaultId::new(2);

        scorer.record_fault_injected(fault1, FaultType::NetworkPartition, FaultSeverity::Critical, None, 0);
        scorer.record_fault_injected(fault2, FaultType::CommunicationFailure, FaultSeverity::Major, None, 100);

        scorer.record_cascade(fault1, fault2);

        let record = scorer.get_record(fault1).unwrap();
        assert_eq!(record.cascade_depth, 1);
        assert!(record.caused_faults.contains(&fault2));
    }

    #[test]
    fn test_availability() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        // 10 seconds of uptime
        scorer.update(10000);

        let availability = scorer.calculate_availability();
        assert!((availability - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_resilience_score() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        // Perfect scenario - instant detection and recovery
        let fault_id = FaultId::new(1);
        scorer.record_fault_injected(fault_id, FaultType::SensorMalfunction, FaultSeverity::Minor, None, 0);
        scorer.record_fault_detected(fault_id, 0);
        scorer.record_recovery(fault_id, 0, true);
        scorer.update(10000);

        let score = scorer.calculate_score();
        // Should be near 100 (instant detection/recovery, no cascades, 100% availability)
        assert!(score > 90.0);
    }

    #[test]
    fn test_metrics_grade() {
        let metrics = ResilienceMetrics {
            resilience_score: 95.0,
            ..Default::default()
        };
        assert!(metrics.grade().contains("Excellent"));

        let metrics2 = ResilienceMetrics {
            resilience_score: 45.0,
            ..Default::default()
        };
        assert!(metrics2.grade().contains("Critical"));
    }

    #[test]
    fn test_report_generation() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        for i in 1..=5 {
            let fault_id = FaultId::new(i);
            scorer.record_fault_injected(
                fault_id,
                FaultType::CommunicationFailure,
                FaultSeverity::Major,
                Some(DroneId::new(i as u64)),
                0,
            );
            scorer.record_fault_detected(fault_id, 500);
            scorer.record_recovery(fault_id, 2000, true);
        }

        let report = scorer.get_report();
        assert_eq!(report.metrics.total_faults, 5);
        assert_eq!(report.metrics.successful_recoveries, 5);
        assert!(!report.fault_breakdown.is_empty());
    }

    #[test]
    fn test_reset() {
        let mut scorer = ResilienceScorer::new_default();
        scorer.start_measurement(0, 10);

        scorer.record_fault_injected(
            FaultId::new(1),
            FaultType::Byzantine,
            FaultSeverity::Critical,
            None,
            0,
        );

        scorer.reset();
        assert_eq!(scorer.get_metrics().total_faults, 0);
    }
}
