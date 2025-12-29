//! Fault Injection Engine for Chaos Engineering
//!
//! Provides controlled fault injection capabilities for testing
//! system resilience. This module is feature-gated and should
//! NEVER be included in production builds.
//!
//! Features:
//! - Subsystem fault injection
//! - Network partition simulation
//! - Byzantine behavior simulation
//! - Latency injection
//! - Deterministic mode for reproducible tests

use crate::types::{DroneId, Result, SwarmError};
use heapless::{FnvIndexMap, FnvIndexSet, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// FAULT TYPES
// ═══════════════════════════════════════════════════════════════════════════

/// Type of fault to inject
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FaultType {
    /// Communication failure - drone cannot send/receive messages
    CommunicationFailure,
    /// Sensor malfunction - drone reports incorrect sensor data
    SensorMalfunction,
    /// Motor failure - drone cannot move properly
    MotorFailure,
    /// Battery drain - rapid battery depletion
    BatteryDrain,
    /// GPS spoofing - incorrect position reports
    GpsSpoofing,
    /// Memory corruption - random state corruption
    MemoryCorruption,
    /// CPU overload - processing delays
    CpuOverload,
    /// Network partition - isolated from group
    NetworkPartition,
    /// Byzantine behavior - malicious/erratic actions
    Byzantine,
    /// Clock drift - time synchronization issues
    ClockDrift,
}

impl FaultType {
    /// Get human-readable name
    pub fn name(&self) -> &'static str {
        match self {
            FaultType::CommunicationFailure => "Communication Failure",
            FaultType::SensorMalfunction => "Sensor Malfunction",
            FaultType::MotorFailure => "Motor Failure",
            FaultType::BatteryDrain => "Battery Drain",
            FaultType::GpsSpoofing => "GPS Spoofing",
            FaultType::MemoryCorruption => "Memory Corruption",
            FaultType::CpuOverload => "CPU Overload",
            FaultType::NetworkPartition => "Network Partition",
            FaultType::Byzantine => "Byzantine Behavior",
            FaultType::ClockDrift => "Clock Drift",
        }
    }

    /// Get severity weight (1-10)
    pub fn base_severity(&self) -> u8 {
        match self {
            FaultType::ClockDrift => 2,
            FaultType::CpuOverload => 3,
            FaultType::BatteryDrain => 4,
            FaultType::SensorMalfunction => 5,
            FaultType::GpsSpoofing => 6,
            FaultType::MotorFailure => 7,
            FaultType::CommunicationFailure => 7,
            FaultType::MemoryCorruption => 8,
            FaultType::NetworkPartition => 8,
            FaultType::Byzantine => 10,
        }
    }
}

/// Fault severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum FaultSeverity {
    /// Minor fault - slight degradation
    Minor,
    /// Moderate fault - noticeable impact
    Moderate,
    /// Major fault - significant degradation
    Major,
    /// Critical fault - complete failure
    Critical,
}

impl FaultSeverity {
    /// Get severity multiplier
    pub fn multiplier(&self) -> f32 {
        match self {
            FaultSeverity::Minor => 0.25,
            FaultSeverity::Moderate => 0.5,
            FaultSeverity::Major => 0.75,
            FaultSeverity::Critical => 1.0,
        }
    }

    /// Get duration multiplier (for fault duration)
    pub fn duration_factor(&self) -> f32 {
        match self {
            FaultSeverity::Minor => 0.5,
            FaultSeverity::Moderate => 1.0,
            FaultSeverity::Major => 2.0,
            FaultSeverity::Critical => 5.0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// INJECTED FAULT
// ═══════════════════════════════════════════════════════════════════════════

/// Unique identifier for an injected fault
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FaultId(pub u64);

impl FaultId {
    pub fn new(id: u64) -> Self {
        Self(id)
    }
}

/// An active injected fault
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InjectedFault {
    /// Unique fault ID
    pub id: FaultId,
    /// Type of fault
    pub fault_type: FaultType,
    /// Severity level
    pub severity: FaultSeverity,
    /// Target drone (None = global)
    pub target: Option<DroneId>,
    /// Injection timestamp (ms)
    pub injected_at_ms: u64,
    /// Duration (ms), None = permanent until cleared
    pub duration_ms: Option<u32>,
    /// Additional parameters
    pub params: FaultParams,
}

impl InjectedFault {
    /// Check if fault has expired
    pub fn is_expired(&self, current_time_ms: u64) -> bool {
        if let Some(duration) = self.duration_ms {
            current_time_ms >= self.injected_at_ms + duration as u64
        } else {
            false
        }
    }

    /// Calculate effective severity (type base * severity multiplier)
    pub fn effective_severity(&self) -> f32 {
        self.fault_type.base_severity() as f32 * self.severity.multiplier()
    }
}

/// Additional fault parameters
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FaultParams {
    /// Latency to add (ms)
    pub latency_ms: Option<u32>,
    /// Packet drop probability (0.0-1.0)
    pub drop_probability: Option<f32>,
    /// Position offset for GPS spoofing
    pub position_offset: Option<(f32, f32, f32)>,
    /// Clock drift rate (ms per second)
    pub clock_drift_rate: Option<i32>,
    /// Battery drain rate multiplier
    pub drain_multiplier: Option<f32>,
    /// Partition group ID (for network partitions)
    pub partition_group: Option<u8>,
}

// ═══════════════════════════════════════════════════════════════════════════
// FAULT INJECTOR CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for the fault injector
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultInjectorConfig {
    /// Maximum concurrent faults
    pub max_concurrent_faults: usize,
    /// Default fault duration (ms)
    pub default_duration_ms: u32,
    /// Enable deterministic mode
    pub deterministic: bool,
    /// Random seed for deterministic mode
    pub seed: u64,
    /// Maximum fault severity allowed
    pub max_severity: FaultSeverity,
    /// Fault types that are disabled
    pub disabled_fault_types: Vec<FaultType, 10>,
}

impl Default for FaultInjectorConfig {
    fn default() -> Self {
        Self {
            max_concurrent_faults: 10,
            default_duration_ms: 5000,
            deterministic: false,
            seed: 0,
            max_severity: FaultSeverity::Critical,
            disabled_fault_types: Vec::new(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FAULT INJECTOR
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum number of active faults
const MAX_ACTIVE_FAULTS: usize = 32;
/// Maximum number of partitioned drones
const MAX_PARTITIONED_DRONES: usize = 64;
/// Maximum number of partition groups
const MAX_PARTITION_GROUPS: usize = 8;

/// Fault injection engine
///
/// Controls fault injection for chaos engineering tests.
/// Feature-gated to prevent accidental use in production.
#[derive(Debug)]
pub struct FaultInjector {
    /// Configuration
    config: FaultInjectorConfig,
    /// Active faults
    active_faults: FnvIndexMap<FaultId, InjectedFault, MAX_ACTIVE_FAULTS>,
    /// Faults by target drone
    faults_by_target: FnvIndexMap<DroneId, Vec<FaultId, 8>, 64>,
    /// Global faults (no specific target)
    global_faults: Vec<FaultId, 16>,
    /// Network partitions: drone -> partition group
    partition_groups: FnvIndexMap<DroneId, u8, MAX_PARTITIONED_DRONES>,
    /// Byzantine drones
    byzantine_drones: FnvIndexSet<DroneId, 16>,
    /// Next fault ID
    next_fault_id: u64,
    /// Injection counter
    injection_count: u64,
    /// RNG state for deterministic mode
    rng_state: u64,
}

impl FaultInjector {
    /// Create a new fault injector
    pub fn new(config: FaultInjectorConfig) -> Self {
        let rng_state = if config.deterministic { config.seed } else { 0 };
        Self {
            config,
            active_faults: FnvIndexMap::new(),
            faults_by_target: FnvIndexMap::new(),
            global_faults: Vec::new(),
            partition_groups: FnvIndexMap::new(),
            byzantine_drones: FnvIndexSet::new(),
            next_fault_id: 1,
            injection_count: 0,
            rng_state,
        }
    }

    /// Create with default configuration
    pub fn new_default() -> Self {
        Self::new(FaultInjectorConfig::default())
    }

    /// Inject a fault
    ///
    /// # Arguments
    /// * `fault_type` - Type of fault to inject
    /// * `severity` - Severity level
    /// * `target` - Target drone (None for global)
    /// * `timestamp_ms` - Current timestamp
    ///
    /// # Returns
    /// Fault ID if successful
    pub fn inject_fault(
        &mut self,
        fault_type: FaultType,
        severity: FaultSeverity,
        target: Option<DroneId>,
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        self.inject_fault_with_params(
            fault_type,
            severity,
            target,
            timestamp_ms,
            None,
            FaultParams::default(),
        )
    }

    /// Inject a fault with custom parameters
    pub fn inject_fault_with_params(
        &mut self,
        fault_type: FaultType,
        severity: FaultSeverity,
        target: Option<DroneId>,
        timestamp_ms: u64,
        duration_ms: Option<u32>,
        params: FaultParams,
    ) -> Result<FaultId> {
        // Check if fault type is disabled
        if self.config.disabled_fault_types.contains(&fault_type) {
            return Err(SwarmError::PermissionDenied);
        }

        // Check severity limit
        if severity > self.config.max_severity {
            return Err(SwarmError::InvalidParameter);
        }

        // Check concurrent fault limit
        if self.active_faults.len() >= self.config.max_concurrent_faults {
            return Err(SwarmError::BufferFull);
        }

        let fault_id = FaultId::new(self.next_fault_id);
        self.next_fault_id += 1;

        let fault = InjectedFault {
            id: fault_id,
            fault_type,
            severity,
            target,
            injected_at_ms: timestamp_ms,
            duration_ms: duration_ms.or(Some(self.config.default_duration_ms)),
            params,
        };

        // Track fault
        self.active_faults.insert(fault_id, fault)
            .map_err(|_| SwarmError::BufferFull)?;

        // Index by target
        if let Some(drone_id) = target {
            if let Some(faults) = self.faults_by_target.get_mut(&drone_id) {
                faults.push(fault_id).ok();
            } else {
                let mut faults = Vec::new();
                faults.push(fault_id).ok();
                self.faults_by_target.insert(drone_id, faults).ok();
            }
        } else {
            self.global_faults.push(fault_id).ok();
        }

        self.injection_count += 1;
        Ok(fault_id)
    }

    /// Inject network partition
    ///
    /// Drones in different groups cannot communicate
    pub fn inject_network_partition(
        &mut self,
        groups: &[(DroneId, u8)],
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        // Add drones to partition groups
        for &(drone_id, group) in groups {
            if group >= MAX_PARTITION_GROUPS as u8 {
                return Err(SwarmError::InvalidParameter);
            }
            self.partition_groups.insert(drone_id, group)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        // Create a global partition fault
        self.inject_fault(
            FaultType::NetworkPartition,
            FaultSeverity::Critical,
            None,
            timestamp_ms,
        )
    }

    /// Inject Byzantine behavior for a drone
    ///
    /// Byzantine drones may:
    /// - Send conflicting messages
    /// - Ignore protocol rules
    /// - Report false data
    pub fn inject_byzantine_behavior(
        &mut self,
        drone_id: DroneId,
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        self.byzantine_drones.insert(drone_id)
            .map_err(|_| SwarmError::BufferFull)?;

        self.inject_fault(
            FaultType::Byzantine,
            FaultSeverity::Critical,
            Some(drone_id),
            timestamp_ms,
        )
    }

    /// Inject latency for a target
    pub fn inject_latency(
        &mut self,
        target: DroneId,
        delay_ms: u32,
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        let params = FaultParams {
            latency_ms: Some(delay_ms),
            ..Default::default()
        };

        self.inject_fault_with_params(
            FaultType::CpuOverload,
            FaultSeverity::Moderate,
            Some(target),
            timestamp_ms,
            None,
            params,
        )
    }

    /// Inject GPS spoofing
    pub fn inject_gps_spoofing(
        &mut self,
        target: DroneId,
        offset: (f32, f32, f32),
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        let params = FaultParams {
            position_offset: Some(offset),
            ..Default::default()
        };

        self.inject_fault_with_params(
            FaultType::GpsSpoofing,
            FaultSeverity::Major,
            Some(target),
            timestamp_ms,
            None,
            params,
        )
    }

    /// Inject packet drops
    pub fn inject_packet_drops(
        &mut self,
        target: DroneId,
        drop_probability: f32,
        timestamp_ms: u64,
    ) -> Result<FaultId> {
        let params = FaultParams {
            drop_probability: Some(drop_probability.clamp(0.0, 1.0)),
            ..Default::default()
        };

        self.inject_fault_with_params(
            FaultType::CommunicationFailure,
            FaultSeverity::Moderate,
            Some(target),
            timestamp_ms,
            None,
            params,
        )
    }

    /// Clear a specific fault
    pub fn clear_fault(&mut self, fault_id: FaultId) -> Result<()> {
        if let Some(fault) = self.active_faults.remove(&fault_id) {
            // Remove from target index
            if let Some(drone_id) = fault.target {
                if let Some(faults) = self.faults_by_target.get_mut(&drone_id) {
                    if let Some(pos) = faults.iter().position(|&id| id == fault_id) {
                        faults.remove(pos);
                    }
                }

                // Clear Byzantine status if this was a Byzantine fault
                if fault.fault_type == FaultType::Byzantine {
                    self.byzantine_drones.remove(&drone_id);
                }
            } else {
                // Remove from global faults
                if let Some(pos) = self.global_faults.iter().position(|&id| id == fault_id) {
                    self.global_faults.remove(pos);
                }

                // Clear partitions if this was a partition fault
                if fault.fault_type == FaultType::NetworkPartition {
                    self.partition_groups.clear();
                }
            }
            Ok(())
        } else {
            Err(SwarmError::InvalidParameter)
        }
    }

    /// Clear all faults for a target
    pub fn clear_faults_for_target(&mut self, target: DroneId) {
        let fault_ids: Vec<FaultId, 8> = self.faults_by_target
            .get(&target)
            .map(|f| f.clone())
            .unwrap_or_default();

        for fault_id in fault_ids {
            self.clear_fault(fault_id).ok();
        }

        self.partition_groups.remove(&target);
        self.byzantine_drones.remove(&target);
    }

    /// Clear all injections
    pub fn clear_all_injections(&mut self) {
        self.active_faults.clear();
        self.faults_by_target.clear();
        self.global_faults.clear();
        self.partition_groups.clear();
        self.byzantine_drones.clear();
    }

    /// Expire old faults
    pub fn expire_faults(&mut self, current_time_ms: u64) {
        let expired: Vec<FaultId, MAX_ACTIVE_FAULTS> = self.active_faults
            .iter()
            .filter(|(_, f)| f.is_expired(current_time_ms))
            .map(|(id, _)| *id)
            .collect();

        for fault_id in expired {
            self.clear_fault(fault_id).ok();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // QUERY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /// Check if a drone is affected by a fault type
    pub fn is_affected(&self, drone_id: DroneId, fault_type: FaultType) -> bool {
        // Check target-specific faults
        if let Some(faults) = self.faults_by_target.get(&drone_id) {
            for fault_id in faults {
                if let Some(fault) = self.active_faults.get(fault_id) {
                    if fault.fault_type == fault_type {
                        return true;
                    }
                }
            }
        }

        // Check global faults
        for fault_id in &self.global_faults {
            if let Some(fault) = self.active_faults.get(fault_id) {
                if fault.fault_type == fault_type {
                    return true;
                }
            }
        }

        false
    }

    /// Get all faults affecting a drone
    pub fn get_faults_for_drone(&self, drone_id: DroneId) -> Vec<&InjectedFault, 16> {
        let mut result = Vec::new();

        // Target-specific faults
        if let Some(fault_ids) = self.faults_by_target.get(&drone_id) {
            for fault_id in fault_ids {
                if let Some(fault) = self.active_faults.get(fault_id) {
                    result.push(fault).ok();
                }
            }
        }

        // Global faults
        for fault_id in &self.global_faults {
            if let Some(fault) = self.active_faults.get(fault_id) {
                result.push(fault).ok();
            }
        }

        result
    }

    /// Check if drone is Byzantine
    pub fn is_byzantine(&self, drone_id: DroneId) -> bool {
        self.byzantine_drones.contains(&drone_id)
    }

    /// Check if two drones can communicate (not partitioned)
    pub fn can_communicate(&self, drone_a: DroneId, drone_b: DroneId) -> bool {
        match (self.partition_groups.get(&drone_a), self.partition_groups.get(&drone_b)) {
            (Some(group_a), Some(group_b)) => group_a == group_b,
            (None, None) => true,
            _ => false, // One partitioned, one not
        }
    }

    /// Get latency for a drone (if any)
    pub fn get_latency(&self, drone_id: DroneId) -> Option<u32> {
        for fault in self.get_faults_for_drone(drone_id) {
            if let Some(latency) = fault.params.latency_ms {
                return Some(latency);
            }
        }
        None
    }

    /// Get position offset for GPS spoofing
    pub fn get_position_offset(&self, drone_id: DroneId) -> Option<(f32, f32, f32)> {
        for fault in self.get_faults_for_drone(drone_id) {
            if fault.fault_type == FaultType::GpsSpoofing {
                return fault.params.position_offset;
            }
        }
        None
    }

    /// Get packet drop probability
    pub fn get_drop_probability(&self, drone_id: DroneId) -> f32 {
        for fault in self.get_faults_for_drone(drone_id) {
            if let Some(prob) = fault.params.drop_probability {
                return prob;
            }
        }
        0.0
    }

    /// Get number of active faults
    pub fn active_fault_count(&self) -> usize {
        self.active_faults.len()
    }

    /// Get total injection count
    pub fn injection_count(&self) -> u64 {
        self.injection_count
    }

    /// Get active fault by ID
    pub fn get_fault(&self, fault_id: FaultId) -> Option<&InjectedFault> {
        self.active_faults.get(&fault_id)
    }

    /// Get all active faults
    pub fn all_active_faults(&self) -> impl Iterator<Item = &InjectedFault> {
        self.active_faults.values()
    }

    // ═══════════════════════════════════════════════════════════════════════
    // DETERMINISTIC RNG (for reproducible chaos)
    // ═══════════════════════════════════════════════════════════════════════

    /// Get next random u32 (deterministic if configured)
    pub fn next_random(&mut self) -> u32 {
        if self.config.deterministic {
            // Simple xorshift64
            self.rng_state ^= self.rng_state << 13;
            self.rng_state ^= self.rng_state >> 7;
            self.rng_state ^= self.rng_state << 17;
            self.rng_state as u32
        } else {
            // Use injection count as pseudo-random (not cryptographically secure)
            let state = self.injection_count.wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            state as u32
        }
    }

    /// Get random f32 in [0, 1)
    pub fn next_random_f32(&mut self) -> f32 {
        (self.next_random() as f32) / (u32::MAX as f32)
    }

    /// Should drop packet based on probability?
    pub fn should_drop_packet(&mut self, drone_id: DroneId) -> bool {
        let prob = self.get_drop_probability(drone_id);
        if prob > 0.0 {
            self.next_random_f32() < prob
        } else {
            false
        }
    }
}

impl Default for FaultInjector {
    fn default() -> Self {
        Self::new_default()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fault_injection() {
        let mut injector = FaultInjector::new_default();
        let drone = DroneId::new(1);

        let fault_id = injector.inject_fault(
            FaultType::CommunicationFailure,
            FaultSeverity::Major,
            Some(drone),
            1000,
        ).unwrap();

        assert!(injector.is_affected(drone, FaultType::CommunicationFailure));
        assert_eq!(injector.active_fault_count(), 1);

        injector.clear_fault(fault_id).unwrap();
        assert!(!injector.is_affected(drone, FaultType::CommunicationFailure));
    }

    #[test]
    fn test_network_partition() {
        let mut injector = FaultInjector::new_default();
        let drone_a = DroneId::new(1);
        let drone_b = DroneId::new(2);
        let drone_c = DroneId::new(3);

        // Put A and B in group 0, C in group 1
        injector.inject_network_partition(
            &[(drone_a, 0), (drone_b, 0), (drone_c, 1)],
            1000,
        ).unwrap();

        assert!(injector.can_communicate(drone_a, drone_b));
        assert!(!injector.can_communicate(drone_a, drone_c));
        assert!(!injector.can_communicate(drone_b, drone_c));
    }

    #[test]
    fn test_byzantine_behavior() {
        let mut injector = FaultInjector::new_default();
        let drone = DroneId::new(1);

        injector.inject_byzantine_behavior(drone, 1000).unwrap();

        assert!(injector.is_byzantine(drone));
        assert!(injector.is_affected(drone, FaultType::Byzantine));
    }

    #[test]
    fn test_latency_injection() {
        let mut injector = FaultInjector::new_default();
        let drone = DroneId::new(1);

        injector.inject_latency(drone, 100, 1000).unwrap();

        assert_eq!(injector.get_latency(drone), Some(100));
    }

    #[test]
    fn test_fault_expiration() {
        let config = FaultInjectorConfig {
            default_duration_ms: 1000,
            ..Default::default()
        };
        let mut injector = FaultInjector::new(config);
        let drone = DroneId::new(1);

        injector.inject_fault(
            FaultType::SensorMalfunction,
            FaultSeverity::Minor,
            Some(drone),
            0,
        ).unwrap();

        assert_eq!(injector.active_fault_count(), 1);

        // Not expired yet
        injector.expire_faults(500);
        assert_eq!(injector.active_fault_count(), 1);

        // Now expired
        injector.expire_faults(1500);
        assert_eq!(injector.active_fault_count(), 0);
    }

    #[test]
    fn test_packet_drops() {
        let mut injector = FaultInjector::new_default();
        let drone = DroneId::new(1);

        injector.inject_packet_drops(drone, 0.5, 1000).unwrap();

        assert!((injector.get_drop_probability(drone) - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_gps_spoofing() {
        let mut injector = FaultInjector::new_default();
        let drone = DroneId::new(1);

        injector.inject_gps_spoofing(drone, (10.0, 20.0, 5.0), 1000).unwrap();

        let offset = injector.get_position_offset(drone).unwrap();
        assert!((offset.0 - 10.0).abs() < 0.001);
        assert!((offset.1 - 20.0).abs() < 0.001);
        assert!((offset.2 - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_clear_all() {
        let mut injector = FaultInjector::new_default();

        for i in 0..5 {
            injector.inject_fault(
                FaultType::SensorMalfunction,
                FaultSeverity::Minor,
                Some(DroneId::new(i)),
                1000,
            ).unwrap();
        }

        assert_eq!(injector.active_fault_count(), 5);

        injector.clear_all_injections();
        assert_eq!(injector.active_fault_count(), 0);
    }

    #[test]
    fn test_deterministic_rng() {
        let config = FaultInjectorConfig {
            deterministic: true,
            seed: 12345,
            ..Default::default()
        };

        let mut injector1 = FaultInjector::new(config.clone());
        let mut injector2 = FaultInjector::new(config);

        // Same seed should produce same sequence
        for _ in 0..10 {
            assert_eq!(injector1.next_random(), injector2.next_random());
        }
    }

    #[test]
    fn test_severity_limits() {
        let config = FaultInjectorConfig {
            max_severity: FaultSeverity::Moderate,
            ..Default::default()
        };
        let mut injector = FaultInjector::new(config);

        // Should succeed
        injector.inject_fault(
            FaultType::SensorMalfunction,
            FaultSeverity::Minor,
            None,
            1000,
        ).unwrap();

        // Should fail (severity too high)
        let result = injector.inject_fault(
            FaultType::SensorMalfunction,
            FaultSeverity::Critical,
            None,
            1000,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_disabled_fault_types() {
        let mut config = FaultInjectorConfig::default();
        config.disabled_fault_types.push(FaultType::Byzantine).ok();

        let mut injector = FaultInjector::new(config);

        // Should fail (fault type disabled)
        let result = injector.inject_fault(
            FaultType::Byzantine,
            FaultSeverity::Critical,
            None,
            1000,
        );
        assert!(result.is_err());
    }
}
