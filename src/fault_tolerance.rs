//! Fault tolerance and self-healing mechanisms
//!
//! Provides:
//! - Hardware fault detection
//! - Software error recovery
//! - Graceful degradation
//! - Self-healing protocols
//! - Redundancy management

use crate::types::*;
use heapless::{FnvIndexMap, Vec};

/// Fault severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FaultSeverity {
    /// Minor issue, can continue
    Minor = 0,
    /// Degraded performance
    Degraded = 1,
    /// Major fault, limited operation
    Major = 2,
    /// Critical fault, emergency landing required
    Critical = 3,
}

/// Fault types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    /// Communication loss
    CommLoss,
    /// GPS failure
    GpsFailure,
    /// Battery critical
    BatteryCritical,
    /// Sensor failure
    SensorFailure,
    /// Motor failure
    MotorFailure,
    /// Memory corruption
    MemoryError,
    /// Computation error
    ComputationError,
    /// Network partition
    NetworkPartition,
    /// Consensus failure
    ConsensusFailure,
    /// Unknown fault
    Unknown,
}

/// Fault record
#[derive(Debug, Clone, Copy)]
pub struct Fault {
    /// Fault type
    pub fault_type: FaultType,
    /// Severity
    pub severity: FaultSeverity,
    /// Timestamp
    pub timestamp: u64,
    /// Affected drone (if known)
    pub affected_drone: Option<DroneId>,
    /// Resolved flag
    pub resolved: bool,
}

/// Fault detector and recovery manager
pub struct FaultTolerance {
    /// Active faults
    active_faults: Vec<Fault, 100>,
    /// Fault history
    fault_history: Vec<Fault, 1000>,
    /// Health status per subsystem
    subsystem_health: SubsystemHealth,
    /// Redundancy manager
    redundancy: RedundancyManager,
    /// Watchdog timer
    watchdog_last_reset: u64,
    /// Watchdog timeout (ms)
    watchdog_timeout_ms: u32,
}

impl FaultTolerance {
    /// Create a new fault tolerance manager
    pub fn new() -> Self {
        Self {
            active_faults: Vec::new(),
            fault_history: Vec::new(),
            subsystem_health: SubsystemHealth::new(),
            redundancy: RedundancyManager::new(),
            watchdog_last_reset: Self::get_time(),
            watchdog_timeout_ms: 1000,
        }
    }

    /// Report a fault
    pub fn report_fault(
        &mut self,
        fault_type: FaultType,
        severity: FaultSeverity,
        affected_drone: Option<DroneId>,
    ) -> Result<()> {
        let fault = Fault {
            fault_type,
            severity,
            timestamp: Self::get_time(),
            affected_drone,
            resolved: false,
        };

        // Add to active faults
        self.active_faults
            .push(fault)
            .map_err(|_| SwarmError::BufferFull)?;

        // Add to history
        if self.fault_history.len() >= self.fault_history.capacity() {
            self.fault_history.remove(0);
        }
        self.fault_history
            .push(fault)
            .map_err(|_| SwarmError::BufferFull)?;

        // Attempt automatic recovery
        self.attempt_recovery(fault)?;

        Ok(())
    }

    /// Attempt automatic fault recovery
    fn attempt_recovery(&mut self, fault: Fault) -> Result<()> {
        match fault.fault_type {
            FaultType::CommLoss => {
                // Switch to backup communication channel
                self.redundancy.switch_to_backup_comm()?;
            }
            FaultType::GpsFailure => {
                // Switch to inertial navigation
                self.redundancy.enable_inertial_nav()?;
            }
            FaultType::BatteryCritical => {
                // Initiate return-to-base
                // This would be handled by higher-level logic
            }
            FaultType::SensorFailure => {
                // Use sensor fusion with redundant sensors
                self.redundancy.enable_sensor_fusion()?;
            }
            FaultType::MotorFailure => {
                // Adjust control to compensate
                self.redundancy.enable_motor_compensation()?;
            }
            FaultType::MemoryError => {
                // Clear affected memory region
                // Reset affected subsystem
            }
            FaultType::NetworkPartition => {
                // Attempt to rejoin network
                // Switch to autonomous operation if needed
            }
            FaultType::ConsensusFailure => {
                // Restart consensus protocol
            }
            _ => {
                // Unknown fault - log and continue
            }
        }
        Ok(())
    }

    /// Resolve a fault
    pub fn resolve_fault(&mut self, fault_type: FaultType) -> Result<()> {
        for fault in &mut self.active_faults {
            if fault.fault_type == fault_type && !fault.resolved {
                fault.resolved = true;
            }
        }

        // Remove resolved faults
        self.active_faults.retain(|f| !f.resolved);

        Ok(())
    }

    /// Check if system is healthy
    pub fn is_healthy(&self) -> bool {
        // No critical faults
        !self
            .active_faults
            .iter()
            .any(|f| f.severity == FaultSeverity::Critical)
    }

    /// Get highest severity active fault
    pub fn max_severity(&self) -> FaultSeverity {
        self.active_faults
            .iter()
            .map(|f| f.severity)
            .max()
            .unwrap_or(FaultSeverity::Minor)
    }

    /// Update subsystem health
    pub fn update_health(&mut self, subsystem: Subsystem, healthy: bool) {
        self.subsystem_health.update(subsystem, healthy);
    }

    /// Check subsystem health
    pub fn check_health(&self, subsystem: Subsystem) -> bool {
        self.subsystem_health.is_healthy(subsystem)
    }

    /// Reset watchdog timer
    pub fn reset_watchdog(&mut self) {
        self.watchdog_last_reset = Self::get_time();
    }

    /// Check watchdog timeout
    pub fn check_watchdog(&self) -> Result<()> {
        let elapsed = Self::get_time() - self.watchdog_last_reset;
        if elapsed > self.watchdog_timeout_ms as u64 {
            Err(SwarmError::Timeout)
        } else {
            Ok(())
        }
    }

    /// Get active fault count
    pub fn active_fault_count(&self) -> usize {
        self.active_faults.len()
    }

    /// Get fault statistics
    pub fn fault_statistics(&self) -> FaultStatistics {
        let mut stats = FaultStatistics::default();

        for fault in &self.fault_history {
            stats.total_faults += 1;
            match fault.severity {
                FaultSeverity::Minor => stats.minor_faults += 1,
                FaultSeverity::Degraded => stats.degraded_faults += 1,
                FaultSeverity::Major => stats.major_faults += 1,
                FaultSeverity::Critical => stats.critical_faults += 1,
            }
            if fault.resolved {
                stats.resolved_faults += 1;
            }
        }

        stats
    }

    /// Get time (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

impl Default for FaultTolerance {
    fn default() -> Self {
        Self::new()
    }
}

/// Subsystem identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Subsystem {
    /// Communication subsystem
    Communication,
    /// Navigation subsystem
    Navigation,
    /// Sensors
    Sensors,
    /// Motors/Actuators
    Actuators,
    /// Power management
    Power,
    /// Computing
    Computing,
    /// Consensus
    Consensus,
}

/// Health status of subsystems
pub struct SubsystemHealth {
    /// Health status map
    health: FnvIndexMap<u8, bool, 16>,
}

impl SubsystemHealth {
    /// Create new subsystem health tracker
    ///
    /// # Panics
    /// Panics if the FnvIndexMap capacity is less than 7 (the number of subsystems).
    /// This should never happen with the configured capacity of 16.
    pub fn new() -> Self {
        let mut health = FnvIndexMap::new();
        // Initialize all subsystems as healthy
        // These insertions should never fail since we have capacity for 16 entries
        // and only insert 7. We use expect() here since this is initialization code
        // that runs once and a failure indicates a severe configuration error.
        health
            .insert(Subsystem::Communication as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Navigation as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Sensors as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Actuators as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Power as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Computing as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");
        health
            .insert(Subsystem::Consensus as u8, true)
            .expect("SubsystemHealth capacity exceeded during init");

        Self { health }
    }

    /// Update subsystem health
    ///
    /// Updates always succeed since we only update existing entries (initialized in new())
    /// or add entries within our 16-entry capacity for 7 subsystems.
    pub fn update(&mut self, subsystem: Subsystem, healthy: bool) {
        // This should never fail since:
        // 1. We're updating an existing key (all 7 are pre-initialized), OR
        // 2. We have capacity for 16 entries and only 7 subsystems exist
        // We still handle the error gracefully to avoid silent failures
        if self.health.insert(subsystem as u8, healthy).is_err() {
            // This indicates a programming error - log it in debug builds
            #[cfg(debug_assertions)]
            panic!("SubsystemHealth: unexpected capacity error");
        }
    }

    /// Check if subsystem is healthy
    pub fn is_healthy(&self, subsystem: Subsystem) -> bool {
        *self.health.get(&(subsystem as u8)).unwrap_or(&false)
    }

    /// Check if all subsystems are healthy
    pub fn all_healthy(&self) -> bool {
        self.health.values().all(|&h| h)
    }
}

impl Default for SubsystemHealth {
    fn default() -> Self {
        Self::new()
    }
}

/// Redundancy manager for backup systems
pub struct RedundancyManager {
    /// Backup communication enabled
    backup_comm: bool,
    /// Inertial navigation enabled
    inertial_nav: bool,
    /// Sensor fusion enabled
    sensor_fusion: bool,
    /// Motor compensation enabled
    motor_compensation: bool,
}

impl RedundancyManager {
    /// Create new redundancy manager
    pub fn new() -> Self {
        Self {
            backup_comm: false,
            inertial_nav: false,
            sensor_fusion: false,
            motor_compensation: false,
        }
    }

    /// Switch to backup communication
    pub fn switch_to_backup_comm(&mut self) -> Result<()> {
        self.backup_comm = true;
        Ok(())
    }

    /// Enable inertial navigation
    pub fn enable_inertial_nav(&mut self) -> Result<()> {
        self.inertial_nav = true;
        Ok(())
    }

    /// Enable sensor fusion
    pub fn enable_sensor_fusion(&mut self) -> Result<()> {
        self.sensor_fusion = true;
        Ok(())
    }

    /// Enable motor compensation
    pub fn enable_motor_compensation(&mut self) -> Result<()> {
        self.motor_compensation = true;
        Ok(())
    }
}

impl Default for RedundancyManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Fault statistics
#[derive(Debug, Clone, Copy, Default)]
pub struct FaultStatistics {
    /// Total faults recorded
    pub total_faults: u32,
    /// Minor faults
    pub minor_faults: u32,
    /// Degraded faults
    pub degraded_faults: u32,
    /// Major faults
    pub major_faults: u32,
    /// Critical faults
    pub critical_faults: u32,
    /// Resolved faults
    pub resolved_faults: u32,
}

/// Health monitor that periodically checks system health
pub struct HealthMonitor {
    /// Check interval (ms)
    check_interval_ms: u32,
    /// Last check time
    last_check: u64,
}

impl HealthMonitor {
    /// Create new health monitor
    pub fn new(check_interval_ms: u32) -> Self {
        Self {
            check_interval_ms,
            last_check: 0,
        }
    }

    /// Perform health check (returns true if check was performed)
    pub fn check<F>(&mut self, mut check_fn: F) -> bool
    where
        F: FnMut() -> Result<()>,
    {
        let now = Self::get_time();
        if now - self.last_check >= self.check_interval_ms as u64 {
            self.last_check = now;
            check_fn().is_ok()
        } else {
            false
        }
    }

    /// Get time (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fault_reporting() {
        let mut ft = FaultTolerance::new();
        assert!(ft.is_healthy());

        ft.report_fault(FaultType::CommLoss, FaultSeverity::Minor, None)
            .unwrap();
        assert_eq!(ft.active_fault_count(), 1);
    }

    #[test]
    fn test_fault_resolution() {
        let mut ft = FaultTolerance::new();

        ft.report_fault(FaultType::CommLoss, FaultSeverity::Minor, None)
            .unwrap();
        assert_eq!(ft.active_fault_count(), 1);

        ft.resolve_fault(FaultType::CommLoss).unwrap();
        assert_eq!(ft.active_fault_count(), 0);
    }

    #[test]
    fn test_subsystem_health() {
        let mut health = SubsystemHealth::new();
        assert!(health.all_healthy());

        health.update(Subsystem::Communication, false);
        assert!(!health.all_healthy());
        assert!(!health.is_healthy(Subsystem::Communication));
    }
}
