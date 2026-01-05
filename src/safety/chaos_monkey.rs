//! Chaos Monkey - Chaos Engineering Controller
//!
//! Orchestrates fault injection scenarios for testing system resilience.
//! This module is feature-gated and should NEVER be included in production.
//!
//! Features:
//! - Predefined chaos scenarios
//! - Random fault injection at configurable rate
//! - Scenario scheduling and management
//! - Integration with fault injector and resilience scorer

use crate::safety::injector::{
    FaultId, FaultInjector, FaultInjectorConfig, FaultSeverity, FaultType,
};
use crate::types::{DroneId, Result, SwarmError};
use heapless::Vec;
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CHAOS SCENARIOS
// ═══════════════════════════════════════════════════════════════════════════

/// Predefined chaos scenario
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ChaosScenario {
    /// Single drone failure - test redundancy
    SingleDroneFailure,
    /// Multiple simultaneous failures
    MultipleDroneFailure,
    /// Network partition between groups
    NetworkPartition,
    /// Byzantine node fault
    ByzantineFault,
    /// Cascading failure simulation
    CascadingFailure,
    /// Gradual degradation
    GradualDegradation,
    /// Leader failure during consensus
    LeaderFailure,
    /// Random chaos - random faults at random times
    RandomChaos,
    /// Stress test - maximum concurrent faults
    StressTest,
    /// Recovery test - inject then clear faults
    RecoveryTest,
}

impl ChaosScenario {
    /// Get scenario description
    pub fn description(&self) -> &'static str {
        match self {
            ChaosScenario::SingleDroneFailure => "Single drone experiences complete failure",
            ChaosScenario::MultipleDroneFailure => "Multiple drones fail simultaneously",
            ChaosScenario::NetworkPartition => "Network splits into isolated partitions",
            ChaosScenario::ByzantineFault => "Byzantine node sends conflicting messages",
            ChaosScenario::CascadingFailure => "Failure spreads through the swarm",
            ChaosScenario::GradualDegradation => "System slowly degrades over time",
            ChaosScenario::LeaderFailure => "Cluster leader fails during operation",
            ChaosScenario::RandomChaos => "Random faults injected at random intervals",
            ChaosScenario::StressTest => "Maximum fault load to test limits",
            ChaosScenario::RecoveryTest => "Test recovery from multiple faults",
        }
    }

    /// Get expected duration (ms)
    pub fn expected_duration_ms(&self) -> u32 {
        match self {
            ChaosScenario::SingleDroneFailure => 10000,
            ChaosScenario::MultipleDroneFailure => 15000,
            ChaosScenario::NetworkPartition => 20000,
            ChaosScenario::ByzantineFault => 30000,
            ChaosScenario::CascadingFailure => 30000,
            ChaosScenario::GradualDegradation => 60000,
            ChaosScenario::LeaderFailure => 15000,
            ChaosScenario::RandomChaos => 60000,
            ChaosScenario::StressTest => 30000,
            ChaosScenario::RecoveryTest => 20000,
        }
    }

    /// Get severity level
    pub fn severity(&self) -> FaultSeverity {
        match self {
            ChaosScenario::SingleDroneFailure => FaultSeverity::Major,
            ChaosScenario::MultipleDroneFailure => FaultSeverity::Critical,
            ChaosScenario::NetworkPartition => FaultSeverity::Critical,
            ChaosScenario::ByzantineFault => FaultSeverity::Critical,
            ChaosScenario::CascadingFailure => FaultSeverity::Critical,
            ChaosScenario::GradualDegradation => FaultSeverity::Moderate,
            ChaosScenario::LeaderFailure => FaultSeverity::Major,
            ChaosScenario::RandomChaos => FaultSeverity::Moderate,
            ChaosScenario::StressTest => FaultSeverity::Critical,
            ChaosScenario::RecoveryTest => FaultSeverity::Major,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CHAOS CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for the chaos controller
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChaosConfig {
    /// Enable chaos injection
    pub enabled: bool,
    /// Injection rate (faults per second)
    pub injection_rate: f32,
    /// Maximum concurrent faults
    pub max_concurrent_faults: usize,
    /// Deterministic seed (None for random)
    pub deterministic_seed: Option<u64>,
    /// Target drones (empty = all drones)
    pub affected_drones: Vec<DroneId, 16>,
    /// Excluded drones (never target these)
    pub excluded_drones: Vec<DroneId, 16>,
    /// Auto-expire faults
    pub auto_expire: bool,
    /// Default fault duration (ms)
    pub default_fault_duration_ms: u32,
}

impl Default for ChaosConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            injection_rate: 0.1, // 1 fault per 10 seconds
            max_concurrent_faults: 5,
            deterministic_seed: None,
            affected_drones: Vec::new(),
            excluded_drones: Vec::new(),
            auto_expire: true,
            default_fault_duration_ms: 5000,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CHAOS STATE
// ═══════════════════════════════════════════════════════════════════════════

/// Current state of chaos controller
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ChaosState {
    /// Idle - no active scenario
    Idle,
    /// Running a scenario
    Running,
    /// Paused
    Paused,
    /// Scenario completed
    Completed,
}

/// Record of an injection event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InjectionEvent {
    /// Fault ID
    pub fault_id: FaultId,
    /// Fault type
    pub fault_type: FaultType,
    /// Subject drone
    pub subject: Option<DroneId>,
    /// Timestamp
    pub timestamp_ms: u64,
    /// Scenario that triggered this
    pub scenario: Option<ChaosScenario>,
}

/// Scenario execution context
#[derive(Debug, Clone)]
pub struct ScenarioContext {
    /// Active scenario
    pub scenario: ChaosScenario,
    /// Start time
    pub started_at_ms: u64,
    /// Expected end time
    pub expected_end_ms: u64,
    /// Faults injected by this scenario
    pub injected_faults: Vec<FaultId, 32>,
    /// Phase within scenario
    pub phase: u8,
}

// ═══════════════════════════════════════════════════════════════════════════
// CHAOS CONTROLLER
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum injection history
const MAX_INJECTION_HISTORY: usize = 100;

/// Chaos Monkey controller
///
/// Orchestrates fault injection for chaos engineering testing.
#[derive(Debug)]
pub struct ChaosController {
    /// Configuration
    config: ChaosConfig,
    /// Fault injector
    injector: FaultInjector,
    /// Current state
    state: ChaosState,
    /// Active scenario context
    active_scenario: Option<ScenarioContext>,
    /// Injection history
    history: Vec<InjectionEvent, MAX_INJECTION_HISTORY>,
    /// Known drones in the swarm
    known_drones: Vec<DroneId, 64>,
    /// Last injection time
    last_injection_ms: u64,
    /// Total scenarios run
    scenarios_run: u32,
}

impl ChaosController {
    /// Create a new chaos controller
    pub fn new(config: ChaosConfig) -> Self {
        let injector_config = FaultInjectorConfig {
            max_concurrent_faults: config.max_concurrent_faults,
            default_duration_ms: config.default_fault_duration_ms,
            deterministic: config.deterministic_seed.is_some(),
            seed: config.deterministic_seed.unwrap_or(0),
            ..Default::default()
        };

        Self {
            config,
            injector: FaultInjector::new(injector_config),
            state: ChaosState::Idle,
            active_scenario: None,
            history: Vec::new(),
            known_drones: Vec::new(),
            last_injection_ms: 0,
            scenarios_run: 0,
        }
    }

    /// Create with default configuration
    pub fn new_default() -> Self {
        Self::new(ChaosConfig::default())
    }

    /// Register a drone as a potential fault candidate
    pub fn register_drone(&mut self, drone_id: DroneId) {
        if !self.known_drones.contains(&drone_id) {
            self.known_drones.push(drone_id).ok();
        }
    }

    /// Remove a drone from fault candidates
    pub fn unregister_drone(&mut self, drone_id: DroneId) {
        if let Some(pos) = self.known_drones.iter().position(|&d| d == drone_id) {
            self.known_drones.remove(pos);
        }
    }

    /// Enable chaos injection
    pub fn enable(&mut self) {
        self.config.enabled = true;
    }

    /// Disable chaos injection
    pub fn disable(&mut self) {
        self.config.enabled = false;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    /// Get current state
    pub fn state(&self) -> ChaosState {
        self.state
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SCENARIO MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════

    /// Start a chaos scenario
    pub fn start_scenario(
        &mut self,
        scenario: ChaosScenario,
        timestamp_ms: u64,
    ) -> Result<()> {
        if !self.config.enabled {
            return Err(SwarmError::PermissionDenied);
        }

        if self.state == ChaosState::Running {
            return Err(SwarmError::PermissionDenied);
        }

        let context = ScenarioContext {
            scenario,
            started_at_ms: timestamp_ms,
            expected_end_ms: timestamp_ms + scenario.expected_duration_ms() as u64,
            injected_faults: Vec::new(),
            phase: 0,
        };

        self.active_scenario = Some(context);
        self.state = ChaosState::Running;
        self.scenarios_run += 1;

        // Execute initial phase
        self.execute_scenario_phase(timestamp_ms)?;

        Ok(())
    }

    /// Stop current scenario
    pub fn stop_scenario(&mut self) {
        if let Some(context) = &self.active_scenario {
            // Clear all faults from this scenario
            for fault_id in &context.injected_faults {
                self.injector.clear_fault(*fault_id).ok();
            }
        }

        self.active_scenario = None;
        self.state = ChaosState::Idle;
    }

    /// Pause current scenario
    pub fn pause(&mut self) {
        if self.state == ChaosState::Running {
            self.state = ChaosState::Paused;
        }
    }

    /// Resume paused scenario
    pub fn resume(&mut self) {
        if self.state == ChaosState::Paused {
            self.state = ChaosState::Running;
        }
    }

    /// Update chaos controller (call periodically)
    pub fn update(&mut self, timestamp_ms: u64) -> Result<()> {
        if !self.config.enabled || self.state != ChaosState::Running {
            return Ok(());
        }

        // Auto-expire faults
        if self.config.auto_expire {
            self.injector.expire_faults(timestamp_ms);
        }

        // Check if scenario is complete
        if let Some(context) = &self.active_scenario {
            if timestamp_ms >= context.expected_end_ms {
                self.state = ChaosState::Completed;
                return Ok(());
            }
        }

        // Execute scenario phase or random injection
        if self.active_scenario.is_some() {
            self.execute_scenario_phase(timestamp_ms)?;
        } else {
            self.random_injection(timestamp_ms)?;
        }

        Ok(())
    }

    /// Execute current scenario phase
    fn execute_scenario_phase(&mut self, timestamp_ms: u64) -> Result<()> {
        let scenario = match &self.active_scenario {
            Some(ctx) => ctx.scenario,
            None => return Ok(()),
        };

        match scenario {
            ChaosScenario::SingleDroneFailure => {
                self.execute_single_drone_failure(timestamp_ms)?;
            }
            ChaosScenario::MultipleDroneFailure => {
                self.execute_multiple_drone_failure(timestamp_ms)?;
            }
            ChaosScenario::NetworkPartition => {
                self.execute_network_partition(timestamp_ms)?;
            }
            ChaosScenario::ByzantineFault => {
                self.execute_byzantine_fault(timestamp_ms)?;
            }
            ChaosScenario::CascadingFailure => {
                self.execute_cascading_failure(timestamp_ms)?;
            }
            ChaosScenario::GradualDegradation => {
                self.execute_gradual_degradation(timestamp_ms)?;
            }
            ChaosScenario::LeaderFailure => {
                self.execute_leader_failure(timestamp_ms)?;
            }
            ChaosScenario::RandomChaos => {
                self.random_injection(timestamp_ms)?;
            }
            ChaosScenario::StressTest => {
                self.execute_stress_test(timestamp_ms)?;
            }
            ChaosScenario::RecoveryTest => {
                self.execute_recovery_test(timestamp_ms)?;
            }
        }

        Ok(())
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SCENARIO IMPLEMENTATIONS
    // ═══════════════════════════════════════════════════════════════════════

    fn execute_single_drone_failure(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_fault(
                FaultType::CommunicationFailure,
                FaultSeverity::Critical,
                Some(drone_id),
                timestamp_ms,
            )?;

            self.record_injection(fault_id, FaultType::CommunicationFailure, Some(drone_id), timestamp_ms);
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_multiple_drone_failure(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        if phase > 0 {
            return Ok(());
        }

        // Fail 3 drones
        for _ in 0..3 {
            if let Some(drone_id) = self.select_random_drone() {
                let fault_id = self.injector.inject_fault(
                    FaultType::CommunicationFailure,
                    FaultSeverity::Critical,
                    Some(drone_id),
                    timestamp_ms,
                )?;
                self.record_injection(fault_id, FaultType::CommunicationFailure, Some(drone_id), timestamp_ms);
            }
        }

        self.advance_phase();
        Ok(())
    }

    fn execute_network_partition(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        if phase > 0 {
            return Ok(());
        }

        // Split drones into two groups
        let mut groups: Vec<(DroneId, u8), 64> = Vec::new();
        for (i, &drone_id) in self.known_drones.iter().enumerate() {
            let group = (i % 2) as u8;
            groups.push((drone_id, group)).ok();
        }

        let fault_id = self.injector.inject_network_partition(&groups, timestamp_ms)?;
        self.record_injection(fault_id, FaultType::NetworkPartition, None, timestamp_ms);
        self.advance_phase();

        Ok(())
    }

    fn execute_byzantine_fault(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        if phase > 0 {
            return Ok(());
        }

        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_byzantine_behavior(drone_id, timestamp_ms)?;
            self.record_injection(fault_id, FaultType::Byzantine, Some(drone_id), timestamp_ms);
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_cascading_failure(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        let ctx = match &self.active_scenario {
            Some(c) => c,
            None => return Ok(()),
        };

        // Inject one fault per 5 seconds
        let elapsed = timestamp_ms.saturating_sub(ctx.started_at_ms);
        let expected_phase = (elapsed / 5000) as u8;

        if phase >= expected_phase || phase >= 5 {
            return Ok(()); // Already caught up or max phases reached
        }

        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_fault(
                FaultType::CommunicationFailure,
                FaultSeverity::Major,
                Some(drone_id),
                timestamp_ms,
            )?;
            self.record_injection(fault_id, FaultType::CommunicationFailure, Some(drone_id), timestamp_ms);
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_gradual_degradation(&mut self, timestamp_ms: u64) -> Result<()> {
        let ctx = match &self.active_scenario {
            Some(c) => c,
            None => return Ok(()),
        };

        let elapsed = timestamp_ms.saturating_sub(ctx.started_at_ms);
        let interval = 10000u64; // 10 seconds

        if elapsed / interval <= self.get_scenario_phase() as u64 {
            return Ok(()); // Not time for next injection
        }

        // Inject minor faults that gradually worsen
        let severity = match self.get_scenario_phase() {
            0..=2 => FaultSeverity::Minor,
            3..=4 => FaultSeverity::Moderate,
            _ => FaultSeverity::Major,
        };

        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_fault(
                FaultType::SensorMalfunction,
                severity,
                Some(drone_id),
                timestamp_ms,
            )?;
            self.record_injection(fault_id, FaultType::SensorMalfunction, Some(drone_id), timestamp_ms);
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_leader_failure(&mut self, timestamp_ms: u64) -> Result<()> {
        let phase = self.get_scenario_phase();
        if phase > 0 {
            return Ok(());
        }

        // Target first drone (assume it's the leader for testing)
        if let Some(&drone_id) = self.known_drones.first() {
            let fault_id = self.injector.inject_fault(
                FaultType::CommunicationFailure,
                FaultSeverity::Critical,
                Some(drone_id),
                timestamp_ms,
            )?;
            self.record_injection(fault_id, FaultType::CommunicationFailure, Some(drone_id), timestamp_ms);
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_stress_test(&mut self, timestamp_ms: u64) -> Result<()> {
        // Inject faults rapidly
        let interval = 1000u64; // 1 second

        if timestamp_ms < self.last_injection_ms + interval {
            return Ok(());
        }

        // Inject multiple fault types
        let fault_types = [
            FaultType::CommunicationFailure,
            FaultType::SensorMalfunction,
            FaultType::CpuOverload,
            FaultType::ClockDrift,
        ];

        let phase = self.get_scenario_phase() as usize;
        let fault_type = fault_types[phase % fault_types.len()];

        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_fault(
                fault_type,
                FaultSeverity::Moderate,
                Some(drone_id),
                timestamp_ms,
            )?;
            self.record_injection(fault_id, fault_type, Some(drone_id), timestamp_ms);
            self.last_injection_ms = timestamp_ms;
            self.advance_phase();
        }

        Ok(())
    }

    fn execute_recovery_test(&mut self, timestamp_ms: u64) -> Result<()> {
        let ctx = match &self.active_scenario {
            Some(c) => c,
            None => return Ok(()),
        };

        let elapsed = timestamp_ms.saturating_sub(ctx.started_at_ms);
        let phase = self.get_scenario_phase();

        // Phase 0: Inject faults (first 5 seconds)
        if phase == 0 && elapsed < 5000 {
            for _ in 0..3 {
                if let Some(drone_id) = self.select_random_drone() {
                    let fault_id = self.injector.inject_fault(
                        FaultType::CommunicationFailure,
                        FaultSeverity::Major,
                        Some(drone_id),
                        timestamp_ms,
                    )?;
                    self.record_injection(fault_id, FaultType::CommunicationFailure, Some(drone_id), timestamp_ms);
                }
            }
            self.advance_phase();
        }
        // Phase 1: Clear faults (after 10 seconds)
        else if phase == 1 && elapsed >= 10000 {
            self.injector.clear_all_injections();
            self.advance_phase();
        }

        Ok(())
    }

    // ═══════════════════════════════════════════════════════════════════════
    // RANDOM INJECTION
    // ═══════════════════════════════════════════════════════════════════════

    /// Perform random fault injection
    pub fn random_injection(&mut self, timestamp_ms: u64) -> Result<()> {
        if !self.config.enabled {
            return Ok(());
        }

        // Check injection rate
        let interval_ms = (1000.0 / self.config.injection_rate) as u64;
        if timestamp_ms < self.last_injection_ms + interval_ms {
            return Ok(());
        }

        // Check concurrent fault limit
        if self.injector.active_fault_count() >= self.config.max_concurrent_faults {
            return Ok(());
        }

        // Select random fault type
        let fault_types = [
            FaultType::CommunicationFailure,
            FaultType::SensorMalfunction,
            FaultType::MotorFailure,
            FaultType::BatteryDrain,
            FaultType::CpuOverload,
            FaultType::ClockDrift,
        ];

        let random_val = self.injector.next_random();
        let fault_type = fault_types[(random_val as usize) % fault_types.len()];

        // Select random severity (weighted towards minor)
        let severity = match random_val % 10 {
            0..=5 => FaultSeverity::Minor,
            6..=8 => FaultSeverity::Moderate,
            _ => FaultSeverity::Major,
        };

        if let Some(drone_id) = self.select_random_drone() {
            let fault_id = self.injector.inject_fault(
                fault_type,
                severity,
                Some(drone_id),
                timestamp_ms,
            )?;

            self.record_injection(fault_id, fault_type, Some(drone_id), timestamp_ms);
            self.last_injection_ms = timestamp_ms;
        }

        Ok(())
    }

    // ═══════════════════════════════════════════════════════════════════════
    // HELPER METHODS
    // ═══════════════════════════════════════════════════════════════════════

    fn select_random_drone(&mut self) -> Option<DroneId> {
        let eligible: Vec<DroneId, 64> = self.known_drones
            .iter()
            .filter(|d| {
                // Check if in target list (if specified)
                let in_targets = self.config.affected_drones.is_empty()
                    || self.config.affected_drones.contains(d);

                // Check if not excluded
                let not_excluded = !self.config.excluded_drones.contains(d);

                in_targets && not_excluded
            })
            .copied()
            .collect();

        if eligible.is_empty() {
            return None;
        }

        let idx = (self.injector.next_random() as usize) % eligible.len();
        Some(eligible[idx])
    }

    fn get_scenario_phase(&self) -> u8 {
        self.active_scenario.as_ref().map(|c| c.phase).unwrap_or(0)
    }

    fn advance_phase(&mut self) {
        if let Some(ctx) = &mut self.active_scenario {
            ctx.phase = ctx.phase.saturating_add(1);
        }
    }

    fn record_injection(
        &mut self,
        fault_id: FaultId,
        fault_type: FaultType,
        subject: Option<DroneId>,
        timestamp_ms: u64,
    ) {
        let event = InjectionEvent {
            fault_id,
            fault_type,
            subject,
            timestamp_ms,
            scenario: self.active_scenario.as_ref().map(|c| c.scenario),
        };

        // Add to history (circular buffer)
        if self.history.len() >= MAX_INJECTION_HISTORY {
            self.history.remove(0);
        }
        self.history.push(event).ok();

        // Track in scenario context
        if let Some(ctx) = &mut self.active_scenario {
            ctx.injected_faults.push(fault_id).ok();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // QUERY METHODS
    // ═══════════════════════════════════════════════════════════════════════

    /// Get injection history
    pub fn get_history(&self) -> &[InjectionEvent] {
        &self.history
    }

    /// Get active scenario
    pub fn active_scenario(&self) -> Option<&ScenarioContext> {
        self.active_scenario.as_ref()
    }

    /// Get fault injector (for querying fault state)
    pub fn injector(&self) -> &FaultInjector {
        &self.injector
    }

    /// Get mutable fault injector
    pub fn injector_mut(&mut self) -> &mut FaultInjector {
        &mut self.injector
    }

    /// Get number of scenarios run
    pub fn scenarios_run(&self) -> u32 {
        self.scenarios_run
    }

    /// Get current injection count
    pub fn injection_count(&self) -> u64 {
        self.injector.injection_count()
    }

    /// Get active fault count
    pub fn active_fault_count(&self) -> usize {
        self.injector.active_fault_count()
    }

    /// Clear all state
    pub fn reset(&mut self) {
        self.stop_scenario();
        self.injector.clear_all_injections();
        self.history.clear();
        self.last_injection_ms = 0;
    }
}

impl Default for ChaosController {
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

    fn setup_controller() -> ChaosController {
        let config = ChaosConfig {
            enabled: true,
            ..Default::default()
        };
        let mut controller = ChaosController::new(config);

        // Register some drones
        for i in 1..=10 {
            controller.register_drone(DroneId::new(i));
        }

        controller
    }

    #[test]
    fn test_controller_creation() {
        let controller = ChaosController::new_default();
        assert!(!controller.is_enabled());
        assert_eq!(controller.state(), ChaosState::Idle);
    }

    #[test]
    fn test_enable_disable() {
        let mut controller = ChaosController::new_default();

        controller.enable();
        assert!(controller.is_enabled());

        controller.disable();
        assert!(!controller.is_enabled());
    }

    #[test]
    fn test_drone_registration() {
        let mut controller = ChaosController::new_default();

        controller.register_drone(DroneId::new(1));
        controller.register_drone(DroneId::new(2));

        // Re-registering shouldn't duplicate
        controller.register_drone(DroneId::new(1));

        controller.unregister_drone(DroneId::new(1));
    }

    #[test]
    fn test_single_drone_failure_scenario() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::SingleDroneFailure, 0).unwrap();
        assert_eq!(controller.state(), ChaosState::Running);
        assert_eq!(controller.active_fault_count(), 1);
    }

    #[test]
    fn test_multiple_drone_failure_scenario() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::MultipleDroneFailure, 0).unwrap();
        assert_eq!(controller.active_fault_count(), 3);
    }

    #[test]
    fn test_network_partition_scenario() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::NetworkPartition, 0).unwrap();

        // Drones in same group should communicate
        assert!(controller.injector().can_communicate(DroneId::new(1), DroneId::new(3)));
        // Drones in different groups should not
        assert!(!controller.injector().can_communicate(DroneId::new(1), DroneId::new(2)));
    }

    #[test]
    fn test_byzantine_scenario() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::ByzantineFault, 0).unwrap();
        assert_eq!(controller.active_fault_count(), 1);
    }

    #[test]
    fn test_stop_scenario() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::MultipleDroneFailure, 0).unwrap();
        assert!(controller.active_fault_count() > 0);

        controller.stop_scenario();
        assert_eq!(controller.state(), ChaosState::Idle);
        assert_eq!(controller.active_fault_count(), 0);
    }

    #[test]
    fn test_pause_resume() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::SingleDroneFailure, 0).unwrap();

        controller.pause();
        assert_eq!(controller.state(), ChaosState::Paused);

        controller.resume();
        assert_eq!(controller.state(), ChaosState::Running);
    }

    #[test]
    fn test_random_injection() {
        let config = ChaosConfig {
            enabled: true,
            injection_rate: 10.0, // Fast for testing
            ..Default::default()
        };
        let mut controller = ChaosController::new(config);

        for i in 1..=10 {
            controller.register_drone(DroneId::new(i));
        }

        // Perform injections
        for t in 0..10 {
            controller.random_injection(t * 200).ok();
        }

        assert!(controller.injection_count() > 0);
    }

    #[test]
    fn test_injection_history() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::MultipleDroneFailure, 0).unwrap();

        let history = controller.get_history();
        assert_eq!(history.len(), 3);

        for event in history {
            assert!(event.scenario.is_some());
            assert_eq!(event.scenario.unwrap(), ChaosScenario::MultipleDroneFailure);
        }
    }

    #[test]
    fn test_scenario_completion() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::SingleDroneFailure, 0).unwrap();

        // Simulate time passing beyond scenario duration
        controller.update(15000).unwrap();

        assert_eq!(controller.state(), ChaosState::Completed);
    }

    #[test]
    fn test_excluded_drones() {
        let config = ChaosConfig {
            enabled: true,
            excluded_drones: {
                let mut v = Vec::new();
                v.push(DroneId::new(1)).ok();
                v.push(DroneId::new(2)).ok();
                v
            },
            ..Default::default()
        };
        let mut controller = ChaosController::new(config);

        for i in 1..=5 {
            controller.register_drone(DroneId::new(i));
        }

        // Run many random injections
        for t in 0..100 {
            controller.random_injection(t * 100).ok();
        }

        // Check that excluded drones were never targeted
        for event in controller.get_history() {
            if let Some(drone_id) = event.subject {
                assert!(drone_id.0 != 1 && drone_id.0 != 2);
            }
        }
    }

    #[test]
    fn test_reset() {
        let mut controller = setup_controller();

        controller.start_scenario(ChaosScenario::MultipleDroneFailure, 0).unwrap();
        assert!(controller.active_fault_count() > 0);

        controller.reset();
        assert_eq!(controller.state(), ChaosState::Idle);
        assert_eq!(controller.active_fault_count(), 0);
        assert!(controller.get_history().is_empty());
    }
}
