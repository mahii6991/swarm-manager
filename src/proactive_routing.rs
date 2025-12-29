//! Proactive Routing Manager
//!
//! Uses trajectory prediction and link quality analysis to
//! pre-establish alternative routes before link failures occur.
//!
//! Key features:
//! - Degrading link detection
//! - Alternative route discovery
//! - Quality-aware route selection
//! - Seamless route switching

use crate::link_predictor::{LinkHealthStatus, LinkPredictor, LinkPredictorConfig, LinkTrend};
use crate::trajectory_predictor::{KalmanConfig, TrajectoryPredictor};
use crate::types::{DroneId, Position, Result, SwarmError, Velocity};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for proactive routing
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ProactiveRoutingConfig {
    /// Enable proactive routing
    pub enabled: bool,
    /// Prediction horizon for link quality (ms)
    pub prediction_horizon_ms: u32,
    /// Minimum link quality to use a route
    pub min_route_quality: f32,
    /// Quality threshold to trigger preemptive rerouting
    pub reroute_threshold: f32,
    /// Maximum alternative routes to maintain
    pub max_alternatives: usize,
    /// Route scan interval (ms)
    pub scan_interval_ms: u32,
    /// Kalman filter configuration
    pub kalman_config: KalmanConfig,
    /// Link predictor configuration
    pub link_config: LinkPredictorConfig,
}

impl Default for ProactiveRoutingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            prediction_horizon_ms: 3000,
            min_route_quality: 0.2,
            reroute_threshold: 0.4,
            max_alternatives: 3,
            scan_interval_ms: 500,
            kalman_config: KalmanConfig::default(),
            link_config: LinkPredictorConfig::default(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ROUTE TYPES
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum hops in a route
pub const MAX_ROUTE_HOPS: usize = 8;

/// Maximum neighbors to track
pub const MAX_NEIGHBORS: usize = 32;

/// A route to a destination
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictiveRoute {
    /// Destination drone
    pub destination: DroneId,
    /// Path to destination (sequence of next hops)
    pub path: Vec<DroneId, MAX_ROUTE_HOPS>,
    /// Current quality estimate
    pub quality: f32,
    /// Predicted quality at horizon
    pub predicted_quality: f32,
    /// Estimated time to failure (ms), if degrading
    pub time_to_failure_ms: Option<u32>,
    /// Route stability score (0.0-1.0)
    pub stability: f32,
    /// Last update timestamp
    pub last_updated_ms: u64,
    /// Is this a backup route?
    pub is_backup: bool,
}

impl PredictiveRoute {
    /// Create a new direct route
    pub fn direct(destination: DroneId, quality: f32, timestamp_ms: u64) -> Self {
        let mut path = Vec::new();
        path.push(destination).ok();

        Self {
            destination,
            path,
            quality,
            predicted_quality: quality,
            time_to_failure_ms: None,
            stability: 1.0,
            last_updated_ms: timestamp_ms,
            is_backup: false,
        }
    }

    /// Get next hop
    pub fn next_hop(&self) -> Option<DroneId> {
        self.path.first().copied()
    }

    /// Get hop count
    pub fn hop_count(&self) -> usize {
        self.path.len()
    }

    /// Check if route is viable
    pub fn is_viable(&self, min_quality: f32) -> bool {
        self.quality >= min_quality && self.predicted_quality >= min_quality * 0.5
    }

    /// Calculate route score for comparison
    pub fn score(&self) -> f32 {
        // Score combines quality, stability, and hop count
        let hop_penalty = 1.0 / (1.0 + self.hop_count() as f32 * 0.2);
        self.quality * self.stability * hop_penalty
    }
}

/// Route status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RouteStatus {
    /// Route is healthy
    Healthy,
    /// Route is degrading, consider alternatives
    Degrading,
    /// Route is at risk, switch recommended
    AtRisk,
    /// Route is broken, switch required
    Broken,
}

impl RouteStatus {
    /// Check if action is needed
    pub fn needs_action(&self) -> bool {
        matches!(self, RouteStatus::AtRisk | RouteStatus::Broken)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NEIGHBOR TRACKING
// ═══════════════════════════════════════════════════════════════════════════

/// Tracked neighbor with predictors
#[derive(Debug, Clone)]
pub struct TrackedNeighbor {
    /// Neighbor ID
    pub id: DroneId,
    /// Trajectory predictor
    pub trajectory: TrajectoryPredictor,
    /// Link predictor
    pub link: LinkPredictor,
    /// Is this a direct neighbor?
    pub is_direct: bool,
    /// Last contact timestamp
    pub last_contact_ms: u64,
}

impl TrackedNeighbor {
    /// Create a new tracked neighbor
    pub fn new(id: DroneId, config: &ProactiveRoutingConfig) -> Self {
        Self {
            id,
            trajectory: TrajectoryPredictor::new(config.kalman_config),
            link: LinkPredictor::new(id, config.link_config),
            is_direct: true,
            last_contact_ms: 0,
        }
    }

    /// Update with new measurement
    pub fn update(
        &mut self,
        position: Position,
        velocity: Option<Velocity>,
        rssi: i8,
        quality: f32,
        timestamp_ms: u64,
    ) -> Result<()> {
        // Update trajectory
        if let Some(vel) = velocity {
            self.trajectory.update_with_velocity(position, vel, timestamp_ms)?;
        } else {
            self.trajectory.update(position, timestamp_ms)?;
        }

        // Update link
        self.link.update_basic(rssi, quality, timestamp_ms)?;

        self.last_contact_ms = timestamp_ms;
        Ok(())
    }

    /// Get predicted position
    pub fn predict_position(&self, time_ahead_ms: u32) -> Result<Position> {
        self.trajectory.predict(time_ahead_ms).map(|p| p.position)
    }

    /// Get predicted link quality
    pub fn predict_quality(&self, time_ahead_ms: u32) -> Result<f32> {
        self.link.predict_quality(time_ahead_ms)
    }

    /// Check if neighbor is at risk of disconnect
    pub fn is_at_risk(&self) -> bool {
        self.link.is_at_risk()
    }

    /// Get time to disconnect
    pub fn time_to_disconnect(&self) -> Option<u32> {
        self.link.time_to_disconnect()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PROACTIVE ROUTE MANAGER
// ═══════════════════════════════════════════════════════════════════════════

/// Proactive routing manager
///
/// Monitors link quality and trajectory predictions to
/// preemptively switch routes before failures occur.
#[derive(Debug, Clone)]
pub struct ProactiveRouteManager {
    /// Our drone ID (used for route identification in multi-hop scenarios)
    #[allow(dead_code)]
    local_id: DroneId,
    /// Tracked neighbors
    neighbors: FnvIndexMap<u64, TrackedNeighbor, MAX_NEIGHBORS>,
    /// Primary routes to destinations
    primary_routes: FnvIndexMap<u64, PredictiveRoute, MAX_NEIGHBORS>,
    /// Backup routes (alternatives)
    backup_routes: FnvIndexMap<u64, Vec<PredictiveRoute, 3>, MAX_NEIGHBORS>,
    /// Configuration
    config: ProactiveRoutingConfig,
    /// Last scan timestamp
    last_scan_ms: u64,
    /// Routes currently at risk
    at_risk_routes: Vec<DroneId, MAX_NEIGHBORS>,
    /// Statistics
    stats: RoutingStats,
}

/// Routing statistics
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct RoutingStats {
    /// Proactive route switches
    pub proactive_switches: u64,
    /// Reactive route switches (after failure)
    pub reactive_switches: u64,
    /// Routes predicted to fail
    pub predicted_failures: u64,
    /// Actual route failures
    pub actual_failures: u64,
    /// Successful predictions
    pub successful_predictions: u64,
}

impl ProactiveRouteManager {
    /// Create a new proactive route manager
    pub fn new(local_id: DroneId, config: ProactiveRoutingConfig) -> Self {
        Self {
            local_id,
            neighbors: FnvIndexMap::new(),
            primary_routes: FnvIndexMap::new(),
            backup_routes: FnvIndexMap::new(),
            config,
            last_scan_ms: 0,
            at_risk_routes: Vec::new(),
            stats: RoutingStats::default(),
        }
    }

    /// Create with default config
    pub fn new_default(local_id: DroneId) -> Self {
        Self::new(local_id, ProactiveRoutingConfig::default())
    }

    /// Update neighbor information
    pub fn update_neighbor(
        &mut self,
        neighbor_id: DroneId,
        position: Position,
        velocity: Option<Velocity>,
        rssi: i8,
        quality: f32,
        timestamp_ms: u64,
    ) -> Result<()> {
        // Get or create neighbor
        let neighbor = if let Some(n) = self.neighbors.get_mut(&neighbor_id.as_u64()) {
            n
        } else {
            let tracked = TrackedNeighbor::new(neighbor_id, &self.config);
            self.neighbors
                .insert(neighbor_id.as_u64(), tracked)
                .map_err(|_| SwarmError::BufferFull)?;
            self.neighbors.get_mut(&neighbor_id.as_u64()).unwrap()
        };

        // Update
        neighbor.update(position, velocity, rssi, quality, timestamp_ms)?;

        // Update/create direct route
        self.update_direct_route(neighbor_id, quality, timestamp_ms)?;

        Ok(())
    }

    /// Update direct route to neighbor
    fn update_direct_route(
        &mut self,
        neighbor_id: DroneId,
        quality: f32,
        timestamp_ms: u64,
    ) -> Result<()> {
        let route = if let Some(r) = self.primary_routes.get_mut(&neighbor_id.as_u64()) {
            r.quality = quality;
            r.last_updated_ms = timestamp_ms;

            // Update predicted quality
            if let Some(neighbor) = self.neighbors.get(&neighbor_id.as_u64()) {
                r.predicted_quality = neighbor
                    .predict_quality(self.config.prediction_horizon_ms)
                    .unwrap_or(quality);
                r.time_to_failure_ms = neighbor.time_to_disconnect();
            }
            return Ok(());
        } else {
            PredictiveRoute::direct(neighbor_id, quality, timestamp_ms)
        };

        self.primary_routes
            .insert(neighbor_id.as_u64(), route)
            .map_err(|_| SwarmError::BufferFull)?;

        Ok(())
    }

    /// Scan for degrading links and prepare alternatives
    pub fn scan_routes(&mut self, current_time_ms: u64) -> Result<Vec<RouteAlert, 10>> {
        // Check scan interval
        if current_time_ms.saturating_sub(self.last_scan_ms) < self.config.scan_interval_ms as u64 {
            return Ok(Vec::new());
        }
        self.last_scan_ms = current_time_ms;

        self.at_risk_routes.clear();

        // First pass: update predictions and collect route info
        let mut route_updates: Vec<(u64, f32, Option<u32>), MAX_NEIGHBORS> = Vec::new();

        for (dest_id, _route) in self.primary_routes.iter() {
            if let Some(neighbor) = self.neighbors.get(dest_id) {
                let predicted = neighbor
                    .predict_quality(self.config.prediction_horizon_ms)
                    .unwrap_or(0.5);
                let ttf = neighbor.time_to_disconnect();
                route_updates.push((*dest_id, predicted, ttf)).ok();
            }
        }

        // Apply updates
        for (dest_id, predicted, ttf) in &route_updates {
            if let Some(route) = self.primary_routes.get_mut(dest_id) {
                route.predicted_quality = *predicted;
                route.time_to_failure_ms = *ttf;
            }
        }

        // Second pass: evaluate and create alerts
        let mut alerts = Vec::new();
        let config = self.config; // Copy config to avoid borrow issues

        for (dest_id, route) in self.primary_routes.iter() {
            let status = Self::evaluate_route_status_static(route, &config);

            if status.needs_action() {
                let dest = DroneId::new(*dest_id);
                self.at_risk_routes.push(dest).ok();

                // Check backup availability
                let has_backup = self.backup_routes
                    .get(dest_id)
                    .map(|b| b.iter().any(|r| r.is_viable(config.min_route_quality)))
                    .unwrap_or(false);

                let alert = RouteAlert {
                    destination: dest,
                    status,
                    current_quality: route.quality,
                    predicted_quality: route.predicted_quality,
                    time_to_failure_ms: route.time_to_failure_ms,
                    has_backup,
                };
                alerts.push(alert).ok();

                self.stats.predicted_failures += 1;
            }
        }

        Ok(alerts)
    }

    /// Evaluate route status (static version to avoid borrow issues)
    fn evaluate_route_status_static(route: &PredictiveRoute, config: &ProactiveRoutingConfig) -> RouteStatus {
        if route.quality < config.min_route_quality {
            RouteStatus::Broken
        } else if route.predicted_quality < config.min_route_quality {
            RouteStatus::AtRisk
        } else if route.quality < config.reroute_threshold
            || route.predicted_quality < config.reroute_threshold
        {
            RouteStatus::Degrading
        } else {
            RouteStatus::Healthy
        }
    }

    /// Evaluate route status (instance method for convenience)
    #[allow(dead_code)]
    fn evaluate_route_status(&self, route: &PredictiveRoute) -> RouteStatus {
        Self::evaluate_route_status_static(route, &self.config)
    }

    /// Check if we have a viable backup route
    #[allow(dead_code)]
    fn has_viable_backup(&self, destination: DroneId) -> bool {
        if let Some(backups) = self.backup_routes.get(&destination.as_u64()) {
            backups
                .iter()
                .any(|r| r.is_viable(self.config.min_route_quality))
        } else {
            false
        }
    }

    /// Get best route to destination
    pub fn get_best_route(&self, destination: DroneId) -> Option<&PredictiveRoute> {
        // Check primary route
        if let Some(primary) = self.primary_routes.get(&destination.as_u64()) {
            if primary.is_viable(self.config.min_route_quality) {
                return Some(primary);
            }
        }

        // Check backups
        if let Some(backups) = self.backup_routes.get(&destination.as_u64()) {
            let best = backups
                .iter()
                .filter(|r| r.is_viable(self.config.min_route_quality))
                .max_by(|a, b| a.score().partial_cmp(&b.score()).unwrap());
            return best;
        }

        None
    }

    /// Add a multi-hop route
    pub fn add_route(
        &mut self,
        destination: DroneId,
        path: Vec<DroneId, MAX_ROUTE_HOPS>,
        quality: f32,
        timestamp_ms: u64,
    ) -> Result<()> {
        let route = PredictiveRoute {
            destination,
            path,
            quality,
            predicted_quality: quality,
            time_to_failure_ms: None,
            stability: 0.8,
            last_updated_ms: timestamp_ms,
            is_backup: false,
        };

        // Add as primary or backup
        if self.primary_routes.contains_key(&destination.as_u64()) {
            // Add as backup
            let backups = if let Some(b) = self.backup_routes.get_mut(&destination.as_u64()) {
                b
            } else {
                self.backup_routes
                    .insert(destination.as_u64(), Vec::new())
                    .map_err(|_| SwarmError::BufferFull)?;
                self.backup_routes.get_mut(&destination.as_u64()).unwrap()
            };

            if backups.len() < self.config.max_alternatives {
                let mut backup_route = route;
                backup_route.is_backup = true;
                backups.push(backup_route).map_err(|_| SwarmError::BufferFull)?;
            }
        } else {
            // Add as primary
            self.primary_routes
                .insert(destination.as_u64(), route)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(())
    }

    /// Switch to backup route
    pub fn switch_to_backup(&mut self, destination: DroneId) -> Result<bool> {
        // Get best backup
        let best_backup = if let Some(backups) = self.backup_routes.get_mut(&destination.as_u64())
        {
            let best_idx = backups
                .iter()
                .enumerate()
                .filter(|(_, r)| r.is_viable(self.config.min_route_quality))
                .max_by(|(_, a), (_, b)| a.score().partial_cmp(&b.score()).unwrap())
                .map(|(i, _)| i);

            if let Some(idx) = best_idx {
                Some(backups.swap_remove(idx))
            } else {
                None
            }
        } else {
            None
        };

        if let Some(mut new_primary) = best_backup {
            new_primary.is_backup = false;

            // Move old primary to backups
            if let Some(old_primary) = self.primary_routes.remove(&destination.as_u64()) {
                let mut old = old_primary;
                old.is_backup = true;

                if let Some(backups) = self.backup_routes.get_mut(&destination.as_u64()) {
                    backups.push(old).ok();
                } else {
                    let mut new_backups = Vec::new();
                    new_backups.push(old).ok();
                    self.backup_routes.insert(destination.as_u64(), new_backups).ok();
                }
            }

            // Set new primary
            self.primary_routes
                .insert(destination.as_u64(), new_primary)
                .map_err(|_| SwarmError::BufferFull)?;

            self.stats.proactive_switches += 1;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Get routes at risk
    pub fn routes_at_risk(&self) -> &[DroneId] {
        &self.at_risk_routes
    }

    /// Get routing statistics
    pub fn stats(&self) -> &RoutingStats {
        &self.stats
    }

    /// Get neighbor count
    pub fn neighbor_count(&self) -> usize {
        self.neighbors.len()
    }

    /// Get route count
    pub fn route_count(&self) -> usize {
        self.primary_routes.len()
    }

    /// Remove stale neighbors
    pub fn prune_stale(&mut self, timeout_ms: u64, current_time_ms: u64) {
        self.neighbors.retain(|_, n| {
            current_time_ms.saturating_sub(n.last_contact_ms) < timeout_ms
        });
    }

    /// Get link health for a neighbor
    pub fn get_link_health(&self, neighbor_id: DroneId) -> Option<LinkHealthStatus> {
        self.neighbors
            .get(&neighbor_id.as_u64())
            .map(|n| n.link.health_status())
    }

    /// Get link trend for a neighbor
    pub fn get_link_trend(&self, neighbor_id: DroneId) -> Option<LinkTrend> {
        self.neighbors
            .get(&neighbor_id.as_u64())
            .map(|n| n.link.trend())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ROUTE ALERT
// ═══════════════════════════════════════════════════════════════════════════

/// Alert about route status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteAlert {
    /// Affected destination
    pub destination: DroneId,
    /// Route status
    pub status: RouteStatus,
    /// Current quality
    pub current_quality: f32,
    /// Predicted quality
    pub predicted_quality: f32,
    /// Time to failure (ms)
    pub time_to_failure_ms: Option<u32>,
    /// Whether backup is available
    pub has_backup: bool,
}

impl RouteAlert {
    /// Get recommended action
    pub fn recommended_action(&self) -> RouteAction {
        match self.status {
            RouteStatus::Broken => {
                if self.has_backup {
                    RouteAction::SwitchToBackup
                } else {
                    RouteAction::DiscoverRoute
                }
            }
            RouteStatus::AtRisk => {
                if self.has_backup {
                    RouteAction::SwitchToBackup
                } else {
                    RouteAction::PrepareBackup
                }
            }
            RouteStatus::Degrading => RouteAction::PrepareBackup,
            RouteStatus::Healthy => RouteAction::None,
        }
    }
}

/// Recommended action for a route
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RouteAction {
    /// No action needed
    None,
    /// Prepare backup routes
    PrepareBackup,
    /// Switch to backup route
    SwitchToBackup,
    /// Discover new route
    DiscoverRoute,
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_predictive_route_score() {
        let route1 = PredictiveRoute::direct(DroneId::new(1), 0.9, 1000);
        let route2 = PredictiveRoute::direct(DroneId::new(2), 0.8, 1000);

        assert!(route1.score() > route2.score());
    }

    #[test]
    fn test_proactive_manager_basic() {
        let mut manager = ProactiveRouteManager::new_default(DroneId::new(0));

        // Add a neighbor
        manager
            .update_neighbor(
                DroneId::new(1),
                Position { x: 10.0, y: 0.0, z: 0.0 },
                None,
                -60,
                0.9,
                1000,
            )
            .unwrap();

        assert_eq!(manager.neighbor_count(), 1);
        assert_eq!(manager.route_count(), 1);

        // Get route
        let route = manager.get_best_route(DroneId::new(1));
        assert!(route.is_some());
        assert!(route.unwrap().quality > 0.8);
    }

    #[test]
    fn test_degrading_link_detection() {
        let mut manager = ProactiveRouteManager::new_default(DroneId::new(0));

        // Simulate degrading link
        for i in 0..10 {
            let quality = 0.9 - (i as f32 * 0.08);
            manager
                .update_neighbor(
                    DroneId::new(1),
                    Position { x: 10.0, y: 0.0, z: 0.0 },
                    None,
                    -60 - i as i8,
                    quality,
                    (i * 1000) as u64,
                )
                .unwrap();
        }

        // Scan should detect degrading route
        let alerts = manager.scan_routes(10000).unwrap();

        // Should have alerts if quality dropped enough
        if manager.get_best_route(DroneId::new(1)).map(|r| r.quality).unwrap_or(0.0) < 0.4 {
            assert!(!alerts.is_empty() || manager.routes_at_risk().is_empty() == false);
        }
    }

    #[test]
    fn test_backup_routes() {
        let mut manager = ProactiveRouteManager::new_default(DroneId::new(0));

        // Add primary route
        let mut path1 = Vec::new();
        path1.push(DroneId::new(1)).ok();
        manager.add_route(DroneId::new(10), path1, 0.9, 1000).unwrap();

        // Add backup route
        let mut path2 = Vec::new();
        path2.push(DroneId::new(2)).ok();
        path2.push(DroneId::new(10)).ok();
        manager.add_route(DroneId::new(10), path2, 0.7, 1000).unwrap();

        // Primary should be preferred
        let route = manager.get_best_route(DroneId::new(10)).unwrap();
        assert_eq!(route.hop_count(), 1);
    }

    #[test]
    fn test_route_switch() {
        let mut manager = ProactiveRouteManager::new_default(DroneId::new(0));

        // Add primary with poor quality
        let mut path1 = Vec::new();
        path1.push(DroneId::new(1)).ok();
        manager.add_route(DroneId::new(10), path1, 0.1, 1000).unwrap();

        // Add good backup
        let mut path2 = Vec::new();
        path2.push(DroneId::new(2)).ok();
        path2.push(DroneId::new(10)).ok();
        manager.add_route(DroneId::new(10), path2, 0.9, 1000).unwrap();

        // Switch should succeed
        let switched = manager.switch_to_backup(DroneId::new(10)).unwrap();
        assert!(switched);

        // New primary should have good quality
        let route = manager.get_best_route(DroneId::new(10)).unwrap();
        assert!(route.quality > 0.5);
    }

    #[test]
    fn test_route_alert_actions() {
        let alert_broken = RouteAlert {
            destination: DroneId::new(1),
            status: RouteStatus::Broken,
            current_quality: 0.1,
            predicted_quality: 0.0,
            time_to_failure_ms: Some(0),
            has_backup: true,
        };

        assert_eq!(alert_broken.recommended_action(), RouteAction::SwitchToBackup);

        let alert_degrading = RouteAlert {
            destination: DroneId::new(1),
            status: RouteStatus::Degrading,
            current_quality: 0.5,
            predicted_quality: 0.3,
            time_to_failure_ms: Some(5000),
            has_backup: false,
        };

        assert_eq!(alert_degrading.recommended_action(), RouteAction::PrepareBackup);
    }

    #[test]
    fn test_neighbor_pruning() {
        let mut manager = ProactiveRouteManager::new_default(DroneId::new(0));

        // Add neighbor
        manager
            .update_neighbor(
                DroneId::new(1),
                Position { x: 10.0, y: 0.0, z: 0.0 },
                None,
                -60,
                0.9,
                1000,
            )
            .unwrap();

        assert_eq!(manager.neighbor_count(), 1);

        // Prune with short timeout
        manager.prune_stale(500, 2000);
        assert_eq!(manager.neighbor_count(), 0);
    }
}
