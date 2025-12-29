//! Cluster Manager for Hierarchical Consensus
//!
//! Manages dynamic clustering of drones based on geographic proximity.
//! Supports cluster formation, leader election, membership tracking,
//! and dynamic rebalancing.

use crate::hierarchy_types::{
    ClusterId, ClusterMembership, HierarchyConfig, HierarchyTier, RegionId, MAX_CLUSTER_SIZE,
};
use crate::types::{DroneId, Position, Result, SwarmError};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum number of clusters
pub const MAX_CLUSTERS: usize = 128;

/// Maximum number of regions (must be power of 2 for heapless)
pub const MAX_REGIONS: usize = 64;

/// Minimum cluster size before triggering merge
pub const MIN_CLUSTER_SIZE: usize = 3;

/// Maximum cluster size before triggering split
pub const MAX_OPTIMAL_CLUSTER_SIZE: usize = 8;

// ═══════════════════════════════════════════════════════════════════════════
// CLUSTER INFO
// ═══════════════════════════════════════════════════════════════════════════

/// Extended cluster information for manager tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClusterInfo {
    /// Core membership info
    pub membership: ClusterMembership,
    /// Creation timestamp
    pub created_at_ms: u64,
    /// Last update timestamp
    pub last_updated_ms: u64,
    /// Is this cluster stable (not in rebalancing)
    pub is_stable: bool,
}

impl ClusterInfo {
    /// Create a new cluster
    pub fn new(cluster_id: ClusterId, region_id: RegionId, centroid: Position, timestamp_ms: u64) -> Self {
        let mut membership = ClusterMembership::new(cluster_id, region_id);
        membership.center = centroid;
        membership.formed_at = timestamp_ms;

        Self {
            membership,
            created_at_ms: timestamp_ms,
            last_updated_ms: timestamp_ms,
            is_stable: false,
        }
    }

    /// Get cluster ID
    pub fn id(&self) -> ClusterId {
        self.membership.cluster_id
    }

    /// Get region ID
    pub fn region_id(&self) -> RegionId {
        self.membership.region_id
    }

    /// Get leader
    pub fn leader(&self) -> Option<DroneId> {
        self.membership.leader
    }

    /// Get centroid
    pub fn centroid(&self) -> Position {
        self.membership.center
    }

    /// Get members
    pub fn members(&self) -> &[DroneId] {
        &self.membership.members
    }

    /// Add a member
    pub fn add_member(&mut self, drone_id: DroneId) -> Result<()> {
        self.membership.add_member(drone_id)?;
        self.is_stable = false;
        Ok(())
    }

    /// Remove a member
    pub fn remove_member(&mut self, drone_id: DroneId) -> Result<()> {
        self.membership.remove_member(drone_id)?;
        self.is_stable = false;
        Ok(())
    }

    /// Set leader
    pub fn set_leader(&mut self, leader_id: DroneId) {
        self.membership.leader = Some(leader_id);
    }

    /// Get member count
    pub fn size(&self) -> usize {
        self.membership.members.len()
    }

    /// Check if cluster is full
    pub fn is_full(&self) -> bool {
        self.membership.members.len() >= MAX_CLUSTER_SIZE
    }

    /// Check if cluster needs splitting
    pub fn needs_split(&self) -> bool {
        self.membership.members.len() > MAX_OPTIMAL_CLUSTER_SIZE
    }

    /// Check if cluster needs merging
    pub fn needs_merge(&self) -> bool {
        self.membership.members.len() < MIN_CLUSTER_SIZE
    }

    /// Calculate distance from position to cluster centroid
    pub fn distance_to(&self, position: &Position) -> f32 {
        let centroid = &self.membership.center;
        let dx = position.x - centroid.x;
        let dy = position.y - centroid.y;
        let dz = position.z - centroid.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Mark cluster as stable
    pub fn mark_stable(&mut self, timestamp_ms: u64) {
        self.is_stable = true;
        self.last_updated_ms = timestamp_ms;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// REGION INFO
// ═══════════════════════════════════════════════════════════════════════════

/// Information about a region (group of clusters)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegionInfo {
    /// Region ID
    pub id: RegionId,
    /// Clusters in this region
    pub clusters: Vec<ClusterId, 16>,
    /// Regional leader (cluster leader elected to represent region)
    pub regional_leader: Option<DroneId>,
    /// Region centroid
    pub centroid: Position,
    /// Total drones in region
    pub total_drones: usize,
}

impl RegionInfo {
    /// Create a new region
    pub fn new(id: RegionId, centroid: Position) -> Self {
        Self {
            id,
            clusters: Vec::new(),
            regional_leader: None,
            centroid,
            total_drones: 0,
        }
    }

    /// Add a cluster
    pub fn add_cluster(&mut self, cluster_id: ClusterId) -> Result<()> {
        if !self.clusters.contains(&cluster_id) {
            self.clusters
                .push(cluster_id)
                .map_err(|_| SwarmError::BufferFull)?;
        }
        Ok(())
    }

    /// Remove a cluster
    pub fn remove_cluster(&mut self, cluster_id: ClusterId) {
        if let Some(pos) = self.clusters.iter().position(|&id| id == cluster_id) {
            self.clusters.swap_remove(pos);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LOCAL DRONE INFO
// ═══════════════════════════════════════════════════════════════════════════

/// Role of this drone in the hierarchy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LocalRole {
    /// Not assigned to any cluster
    Unassigned,
    /// Follower in a cluster
    Follower,
    /// Leader of a cluster
    Leader,
    /// Regional leader (leads inter-cluster coordination)
    RegionalLeader,
}

/// Local drone's hierarchy membership info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalMembership {
    /// Our drone ID
    pub drone_id: DroneId,
    /// Cluster we belong to
    pub cluster_id: ClusterId,
    /// Region we belong to
    pub region_id: RegionId,
    /// Our role
    pub role: LocalRole,
    /// When we joined
    pub joined_at_ms: u64,
}

// ═══════════════════════════════════════════════════════════════════════════
// CLUSTER MANAGER
// ═══════════════════════════════════════════════════════════════════════════

/// Manages cluster membership and operations
#[derive(Debug, Clone)]
pub struct ClusterManager {
    /// Our drone ID
    local_id: DroneId,
    /// Our membership info
    local_membership: Option<LocalMembership>,
    /// All known clusters
    clusters: FnvIndexMap<u32, ClusterInfo, MAX_CLUSTERS>,
    /// All known regions
    regions: FnvIndexMap<u16, RegionInfo, MAX_REGIONS>,
    /// Drone position cache
    drone_positions: FnvIndexMap<u64, Position, 256>,
    /// Configuration
    config: HierarchyConfig,
    /// Next cluster ID to assign
    next_cluster_id: u32,
    /// Next region ID to assign (reserved for future use)
    #[allow(dead_code)]
    next_region_id: u16,
    /// Current timestamp
    current_time_ms: u64,
}

impl ClusterManager {
    /// Create a new cluster manager
    pub fn new(local_id: DroneId, config: HierarchyConfig) -> Self {
        Self {
            local_id,
            local_membership: None,
            clusters: FnvIndexMap::new(),
            regions: FnvIndexMap::new(),
            drone_positions: FnvIndexMap::new(),
            config,
            next_cluster_id: 1,
            next_region_id: 1,
            current_time_ms: 0,
        }
    }

    /// Create with default config
    pub fn new_default(local_id: DroneId) -> Self {
        Self::new(local_id, HierarchyConfig::default())
    }

    /// Get local drone ID
    pub fn local_id(&self) -> DroneId {
        self.local_id
    }

    /// Get our cluster ID
    pub fn cluster_id(&self) -> Option<ClusterId> {
        self.local_membership.as_ref().map(|m| m.cluster_id)
    }

    /// Get our region ID
    pub fn region_id(&self) -> Option<RegionId> {
        self.local_membership.as_ref().map(|m| m.region_id)
    }

    /// Get our role
    pub fn role(&self) -> LocalRole {
        self.local_membership
            .as_ref()
            .map(|m| m.role)
            .unwrap_or(LocalRole::Unassigned)
    }

    /// Check if we are a cluster leader
    pub fn is_cluster_leader(&self) -> bool {
        matches!(self.role(), LocalRole::Leader | LocalRole::RegionalLeader)
    }

    /// Check if we are a regional leader
    pub fn is_regional_leader(&self) -> bool {
        self.role() == LocalRole::RegionalLeader
    }

    /// Update drone position
    pub fn update_position(&mut self, drone_id: DroneId, position: Position) -> Result<()> {
        if self.drone_positions.contains_key(&drone_id.as_u64()) {
            if let Some(p) = self.drone_positions.get_mut(&drone_id.as_u64()) {
                *p = position;
            }
        } else {
            self.drone_positions
                .insert(drone_id.as_u64(), position)
                .map_err(|_| SwarmError::BufferFull)?;
        }
        Ok(())
    }

    /// Update current time
    pub fn update_time(&mut self, timestamp_ms: u64) {
        self.current_time_ms = timestamp_ms;
    }

    /// Request to join the hierarchy
    pub fn join_hierarchy(&mut self, position: Position) -> Result<JoinRequest> {
        // Update our position
        self.update_position(self.local_id, position)?;

        // Find nearest cluster or create new one
        let cluster = self.find_nearest_cluster(&position);

        match cluster {
            Some(cluster_id) => Ok(JoinRequest::JoinExisting { cluster_id }),
            None => Ok(JoinRequest::CreateNew { position }),
        }
    }

    /// Find nearest cluster that can accept new member
    fn find_nearest_cluster(&self, position: &Position) -> Option<ClusterId> {
        let mut best: Option<(ClusterId, f32)> = None;
        let threshold = self.config.cluster_radius_m;

        for (_, cluster) in self.clusters.iter() {
            if cluster.is_full() {
                continue;
            }

            let dist = cluster.distance_to(position);
            if dist > threshold {
                continue;
            }

            match best {
                None => best = Some((cluster.id(), dist)),
                Some((_, best_dist)) if dist < best_dist => {
                    best = Some((cluster.id(), dist));
                }
                _ => {}
            }
        }

        best.map(|(id, _)| id)
    }

    /// Process join response
    pub fn process_join_response(
        &mut self,
        accepted: bool,
        cluster_id: Option<ClusterId>,
        region_id: Option<RegionId>,
    ) -> Result<()> {
        if accepted {
            if let (Some(cid), Some(rid)) = (cluster_id, region_id) {
                self.local_membership = Some(LocalMembership {
                    drone_id: self.local_id,
                    cluster_id: cid,
                    region_id: rid,
                    role: LocalRole::Follower,
                    joined_at_ms: self.current_time_ms,
                });
            }
        }
        Ok(())
    }

    /// Create a new cluster
    pub fn create_cluster(&mut self, centroid: Position) -> Result<ClusterId> {
        let cluster_id = ClusterId::new(self.next_cluster_id);
        self.next_cluster_id += 1;

        // Find or create region
        let region_id = self.find_or_create_region(&centroid)?;

        let cluster = ClusterInfo::new(cluster_id, region_id, centroid, self.current_time_ms);

        self.clusters
            .insert(cluster_id.as_u32(), cluster)
            .map_err(|_| SwarmError::BufferFull)?;

        // Add to region
        if let Some(region) = self.regions.get_mut(&region_id.as_u16()) {
            region.add_cluster(cluster_id)?;
        }

        Ok(cluster_id)
    }

    /// Find or create region for position
    fn find_or_create_region(&mut self, position: &Position) -> Result<RegionId> {
        // Simple region assignment based on grid
        let region_size = self.config.cluster_radius_m * 10.0; // Regions are 10x cluster radius
        let grid_x = (position.x / region_size) as i32;
        let grid_y = (position.y / region_size) as i32;

        // Create deterministic region ID from grid position
        let region_id = ((grid_x.unsigned_abs() as u16) << 8) | (grid_y.unsigned_abs() as u16 & 0xFF);
        let region_id = RegionId::new(region_id);

        if !self.regions.contains_key(&region_id.as_u16()) {
            let region = RegionInfo::new(
                region_id,
                Position {
                    x: (grid_x as f32 + 0.5) * region_size,
                    y: (grid_y as f32 + 0.5) * region_size,
                    z: position.z,
                },
            );
            self.regions
                .insert(region_id.as_u16(), region)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(region_id)
    }

    /// Add drone to cluster
    pub fn add_to_cluster(&mut self, drone_id: DroneId, cluster_id: ClusterId) -> Result<()> {
        if let Some(cluster) = self.clusters.get_mut(&cluster_id.as_u32()) {
            cluster.add_member(drone_id)?;

            // Update region drone count
            let region_id = cluster.region_id();
            if let Some(region) = self.regions.get_mut(&region_id.as_u16()) {
                region.total_drones += 1;
            }
        }
        Ok(())
    }

    /// Remove drone from cluster
    pub fn remove_from_cluster(&mut self, drone_id: DroneId, cluster_id: ClusterId) {
        if let Some(cluster) = self.clusters.get_mut(&cluster_id.as_u32()) {
            if cluster.remove_member(drone_id).is_ok() {
                // Update region drone count
                let region_id = cluster.region_id();
                if let Some(region) = self.regions.get_mut(&region_id.as_u16()) {
                    region.total_drones = region.total_drones.saturating_sub(1);
                }
            }
        }
    }

    /// Set cluster leader
    pub fn set_cluster_leader(&mut self, cluster_id: ClusterId, leader_id: DroneId) {
        if let Some(cluster) = self.clusters.get_mut(&cluster_id.as_u32()) {
            cluster.set_leader(leader_id);
        }

        // Update our role if we're the leader
        if leader_id == self.local_id {
            if let Some(ref mut membership) = self.local_membership {
                membership.role = LocalRole::Leader;
            }
        }
    }

    /// Set regional leader
    pub fn set_regional_leader(&mut self, region_id: RegionId, leader_id: DroneId) {
        if let Some(region) = self.regions.get_mut(&region_id.as_u16()) {
            region.regional_leader = Some(leader_id);
        }

        // Update our role if we're the leader
        if leader_id == self.local_id {
            if let Some(ref mut membership) = self.local_membership {
                membership.role = LocalRole::RegionalLeader;
            }
        }
    }

    /// Get cluster info
    pub fn get_cluster(&self, cluster_id: ClusterId) -> Option<&ClusterInfo> {
        self.clusters.get(&cluster_id.as_u32())
    }

    /// Get region info
    pub fn get_region(&self, region_id: RegionId) -> Option<&RegionInfo> {
        self.regions.get(&region_id.as_u16())
    }

    /// Get cluster members
    pub fn get_cluster_members(&self, cluster_id: ClusterId) -> Option<&[DroneId]> {
        self.clusters
            .get(&cluster_id.as_u32())
            .map(|c| c.members())
    }

    /// Get cluster leader
    pub fn get_cluster_leader(&self, cluster_id: ClusterId) -> Option<DroneId> {
        self.clusters
            .get(&cluster_id.as_u32())
            .and_then(|c| c.leader())
    }

    /// Get all cluster IDs in a region
    pub fn get_region_clusters(&self, region_id: RegionId) -> Option<&[ClusterId]> {
        self.regions
            .get(&region_id.as_u16())
            .map(|r| r.clusters.as_slice())
    }

    /// Get cluster count
    pub fn cluster_count(&self) -> usize {
        self.clusters.len()
    }

    /// Get region count
    pub fn region_count(&self) -> usize {
        self.regions.len()
    }

    /// Check for clusters needing rebalancing
    pub fn check_rebalancing(&self) -> Vec<RebalanceAction, 16> {
        let mut actions = Vec::new();

        for (_, cluster) in self.clusters.iter() {
            if cluster.needs_split() {
                actions
                    .push(RebalanceAction::Split {
                        cluster_id: cluster.id(),
                    })
                    .ok();
            } else if cluster.needs_merge() {
                // Find nearby cluster to merge with
                if let Some(target) = self.find_merge_target(cluster) {
                    actions
                        .push(RebalanceAction::Merge {
                            source: cluster.id(),
                            target,
                        })
                        .ok();
                }
            }
        }

        actions
    }

    /// Find a cluster to merge with
    fn find_merge_target(&self, source: &ClusterInfo) -> Option<ClusterId> {
        let mut best: Option<(ClusterId, f32)> = None;

        for (_, cluster) in self.clusters.iter() {
            if cluster.id() == source.id() {
                continue;
            }
            if cluster.region_id() != source.region_id() {
                continue;
            }
            // Don't merge if combined size would be too large
            if cluster.size() + source.size() > MAX_OPTIMAL_CLUSTER_SIZE {
                continue;
            }

            let dist = source.distance_to(&cluster.centroid());

            match best {
                None => best = Some((cluster.id(), dist)),
                Some((_, best_dist)) if dist < best_dist => {
                    best = Some((cluster.id(), dist));
                }
                _ => {}
            }
        }

        best.map(|(id, _)| id)
    }

    /// Get configuration
    pub fn config(&self) -> &HierarchyConfig {
        &self.config
    }

    /// Get hierarchy tier for current swarm size
    pub fn current_tier(&self) -> HierarchyTier {
        let total_drones: usize = self.regions.values().map(|r| r.total_drones).sum();

        // Tier thresholds based on cluster/region structure
        if total_drones <= 50 {
            HierarchyTier::Local
        } else if total_drones <= 500 {
            HierarchyTier::Regional
        } else {
            HierarchyTier::Global
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// JOIN REQUEST
// ═══════════════════════════════════════════════════════════════════════════

/// Request to join hierarchy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JoinRequest {
    /// Join existing cluster
    JoinExisting { cluster_id: ClusterId },
    /// Create new cluster at position
    CreateNew { position: Position },
}

// ═══════════════════════════════════════════════════════════════════════════
// REBALANCE ACTION
// ═══════════════════════════════════════════════════════════════════════════

/// Cluster rebalancing action
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum RebalanceAction {
    /// Split cluster into two
    Split { cluster_id: ClusterId },
    /// Merge source into target
    Merge { source: ClusterId, target: ClusterId },
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cluster_manager_creation() {
        let manager = ClusterManager::new_default(DroneId::new(1));
        assert_eq!(manager.local_id(), DroneId::new(1));
        assert!(manager.cluster_id().is_none());
    }

    #[test]
    fn test_cluster_creation() {
        let mut manager = ClusterManager::new_default(DroneId::new(1));
        manager.update_time(1000);

        let pos = Position {
            x: 50.0,
            y: 50.0,
            z: 10.0,
        };
        let cluster_id = manager.create_cluster(pos).unwrap();

        assert_eq!(manager.cluster_count(), 1);
        assert!(manager.get_cluster(cluster_id).is_some());
    }

    #[test]
    fn test_add_member_to_cluster() {
        let mut manager = ClusterManager::new_default(DroneId::new(0));
        manager.update_time(1000);

        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let cluster_id = manager.create_cluster(pos).unwrap();

        // Add members
        for i in 1..=5 {
            manager.add_to_cluster(DroneId::new(i), cluster_id).unwrap();
        }

        let members = manager.get_cluster_members(cluster_id).unwrap();
        assert_eq!(members.len(), 5);
    }

    #[test]
    fn test_cluster_leader_election() {
        let mut manager = ClusterManager::new_default(DroneId::new(1));
        manager.update_time(1000);

        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let cluster_id = manager.create_cluster(pos).unwrap();

        manager.add_to_cluster(DroneId::new(1), cluster_id).unwrap();
        manager.add_to_cluster(DroneId::new(2), cluster_id).unwrap();

        // Set leader
        manager.set_cluster_leader(cluster_id, DroneId::new(1));

        assert_eq!(manager.get_cluster_leader(cluster_id), Some(DroneId::new(1)));
    }
}
