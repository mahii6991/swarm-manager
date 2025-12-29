//! Hierarchical Consensus Type Definitions
//!
//! Core types for the 3-tier hierarchical consensus architecture:
//! - Tier 1 (Local): Raft consensus within 5-8 drone clusters
//! - Tier 2 (Regional): PBFT across cluster leaders
//! - Tier 3 (Global): Vote aggregation for swarm-wide decisions
//!
//! Designed for 500-5000+ drone swarms with Byzantine fault tolerance.

use crate::types::{DroneId, Position, Result, SwarmError};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CLUSTER IDENTIFICATION
// ═══════════════════════════════════════════════════════════════════════════

/// Unique identifier for a cluster of drones
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ClusterId(pub u32);

impl ClusterId {
    /// Create a new cluster ID
    pub const fn new(id: u32) -> Self {
        Self(id)
    }

    /// Get the inner value
    pub const fn as_u32(&self) -> u32 {
        self.0
    }
}

impl core::fmt::Display for ClusterId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Cluster-{}", self.0)
    }
}

/// Unique identifier for a region (group of clusters)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RegionId(pub u16);

impl RegionId {
    /// Create a new region ID
    pub const fn new(id: u16) -> Self {
        Self(id)
    }

    /// Get the inner value
    pub const fn as_u16(&self) -> u16 {
        self.0
    }
}

impl core::fmt::Display for RegionId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Region-{}", self.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// HIERARCHY TIERS
// ═══════════════════════════════════════════════════════════════════════════

/// Hierarchy tier levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum HierarchyTier {
    /// Tier 1: Local cluster (5-8 drones, Raft consensus)
    Local = 1,
    /// Tier 2: Regional (3-10 cluster leaders, PBFT consensus)
    Regional = 2,
    /// Tier 3: Global (all regional leaders, vote aggregation)
    Global = 3,
}

impl HierarchyTier {
    /// Get the typical consensus latency for this tier
    pub const fn typical_latency_ms(&self) -> u32 {
        match self {
            HierarchyTier::Local => 10,     // Fast Raft
            HierarchyTier::Regional => 50,  // PBFT with f=1
            HierarchyTier::Global => 100,   // Vote aggregation
        }
    }

    /// Get the maximum nodes at this tier
    pub const fn max_nodes(&self) -> usize {
        match self {
            HierarchyTier::Local => 8,      // 5-8 drones per cluster
            HierarchyTier::Regional => 10,  // 3-10 clusters per region
            HierarchyTier::Global => 50,    // Up to 50 regions
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CLUSTER MEMBERSHIP
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum drones per cluster (Raft works best with 5-8 nodes)
pub const MAX_CLUSTER_SIZE: usize = 8;

/// Maximum clusters per region
pub const MAX_CLUSTERS_PER_REGION: usize = 10;

/// Maximum regions in the swarm
pub const MAX_REGIONS: usize = 50;

/// Cluster membership information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClusterMembership {
    /// Cluster identifier
    pub cluster_id: ClusterId,
    /// Region this cluster belongs to
    pub region_id: RegionId,
    /// Member drones in this cluster
    pub members: Vec<DroneId, MAX_CLUSTER_SIZE>,
    /// Current cluster leader (elected via Raft)
    pub leader: Option<DroneId>,
    /// Cluster center position (geometric mean)
    pub center: Position,
    /// Cluster formation timestamp
    pub formed_at: u64,
}

impl ClusterMembership {
    /// Create a new empty cluster
    pub fn new(cluster_id: ClusterId, region_id: RegionId) -> Self {
        Self {
            cluster_id,
            region_id,
            members: Vec::new(),
            leader: None,
            center: Position { x: 0.0, y: 0.0, z: 0.0 },
            formed_at: 0,
        }
    }

    /// Add a drone to the cluster
    pub fn add_member(&mut self, drone_id: DroneId) -> Result<()> {
        if self.members.len() >= MAX_CLUSTER_SIZE {
            return Err(SwarmError::SwarmSizeExceeded);
        }
        if self.members.contains(&drone_id) {
            return Err(SwarmError::InvalidParameter);
        }
        self.members.push(drone_id).map_err(|_| SwarmError::BufferFull)
    }

    /// Remove a drone from the cluster
    pub fn remove_member(&mut self, drone_id: DroneId) -> Result<()> {
        if let Some(pos) = self.members.iter().position(|&id| id == drone_id) {
            self.members.swap_remove(pos);
            // Clear leader if removed
            if self.leader == Some(drone_id) {
                self.leader = None;
            }
            Ok(())
        } else {
            Err(SwarmError::InvalidDroneId)
        }
    }

    /// Check if cluster has quorum for Raft
    pub fn has_quorum(&self) -> bool {
        let majority = (self.members.len() / 2) + 1;
        self.members.len() >= majority && self.members.len() >= 3
    }

    /// Update cluster center from member positions
    pub fn update_center(&mut self, positions: &FnvIndexMap<u64, Position, MAX_CLUSTER_SIZE>) {
        if self.members.is_empty() {
            return;
        }

        let mut sum_x = 0.0f32;
        let mut sum_y = 0.0f32;
        let mut sum_z = 0.0f32;
        let mut count = 0;

        for member in &self.members {
            if let Some(pos) = positions.get(&member.as_u64()) {
                sum_x += pos.x;
                sum_y += pos.y;
                sum_z += pos.z;
                count += 1;
            }
        }

        if count > 0 {
            self.center = Position {
                x: sum_x / count as f32,
                y: sum_y / count as f32,
                z: sum_z / count as f32,
            };
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// HIERARCHICAL MESSAGES
// ═══════════════════════════════════════════════════════════════════════════

/// Messages for hierarchical consensus communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HierarchicalMessage {
    // ─── Tier 1: Local Raft Messages ───
    /// Raft vote request within cluster
    LocalVoteRequest {
        cluster_id: ClusterId,
        term: u64,
        candidate_id: DroneId,
        last_log_index: u64,
        last_log_term: u64,
    },
    /// Raft vote response
    LocalVoteResponse {
        cluster_id: ClusterId,
        term: u64,
        vote_granted: bool,
        voter_id: DroneId,
    },
    /// Raft append entries (heartbeat/replication)
    LocalAppendEntries {
        cluster_id: ClusterId,
        term: u64,
        leader_id: DroneId,
        prev_log_index: u64,
        prev_log_term: u64,
        entries: Vec<HierarchyLogEntry, 32>,
        leader_commit: u64,
    },
    /// Raft append entries response
    LocalAppendResponse {
        cluster_id: ClusterId,
        term: u64,
        success: bool,
        match_index: u64,
        follower_id: DroneId,
    },

    // ─── Tier 2: Regional PBFT Messages ───
    /// PBFT pre-prepare (from primary)
    RegionalPrePrepare {
        region_id: RegionId,
        view: u64,
        sequence: u64,
        proposal: HierarchyProposal,
        primary_id: DroneId,
    },
    /// PBFT prepare (broadcast)
    RegionalPrepare {
        region_id: RegionId,
        view: u64,
        sequence: u64,
        proposal_hash: [u8; 32],
        sender_id: DroneId,
    },
    /// PBFT commit (broadcast)
    RegionalCommit {
        region_id: RegionId,
        view: u64,
        sequence: u64,
        proposal_hash: [u8; 32],
        sender_id: DroneId,
    },
    /// PBFT view change request
    RegionalViewChange {
        region_id: RegionId,
        new_view: u64,
        sender_id: DroneId,
    },

    // ─── Tier 3: Global Aggregation Messages ───
    /// Regional decision announcement
    GlobalDecision {
        region_id: RegionId,
        decision: HierarchyDecision,
        leader_id: DroneId,
        /// Ed25519 signature (64 bytes) as heapless Vec for serde compatibility
        signature: heapless::Vec<u8, 64>,
    },
    /// Global vote request
    GlobalVoteRequest {
        proposal: HierarchyProposal,
        initiator: DroneId,
    },
    /// Global vote response
    GlobalVoteResponse {
        proposal_hash: [u8; 32],
        approve: bool,
        region_id: RegionId,
        voter_id: DroneId,
    },

    // ─── Cluster Management ───
    /// Request to join a cluster
    JoinClusterRequest {
        drone_id: DroneId,
        position: Position,
    },
    /// Response to join request
    JoinClusterResponse {
        accepted: bool,
        cluster_id: Option<ClusterId>,
        leader_id: Option<DroneId>,
    },
    /// Cluster rebalancing notification
    ClusterRebalance {
        old_cluster: ClusterId,
        new_cluster: ClusterId,
        affected_drones: Vec<DroneId, MAX_CLUSTER_SIZE>,
    },
}

/// Log entry for hierarchical consensus
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HierarchyLogEntry {
    /// Term when entry was created
    pub term: u64,
    /// Entry index
    pub index: u64,
    /// The command to execute
    pub command: HierarchyCommand,
}

/// Commands that can be proposed through hierarchy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HierarchyCommand {
    /// Formation change (local cluster)
    LocalFormationChange { formation_type: u8 },
    /// Task assignment within cluster
    LocalTaskAssign { drone_id: DroneId, task_id: u64 },
    /// Emergency stop (local)
    LocalEmergencyStop,
    /// Regional formation update
    RegionalFormationSync { formation_hash: [u8; 32] },
    /// Cross-cluster task reallocation
    RegionalTaskRealloc { from_cluster: ClusterId, to_cluster: ClusterId, task_id: u64 },
    /// Global mission parameter update
    GlobalMissionUpdate { params: Vec<u8, 256> },
    /// Global emergency broadcast
    GlobalEmergency { reason: u8 },
}

/// Proposal for PBFT consensus
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HierarchyProposal {
    /// Proposal ID
    pub id: u64,
    /// The command being proposed
    pub command: HierarchyCommand,
    /// Proposer drone ID
    pub proposer: DroneId,
    /// Timestamp
    pub timestamp: u64,
}

impl HierarchyProposal {
    /// Compute hash of this proposal for verification
    pub fn hash(&self) -> [u8; 32] {
        use crate::crypto::CryptoContext;

        let mut data = Vec::<u8, 512>::new();
        data.extend_from_slice(&self.id.to_le_bytes()).ok();
        data.extend_from_slice(&self.proposer.as_u64().to_le_bytes()).ok();
        data.extend_from_slice(&self.timestamp.to_le_bytes()).ok();

        CryptoContext::secure_hash(&data)
    }
}

/// Decision from hierarchical consensus
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HierarchyDecision {
    /// The accepted proposal
    pub proposal: HierarchyProposal,
    /// Consensus sequence number
    pub sequence: u64,
    /// Number of votes received
    pub vote_count: u32,
    /// Decision timestamp
    pub decided_at: u64,
}

// ═══════════════════════════════════════════════════════════════════════════
// PBFT STATE
// ═══════════════════════════════════════════════════════════════════════════

/// PBFT node state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PBFTState {
    /// Idle, waiting for proposals
    Idle,
    /// Pre-prepare received, waiting for prepares
    PrePrepared,
    /// Enough prepares received, waiting for commits
    Prepared,
    /// Enough commits received, executing
    Committed,
}

/// PBFT round state
#[derive(Debug, Clone)]
pub struct PBFTRound {
    /// Current view number
    pub view: u64,
    /// Sequence number
    pub sequence: u64,
    /// Current state
    pub state: PBFTState,
    /// The proposal being voted on
    pub proposal: Option<HierarchyProposal>,
    /// Prepare votes received
    pub prepares: Vec<DroneId, MAX_CLUSTERS_PER_REGION>,
    /// Commit votes received
    pub commits: Vec<DroneId, MAX_CLUSTERS_PER_REGION>,
    /// Round start time
    pub started_at: u64,
}

impl PBFTRound {
    /// Create a new PBFT round
    pub fn new(view: u64, sequence: u64) -> Self {
        Self {
            view,
            sequence,
            state: PBFTState::Idle,
            proposal: None,
            prepares: Vec::new(),
            commits: Vec::new(),
            started_at: 0,
        }
    }

    /// Check if we have enough prepares for quorum (2f+1)
    pub fn has_prepare_quorum(&self, total_nodes: usize) -> bool {
        let f = (total_nodes - 1) / 3; // Byzantine tolerance
        let quorum = 2 * f + 1;
        self.prepares.len() >= quorum
    }

    /// Check if we have enough commits for quorum (2f+1)
    pub fn has_commit_quorum(&self, total_nodes: usize) -> bool {
        let f = (total_nodes - 1) / 3;
        let quorum = 2 * f + 1;
        self.commits.len() >= quorum
    }

    /// Add a prepare vote
    pub fn add_prepare(&mut self, voter: DroneId) -> Result<()> {
        if !self.prepares.contains(&voter) {
            self.prepares.push(voter).map_err(|_| SwarmError::BufferFull)?;
        }
        Ok(())
    }

    /// Add a commit vote
    pub fn add_commit(&mut self, voter: DroneId) -> Result<()> {
        if !self.commits.contains(&voter) {
            self.commits.push(voter).map_err(|_| SwarmError::BufferFull)?;
        }
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

/// Configuration for hierarchical consensus
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HierarchyConfig {
    /// This node's tier
    pub tier: HierarchyTier,
    /// Target cluster size (5-8)
    pub target_cluster_size: usize,
    /// Maximum distance for cluster membership (meters)
    pub cluster_radius_m: f32,
    /// Raft election timeout (ms)
    pub raft_election_timeout_ms: u32,
    /// Raft heartbeat interval (ms)
    pub raft_heartbeat_ms: u32,
    /// PBFT round timeout (ms)
    pub pbft_timeout_ms: u32,
    /// Enable dynamic cluster rebalancing
    pub dynamic_rebalancing: bool,
}

impl Default for HierarchyConfig {
    fn default() -> Self {
        Self {
            tier: HierarchyTier::Local,
            target_cluster_size: 6,
            cluster_radius_m: 100.0,
            raft_election_timeout_ms: 150,
            raft_heartbeat_ms: 50,
            pbft_timeout_ms: 500,
            dynamic_rebalancing: true,
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
    fn test_cluster_membership() {
        let mut cluster = ClusterMembership::new(ClusterId::new(1), RegionId::new(0));

        // Add members
        assert!(cluster.add_member(DroneId::new(1)).is_ok());
        assert!(cluster.add_member(DroneId::new(2)).is_ok());
        assert!(cluster.add_member(DroneId::new(3)).is_ok());

        // Check quorum (3 nodes = quorum)
        assert!(cluster.has_quorum());

        // Cannot add duplicate
        assert!(cluster.add_member(DroneId::new(1)).is_err());

        // Remove member
        assert!(cluster.remove_member(DroneId::new(2)).is_ok());
        assert_eq!(cluster.members.len(), 2);
    }

    #[test]
    fn test_cluster_size_limit() {
        let mut cluster = ClusterMembership::new(ClusterId::new(1), RegionId::new(0));

        // Fill cluster
        for i in 0..MAX_CLUSTER_SIZE {
            assert!(cluster.add_member(DroneId::new(i as u64)).is_ok());
        }

        // Cannot exceed limit
        assert!(cluster.add_member(DroneId::new(100)).is_err());
    }

    #[test]
    fn test_pbft_quorum() {
        let mut round = PBFTRound::new(1, 1);

        // 4 nodes: f=1, quorum=3
        round.add_prepare(DroneId::new(1)).unwrap();
        assert!(!round.has_prepare_quorum(4));

        round.add_prepare(DroneId::new(2)).unwrap();
        assert!(!round.has_prepare_quorum(4));

        round.add_prepare(DroneId::new(3)).unwrap();
        assert!(round.has_prepare_quorum(4));
    }

    #[test]
    fn test_hierarchy_tier_properties() {
        assert!(HierarchyTier::Local.typical_latency_ms() < HierarchyTier::Regional.typical_latency_ms());
        assert!(HierarchyTier::Regional.typical_latency_ms() < HierarchyTier::Global.typical_latency_ms());

        assert!(HierarchyTier::Local.max_nodes() <= 8);
        assert!(HierarchyTier::Regional.max_nodes() <= 10);
    }

    #[test]
    fn test_proposal_hash() {
        let proposal1 = HierarchyProposal {
            id: 1,
            command: HierarchyCommand::LocalEmergencyStop,
            proposer: DroneId::new(42),
            timestamp: 1000,
        };

        let proposal2 = HierarchyProposal {
            id: 2,
            command: HierarchyCommand::LocalEmergencyStop,
            proposer: DroneId::new(42),
            timestamp: 1000,
        };

        // Different proposals should have different hashes
        assert_ne!(proposal1.hash(), proposal2.hash());

        // Same proposal should have same hash
        assert_eq!(proposal1.hash(), proposal1.hash());
    }
}
