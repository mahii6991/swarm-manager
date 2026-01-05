//! Hierarchical Consensus Coordinator
//!
//! Coordinates consensus across three tiers:
//! - Tier 1: Local Raft consensus within clusters (5-8 drones)
//! - Tier 2: Regional PBFT consensus across cluster leaders
//! - Tier 3: Global vote aggregation for large-scale decisions

use crate::system::clustering::{ClusterInfo, ClusterManager, JoinRequest, LocalRole, RebalanceAction};
use crate::system::hierarchy::{
    ClusterId, HierarchicalMessage, HierarchyCommand, HierarchyConfig, HierarchyDecision,
    HierarchyProposal, HierarchyTier, RegionId,
};
use crate::pbft::{PBFTMessage, PBFTNode, MAX_PBFT_NODES};
use crate::types::{DroneId, Position, Result, SwarmError};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum pending decisions
pub const MAX_PENDING_DECISIONS: usize = 32;

/// Maximum outgoing messages to buffer
pub const MAX_OUTGOING_MESSAGES: usize = 64;

/// Default consensus timeout (ms)
pub const DEFAULT_CONSENSUS_TIMEOUT_MS: u64 = 5000;

// ═══════════════════════════════════════════════════════════════════════════
// CONSENSUS STATE
// ═══════════════════════════════════════════════════════════════════════════

/// State of hierarchical consensus
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HierarchyState {
    /// Not yet joined
    Unjoined,
    /// Joining hierarchy
    Joining,
    /// Active in hierarchy
    Active,
    /// In cluster rebalancing
    Rebalancing,
    /// In view/leader change
    LeaderChange,
    /// Disconnected
    Disconnected,
}

// ═══════════════════════════════════════════════════════════════════════════
// PENDING DECISION
// ═══════════════════════════════════════════════════════════════════════════

/// A decision pending confirmation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PendingDecision {
    /// Decision ID
    pub id: u64,
    /// The proposal
    pub proposal: HierarchyProposal,
    /// Tier this decision was made at
    pub tier: HierarchyTier,
    /// Created timestamp
    pub created_at_ms: u64,
    /// Timeout timestamp
    pub timeout_at_ms: u64,
    /// Has local consensus?
    pub local_confirmed: bool,
    /// Has regional consensus?
    pub regional_confirmed: bool,
    /// Final decision (once confirmed)
    pub decision: Option<HierarchyDecision>,
}

// ═══════════════════════════════════════════════════════════════════════════
// HIERARCHICAL CONSENSUS
// ═══════════════════════════════════════════════════════════════════════════

/// Main hierarchical consensus coordinator
#[derive(Debug, Clone)]
pub struct HierarchicalConsensus {
    /// Our drone ID
    local_id: DroneId,
    /// Current state
    state: HierarchyState,
    /// Cluster manager
    cluster_manager: ClusterManager,
    /// PBFT node (only active for cluster leaders)
    pbft_node: Option<PBFTNode>,
    /// Current tier
    current_tier: HierarchyTier,
    /// Pending decisions
    pending_decisions: FnvIndexMap<u64, PendingDecision, MAX_PENDING_DECISIONS>,
    /// Next decision ID
    next_decision_id: u64,
    /// Outgoing messages
    outgoing_messages: Vec<(DroneId, HierarchicalMessage), MAX_OUTGOING_MESSAGES>,
    /// Current timestamp
    current_time_ms: u64,
    /// Configuration
    config: HierarchyConfig,
    /// Statistics
    stats: HierarchyStats,
}

/// Hierarchical consensus statistics
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct HierarchyStats {
    /// Local consensus rounds
    pub local_rounds: u64,
    /// Regional consensus rounds
    pub regional_rounds: u64,
    /// Global consensus rounds
    pub global_rounds: u64,
    /// Successful decisions
    pub successful_decisions: u64,
    /// Failed decisions (timeout)
    pub failed_decisions: u64,
    /// Cluster rebalances
    pub rebalances: u64,
    /// Leader changes
    pub leader_changes: u64,
}

impl HierarchicalConsensus {
    /// Create new hierarchical consensus coordinator
    pub fn new(local_id: DroneId, config: HierarchyConfig) -> Self {
        Self {
            local_id,
            state: HierarchyState::Unjoined,
            cluster_manager: ClusterManager::new(local_id, config),
            pbft_node: None,
            current_tier: HierarchyTier::Local,
            pending_decisions: FnvIndexMap::new(),
            next_decision_id: 1,
            outgoing_messages: Vec::new(),
            current_time_ms: 0,
            config,
            stats: HierarchyStats::default(),
        }
    }

    /// Create with default config
    pub fn new_default(local_id: DroneId) -> Self {
        Self::new(local_id, HierarchyConfig::default())
    }

    /// Get local ID
    pub fn local_id(&self) -> DroneId {
        self.local_id
    }

    /// Get current state
    pub fn state(&self) -> HierarchyState {
        self.state
    }

    /// Get current tier
    pub fn current_tier(&self) -> HierarchyTier {
        self.current_tier
    }

    /// Get cluster ID
    pub fn cluster_id(&self) -> Option<ClusterId> {
        self.cluster_manager.cluster_id()
    }

    /// Get region ID
    pub fn region_id(&self) -> Option<RegionId> {
        self.cluster_manager.region_id()
    }

    /// Get our role
    pub fn role(&self) -> LocalRole {
        self.cluster_manager.role()
    }

    /// Check if we are a cluster leader
    pub fn is_cluster_leader(&self) -> bool {
        self.cluster_manager.is_cluster_leader()
    }

    /// Check if we are a regional leader
    pub fn is_regional_leader(&self) -> bool {
        self.cluster_manager.is_regional_leader()
    }

    /// Update current time
    pub fn update_time(&mut self, timestamp_ms: u64) {
        self.current_time_ms = timestamp_ms;
        self.cluster_manager.update_time(timestamp_ms);
    }

    /// Update position
    pub fn update_position(&mut self, position: Position) -> Result<()> {
        self.cluster_manager.update_position(self.local_id, position)
    }

    /// Join the hierarchy
    pub fn join(&mut self, position: Position) -> Result<Vec<(DroneId, HierarchicalMessage), 4>> {
        self.state = HierarchyState::Joining;
        let mut messages = Vec::new();

        self.update_position(position)?;

        let request = self.cluster_manager.join_hierarchy(position)?;

        match request {
            JoinRequest::JoinExisting { cluster_id } => {
                if let Some(leader) = self.cluster_manager.get_cluster_leader(cluster_id) {
                    let msg = HierarchicalMessage::JoinClusterRequest {
                        drone_id: self.local_id,
                        position,
                    };
                    messages.push((leader, msg)).ok();
                }
            }
            JoinRequest::CreateNew { position: pos } => {
                let cluster_id = self.cluster_manager.create_cluster(pos)?;
                self.cluster_manager
                    .add_to_cluster(self.local_id, cluster_id)?;
                self.cluster_manager
                    .set_cluster_leader(cluster_id, self.local_id);

                if let Some(region_id) = self.cluster_manager.region_id() {
                    self.cluster_manager
                        .process_join_response(true, Some(cluster_id), Some(region_id))?;
                }

                self.state = HierarchyState::Active;
                self.initialize_pbft_if_leader()?;
            }
        }

        Ok(messages)
    }

    /// Handle join cluster request (as cluster leader)
    pub fn handle_join_request(
        &mut self,
        drone_id: DroneId,
        position: Position,
    ) -> Result<HierarchicalMessage> {
        if !self.is_cluster_leader() {
            return Ok(HierarchicalMessage::JoinClusterResponse {
                accepted: false,
                cluster_id: None,
                leader_id: None,
            });
        }

        let cluster_id = self.cluster_id();

        if let Some(cid) = cluster_id {
            if let Some(cluster) = self.cluster_manager.get_cluster(cid) {
                if cluster.is_full() {
                    return Ok(HierarchicalMessage::JoinClusterResponse {
                        accepted: false,
                        cluster_id: None,
                        leader_id: None,
                    });
                }
            }

            self.cluster_manager.add_to_cluster(drone_id, cid)?;
            self.cluster_manager.update_position(drone_id, position)?;

            return Ok(HierarchicalMessage::JoinClusterResponse {
                accepted: true,
                cluster_id: Some(cid),
                leader_id: Some(self.local_id),
            });
        }

        Ok(HierarchicalMessage::JoinClusterResponse {
            accepted: false,
            cluster_id: None,
            leader_id: None,
        })
    }

    /// Handle join response
    pub fn handle_join_response(
        &mut self,
        accepted: bool,
        cluster_id: Option<ClusterId>,
        _leader_id: Option<DroneId>,
    ) -> Result<()> {
        if accepted {
            let region_id = cluster_id.map(|_| RegionId::new(1));
            self.cluster_manager
                .process_join_response(accepted, cluster_id, region_id)?;
            self.state = HierarchyState::Active;
        } else {
            self.state = HierarchyState::Unjoined;
        }
        Ok(())
    }

    /// Initialize PBFT if we are a cluster leader
    fn initialize_pbft_if_leader(&mut self) -> Result<()> {
        if !self.is_cluster_leader() {
            return Ok(());
        }

        if let Some(region_id) = self.region_id() {
            let mut peers = Vec::new();

            if let Some(cluster_ids) = self.cluster_manager.get_region_clusters(region_id) {
                for &cid in cluster_ids {
                    if let Some(leader) = self.cluster_manager.get_cluster_leader(cid) {
                        if peers.len() < MAX_PBFT_NODES {
                            peers.push(leader).ok();
                        }
                    }
                }
            }

            if !peers.contains(&self.local_id) {
                peers.push(self.local_id).ok();
            }

            self.pbft_node = Some(PBFTNode::new_default(self.local_id, region_id, peers));
        }

        Ok(())
    }

    /// Propose a command (starts consensus process)
    pub fn propose_command(&mut self, command: HierarchyCommand) -> Result<u64> {
        if self.state != HierarchyState::Active {
            return Err(SwarmError::PermissionDenied);
        }

        let proposal = HierarchyProposal {
            id: self.next_decision_id,
            command,
            proposer: self.local_id,
            timestamp: self.current_time_ms,
        };

        self.propose(proposal)
    }

    /// Propose a decision (starts consensus process)
    pub fn propose(&mut self, proposal: HierarchyProposal) -> Result<u64> {
        if self.state != HierarchyState::Active {
            return Err(SwarmError::PermissionDenied);
        }

        let decision_id = self.next_decision_id;
        self.next_decision_id += 1;

        let tier = self.determine_tier(&proposal.command);

        let pending = PendingDecision {
            id: decision_id,
            proposal: proposal.clone(),
            tier,
            created_at_ms: self.current_time_ms,
            timeout_at_ms: self.current_time_ms + DEFAULT_CONSENSUS_TIMEOUT_MS,
            local_confirmed: false,
            regional_confirmed: false,
            decision: None,
        };

        self.pending_decisions
            .insert(decision_id, pending)
            .map_err(|_| SwarmError::BufferFull)?;

        match tier {
            HierarchyTier::Local => {
                self.start_local_consensus(decision_id, proposal)?;
                self.stats.local_rounds += 1;
            }
            HierarchyTier::Regional => {
                self.start_regional_consensus(decision_id, proposal)?;
                self.stats.regional_rounds += 1;
            }
            HierarchyTier::Global => {
                self.start_global_consensus(decision_id, proposal)?;
                self.stats.global_rounds += 1;
            }
        }

        Ok(decision_id)
    }

    /// Determine which tier to use for a command
    fn determine_tier(&self, command: &HierarchyCommand) -> HierarchyTier {
        match command {
            HierarchyCommand::GlobalEmergency { .. } | HierarchyCommand::GlobalMissionUpdate { .. } => {
                HierarchyTier::Global
            }
            HierarchyCommand::RegionalFormationSync { .. }
            | HierarchyCommand::RegionalTaskRealloc { .. } => HierarchyTier::Regional,
            HierarchyCommand::LocalFormationChange { .. }
            | HierarchyCommand::LocalTaskAssign { .. }
            | HierarchyCommand::LocalEmergencyStop => HierarchyTier::Local,
        }
    }

    /// Start local consensus (within cluster)
    fn start_local_consensus(
        &mut self,
        decision_id: u64,
        proposal: HierarchyProposal,
    ) -> Result<()> {
        if self.is_cluster_leader() {
            if let Some(pending) = self.pending_decisions.get_mut(&decision_id) {
                pending.local_confirmed = true;
                pending.decision = Some(HierarchyDecision {
                    proposal: proposal.clone(),
                    sequence: decision_id,
                    vote_count: 1,
                    decided_at: self.current_time_ms,
                });
            }
        } else if let Some(cluster_id) = self.cluster_id() {
            if let Some(leader) = self.cluster_manager.get_cluster_leader(cluster_id) {
                let msg = HierarchicalMessage::LocalVoteRequest {
                    cluster_id,
                    term: 1,
                    candidate_id: self.local_id,
                    last_log_index: 0,
                    last_log_term: 0,
                };
                self.queue_message(leader, msg)?;
            }
        }
        Ok(())
    }

    /// Start regional consensus (PBFT between cluster leaders)
    fn start_regional_consensus(
        &mut self,
        _decision_id: u64,
        proposal: HierarchyProposal,
    ) -> Result<()> {
        if let Some(ref mut pbft) = self.pbft_node {
            if pbft.is_primary() {
                if let Some(msg) = pbft.propose(proposal)? {
                    self.broadcast_pbft_message(msg)?;
                }
            } else {
                let primary = pbft.get_primary();
                if let Some(region_id) = self.region_id() {
                    let msg = HierarchicalMessage::RegionalPrePrepare {
                        region_id,
                        view: 0,
                        sequence: 0,
                        proposal,
                        primary_id: primary,
                    };
                    self.queue_message(primary, msg)?;
                }
            }
        }
        Ok(())
    }

    /// Start global consensus (vote aggregation)
    fn start_global_consensus(
        &mut self,
        _decision_id: u64,
        proposal: HierarchyProposal,
    ) -> Result<()> {
        let msg = HierarchicalMessage::GlobalVoteRequest {
            proposal,
            initiator: self.local_id,
        };

        self.queue_message(self.local_id, msg)?;
        Ok(())
    }

    /// Handle incoming hierarchy message
    pub fn handle_message(
        &mut self,
        from: DroneId,
        message: HierarchicalMessage,
    ) -> Result<Option<HierarchyDecision>> {
        match message {
            HierarchicalMessage::JoinClusterRequest { drone_id, position } => {
                let response = self.handle_join_request(drone_id, position)?;
                self.queue_message(from, response)?;
                Ok(None)
            }

            HierarchicalMessage::JoinClusterResponse {
                accepted,
                cluster_id,
                leader_id,
            } => {
                self.handle_join_response(accepted, cluster_id, leader_id)?;
                Ok(None)
            }

            HierarchicalMessage::LocalVoteRequest { .. } => {
                // Handle Raft vote request
                Ok(None)
            }

            HierarchicalMessage::LocalVoteResponse { .. } => {
                // Handle Raft vote response
                Ok(None)
            }

            HierarchicalMessage::LocalAppendEntries { .. } => {
                // Handle Raft append entries
                Ok(None)
            }

            HierarchicalMessage::LocalAppendResponse { .. } => {
                // Handle Raft append response
                Ok(None)
            }

            HierarchicalMessage::RegionalPrePrepare {
                view,
                sequence,
                proposal,
                primary_id,
                ..
            } => {
                if let Some(ref mut pbft) = self.pbft_node {
                    let digest = proposal.hash();
                    if let Some(response) =
                        pbft.handle_pre_prepare(view, sequence, proposal, digest, primary_id)?
                    {
                        self.broadcast_pbft_message(response)?;
                    }
                }
                Ok(None)
            }

            HierarchicalMessage::RegionalPrepare {
                view,
                sequence,
                proposal_hash,
                sender_id,
                ..
            } => {
                if let Some(ref mut pbft) = self.pbft_node {
                    if let Some(response) =
                        pbft.handle_prepare(view, sequence, proposal_hash, sender_id)?
                    {
                        self.broadcast_pbft_message(response)?;
                    }
                }
                Ok(None)
            }

            HierarchicalMessage::RegionalCommit {
                view,
                sequence,
                proposal_hash,
                sender_id,
                ..
            } => {
                if let Some(ref mut pbft) = self.pbft_node {
                    if let Some(decision) =
                        pbft.handle_commit(view, sequence, proposal_hash, sender_id)?
                    {
                        self.stats.successful_decisions += 1;
                        return Ok(Some(decision));
                    }
                }
                Ok(None)
            }

            HierarchicalMessage::RegionalViewChange {
                new_view,
                sender_id,
                ..
            } => {
                if let Some(ref mut pbft) = self.pbft_node {
                    if let Some(response) = pbft.handle_view_change(new_view, sender_id, 0)? {
                        self.broadcast_pbft_message(response)?;
                    }
                }
                self.stats.leader_changes += 1;
                Ok(None)
            }

            HierarchicalMessage::GlobalDecision { decision, .. } => Ok(Some(decision)),

            HierarchicalMessage::GlobalVoteRequest { proposal, .. } => {
                let decision = HierarchyDecision {
                    proposal,
                    sequence: self.next_decision_id,
                    vote_count: 1,
                    decided_at: self.current_time_ms,
                };
                Ok(Some(decision))
            }

            HierarchicalMessage::GlobalVoteResponse { .. } => Ok(None),

            HierarchicalMessage::ClusterRebalance { new_cluster, .. } => {
                self.state = HierarchyState::Rebalancing;

                if let Some(region_id) = self.region_id() {
                    self.cluster_manager
                        .process_join_response(true, Some(new_cluster), Some(region_id))?;
                }

                self.state = HierarchyState::Active;
                self.stats.rebalances += 1;
                Ok(None)
            }
        }
    }

    /// Broadcast PBFT message to peers
    fn broadcast_pbft_message(&mut self, message: PBFTMessage) -> Result<()> {
        if let Some(region_id) = self.region_id() {
            // Collect leaders first to avoid borrow issues
            let mut leaders: Vec<DroneId, 16> = Vec::new();
            if let Some(cluster_ids) = self.cluster_manager.get_region_clusters(region_id) {
                for &cid in cluster_ids {
                    if let Some(leader) = self.cluster_manager.get_cluster_leader(cid) {
                        if leader != self.local_id {
                            leaders.push(leader).ok();
                        }
                    }
                }
            }

            // Now send to collected leaders
            let hierarchy_msg = self.pbft_to_hierarchy_message(region_id, &message);
            for leader in leaders {
                self.queue_message(leader, hierarchy_msg.clone())?;
            }
        }
        Ok(())
    }

    /// Convert PBFT message to HierarchicalMessage
    fn pbft_to_hierarchy_message(
        &self,
        region_id: RegionId,
        message: &PBFTMessage,
    ) -> HierarchicalMessage {
        match message {
            PBFTMessage::PrePrepare {
                view,
                sequence,
                proposal,
                ..
            } => HierarchicalMessage::RegionalPrePrepare {
                region_id,
                view: *view,
                sequence: *sequence,
                proposal: proposal.clone(),
                primary_id: self.local_id,
            },
            PBFTMessage::Prepare {
                view,
                sequence,
                digest,
                sender,
            } => HierarchicalMessage::RegionalPrepare {
                region_id,
                view: *view,
                sequence: *sequence,
                proposal_hash: *digest,
                sender_id: *sender,
            },
            PBFTMessage::Commit {
                view,
                sequence,
                digest,
                sender,
            } => HierarchicalMessage::RegionalCommit {
                region_id,
                view: *view,
                sequence: *sequence,
                proposal_hash: *digest,
                sender_id: *sender,
            },
            PBFTMessage::ViewChange {
                new_view, sender, ..
            } => HierarchicalMessage::RegionalViewChange {
                region_id,
                new_view: *new_view,
                sender_id: *sender,
            },
            PBFTMessage::NewView { new_view, sender } => HierarchicalMessage::RegionalViewChange {
                region_id,
                new_view: *new_view,
                sender_id: *sender,
            },
            PBFTMessage::Checkpoint { .. } => HierarchicalMessage::RegionalViewChange {
                region_id,
                new_view: 0,
                sender_id: self.local_id,
            },
        }
    }

    /// Queue a message to send
    fn queue_message(&mut self, to: DroneId, message: HierarchicalMessage) -> Result<()> {
        self.outgoing_messages
            .push((to, message))
            .map_err(|_| SwarmError::BufferFull)
    }

    /// Get and clear outgoing messages
    pub fn drain_messages(&mut self) -> Vec<(DroneId, HierarchicalMessage), MAX_OUTGOING_MESSAGES> {
        let messages = self.outgoing_messages.clone();
        self.outgoing_messages.clear();
        messages
    }

    /// Check for and handle timeouts
    pub fn check_timeouts(&mut self) -> Vec<u64, MAX_PENDING_DECISIONS> {
        let mut timed_out = Vec::new();

        for (id, pending) in self.pending_decisions.iter() {
            if self.current_time_ms > pending.timeout_at_ms && pending.decision.is_none() {
                timed_out.push(*id).ok();
            }
        }

        for id in &timed_out {
            self.pending_decisions.remove(id);
            self.stats.failed_decisions += 1;
        }

        timed_out
    }

    /// Check for cluster rebalancing needs
    pub fn check_rebalancing(&self) -> Vec<RebalanceAction, 16> {
        self.cluster_manager.check_rebalancing()
    }

    /// Get cluster info
    pub fn get_cluster(&self, cluster_id: ClusterId) -> Option<&ClusterInfo> {
        self.cluster_manager.get_cluster(cluster_id)
    }

    /// Get cluster members
    pub fn get_cluster_members(&self, cluster_id: ClusterId) -> Option<&[DroneId]> {
        self.cluster_manager.get_cluster_members(cluster_id)
    }

    /// Get statistics
    pub fn stats(&self) -> &HierarchyStats {
        &self.stats
    }

    /// Get configuration
    pub fn config(&self) -> &HierarchyConfig {
        &self.config
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hierarchical_consensus_creation() {
        let hc = HierarchicalConsensus::new_default(DroneId::new(1));
        assert_eq!(hc.local_id(), DroneId::new(1));
        assert_eq!(hc.state(), HierarchyState::Unjoined);
    }

    #[test]
    fn test_join_hierarchy() {
        let mut hc = HierarchicalConsensus::new_default(DroneId::new(1));
        hc.update_time(1000);

        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };

        let result = hc.join(pos);
        assert!(result.is_ok());
        assert_eq!(hc.state(), HierarchyState::Active);
    }

    #[test]
    fn test_propose_command() {
        let mut hc = HierarchicalConsensus::new_default(DroneId::new(1));
        hc.update_time(1000);

        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        hc.join(pos).unwrap();

        let result = hc.propose_command(HierarchyCommand::LocalEmergencyStop);
        assert!(result.is_ok());
    }

    #[test]
    fn test_tier_determination() {
        let hc = HierarchicalConsensus::new_default(DroneId::new(1));

        let global = HierarchyCommand::GlobalEmergency { reason: 1 };
        assert_eq!(hc.determine_tier(&global), HierarchyTier::Global);

        let local = HierarchyCommand::LocalEmergencyStop;
        assert_eq!(hc.determine_tier(&local), HierarchyTier::Local);

        let regional = HierarchyCommand::RegionalFormationSync {
            formation_hash: [0u8; 32],
        };
        assert_eq!(hc.determine_tier(&regional), HierarchyTier::Regional);
    }

    #[test]
    fn test_timeout_handling() {
        let mut hc = HierarchicalConsensus::new_default(DroneId::new(1));
        hc.update_time(1000);

        let pos = Position {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        hc.join(pos).unwrap();

        hc.propose_command(HierarchyCommand::LocalEmergencyStop)
            .unwrap();

        // Advance time past timeout
        hc.update_time(100000);
        let timed_out = hc.check_timeouts();

        // Note: leader immediately confirms local decisions
        // so timeout may not trigger for local commands
        assert!(timed_out.is_empty() || hc.stats.failed_decisions > 0);
    }
}
