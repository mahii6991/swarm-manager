//! PBFT (Practical Byzantine Fault Tolerance) Protocol
//!
//! Implementation of PBFT for regional consensus in hierarchical swarms.
//! Used at Tier 2 to coordinate between cluster leaders.
//!
//! # Protocol Overview
//! - Tolerates f Byzantine failures with 3f+1 nodes
//! - Three phases: Pre-Prepare, Prepare, Commit
//! - View change for leader failure recovery

use crate::hierarchy_types::{
    HierarchyCommand, HierarchyDecision, HierarchyProposal, PBFTState, RegionId,
};
use crate::types::{DroneId, Result, SwarmError};
use heapless::{FnvIndexMap, FnvIndexSet, Vec};
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

/// Maximum PBFT participants (cluster leaders in a region)
pub const MAX_PBFT_NODES: usize = 16;

/// Maximum pending proposals
pub const MAX_PENDING_PROPOSALS: usize = 32;

/// Maximum log entries
pub const MAX_LOG_ENTRIES: usize = 128;

// ═══════════════════════════════════════════════════════════════════════════
// PBFT MESSAGE TYPES
// ═══════════════════════════════════════════════════════════════════════════

/// PBFT message types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PBFTMessage {
    /// Pre-prepare message (from primary)
    PrePrepare {
        view: u64,
        sequence: u64,
        proposal: HierarchyProposal,
        digest: [u8; 32],
    },
    /// Prepare message (from replicas)
    Prepare {
        view: u64,
        sequence: u64,
        digest: [u8; 32],
        sender: DroneId,
    },
    /// Commit message (from all)
    Commit {
        view: u64,
        sequence: u64,
        digest: [u8; 32],
        sender: DroneId,
    },
    /// View change request
    ViewChange {
        new_view: u64,
        sender: DroneId,
        last_sequence: u64,
    },
    /// New view announcement (from new primary)
    NewView { new_view: u64, sender: DroneId },
    /// Checkpoint message
    Checkpoint {
        sequence: u64,
        digest: [u8; 32],
        sender: DroneId,
    },
}

// ═══════════════════════════════════════════════════════════════════════════
// PBFT LOG ENTRY
// ═══════════════════════════════════════════════════════════════════════════

/// Log entry for PBFT
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PBFTLogEntry {
    /// Sequence number
    pub sequence: u64,
    /// View number
    pub view: u64,
    /// Proposal
    pub proposal: HierarchyProposal,
    /// Message digest
    pub digest: [u8; 32],
    /// Current phase
    pub state: PBFTState,
    /// Prepare votes received
    pub prepares: FnvIndexSet<u64, MAX_PBFT_NODES>,
    /// Commit votes received
    pub commits: FnvIndexSet<u64, MAX_PBFT_NODES>,
    /// Is committed?
    pub committed: bool,
    /// Is executed?
    pub executed: bool,
}

impl PBFTLogEntry {
    /// Create new log entry
    pub fn new(sequence: u64, view: u64, proposal: HierarchyProposal, digest: [u8; 32]) -> Self {
        Self {
            sequence,
            view,
            proposal,
            digest,
            state: PBFTState::PrePrepared,
            prepares: FnvIndexSet::new(),
            commits: FnvIndexSet::new(),
            committed: false,
            executed: false,
        }
    }

    /// Add prepare vote
    pub fn add_prepare(&mut self, sender: DroneId) -> Result<()> {
        self.prepares
            .insert(sender.as_u64())
            .map_err(|_| SwarmError::BufferFull)?;
        Ok(())
    }

    /// Add commit vote
    pub fn add_commit(&mut self, sender: DroneId) -> Result<()> {
        self.commits
            .insert(sender.as_u64())
            .map_err(|_| SwarmError::BufferFull)?;
        Ok(())
    }

    /// Check if prepared (2f+1 prepares)
    pub fn is_prepared(&self, quorum: usize) -> bool {
        self.prepares.len() >= quorum
    }

    /// Check if committed (2f+1 commits)
    pub fn is_committed_local(&self, quorum: usize) -> bool {
        self.commits.len() >= quorum
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PBFT NODE STATE
// ═══════════════════════════════════════════════════════════════════════════

/// PBFT node operational state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PBFTNodeState {
    /// Normal operation
    Normal,
    /// View change in progress
    ViewChange,
    /// Recovering
    Recovering,
}

// ═══════════════════════════════════════════════════════════════════════════
// PBFT NODE
// ═══════════════════════════════════════════════════════════════════════════

/// PBFT consensus node
#[derive(Debug, Clone)]
pub struct PBFTNode {
    /// Our node ID
    node_id: DroneId,
    /// Region we're in (used for message routing)
    #[allow(dead_code)]
    region_id: RegionId,
    /// Current view number
    view: u64,
    /// Current sequence number
    sequence: u64,
    /// Node state
    state: PBFTNodeState,
    /// Peer nodes (cluster leaders)
    peers: Vec<DroneId, MAX_PBFT_NODES>,
    /// Message log
    log: FnvIndexMap<u64, PBFTLogEntry, MAX_LOG_ENTRIES>,
    /// Last stable checkpoint
    last_checkpoint: u64,
    /// View change votes collected
    view_change_votes: FnvIndexSet<u64, MAX_PBFT_NODES>,
    /// Pending view change target
    pending_view_change: Option<u64>,
    /// Configuration (used for timeouts)
    #[allow(dead_code)]
    config: PBFTConfig,
    /// Statistics
    stats: PBFTStats,
    /// Next proposal ID
    next_proposal_id: u64,
}

/// PBFT configuration
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PBFTConfig {
    /// Request timeout (ms)
    pub request_timeout_ms: u32,
    /// View change timeout (ms)
    pub view_change_timeout_ms: u32,
    /// Checkpoint interval
    pub checkpoint_interval: u64,
}

impl Default for PBFTConfig {
    fn default() -> Self {
        Self {
            request_timeout_ms: 5000,
            view_change_timeout_ms: 10000,
            checkpoint_interval: 100,
        }
    }
}

/// PBFT statistics
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct PBFTStats {
    /// Total proposals
    pub total_proposals: u64,
    /// Committed proposals
    pub committed_proposals: u64,
    /// View changes
    pub view_changes: u64,
    /// Byzantine detections
    pub byzantine_detections: u64,
}

impl PBFTNode {
    /// Create a new PBFT node
    pub fn new(
        node_id: DroneId,
        region_id: RegionId,
        peers: Vec<DroneId, MAX_PBFT_NODES>,
        config: PBFTConfig,
    ) -> Self {
        Self {
            node_id,
            region_id,
            view: 0,
            sequence: 0,
            state: PBFTNodeState::Normal,
            peers,
            log: FnvIndexMap::new(),
            last_checkpoint: 0,
            view_change_votes: FnvIndexSet::new(),
            pending_view_change: None,
            config,
            stats: PBFTStats::default(),
            next_proposal_id: 1,
        }
    }

    /// Create with default config
    pub fn new_default(
        node_id: DroneId,
        region_id: RegionId,
        peers: Vec<DroneId, MAX_PBFT_NODES>,
    ) -> Self {
        Self::new(node_id, region_id, peers, PBFTConfig::default())
    }

    /// Get node ID
    pub fn node_id(&self) -> DroneId {
        self.node_id
    }

    /// Get current view
    pub fn view(&self) -> u64 {
        self.view
    }

    /// Get current sequence
    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Get node state
    pub fn state(&self) -> PBFTNodeState {
        self.state
    }

    /// Check if we are the primary
    pub fn is_primary(&self) -> bool {
        self.get_primary() == self.node_id
    }

    /// Get primary for current view
    pub fn get_primary(&self) -> DroneId {
        let n = self.peers.len();
        if n == 0 {
            return self.node_id;
        }
        let primary_idx = (self.view as usize) % n;
        self.peers.get(primary_idx).copied().unwrap_or(self.node_id)
    }

    /// Get number of Byzantine faults tolerated
    pub fn max_faulty(&self) -> usize {
        let n = self.peers.len();
        if n <= 3 {
            0
        } else {
            (n - 1) / 3
        }
    }

    /// Get quorum size (2f+1)
    pub fn quorum(&self) -> usize {
        let f = self.max_faulty();
        2 * f + 1
    }

    /// Create a proposal for a command
    pub fn create_proposal(&mut self, command: HierarchyCommand, timestamp: u64) -> HierarchyProposal {
        let id = self.next_proposal_id;
        self.next_proposal_id += 1;

        HierarchyProposal {
            id,
            command,
            proposer: self.node_id,
            timestamp,
        }
    }

    /// Propose a new value (only primary)
    pub fn propose(&mut self, proposal: HierarchyProposal) -> Result<Option<PBFTMessage>> {
        if !self.is_primary() {
            return Err(SwarmError::ConsensusError);
        }
        if self.state != PBFTNodeState::Normal {
            return Err(SwarmError::Timeout);
        }

        self.sequence += 1;
        let sequence = self.sequence;
        let view = self.view;
        let digest = proposal.hash();

        // Create log entry
        let entry = PBFTLogEntry::new(sequence, view, proposal.clone(), digest);
        self.log
            .insert(sequence, entry)
            .map_err(|_| SwarmError::BufferFull)?;

        self.stats.total_proposals += 1;

        Ok(Some(PBFTMessage::PrePrepare {
            view,
            sequence,
            proposal,
            digest,
        }))
    }

    /// Handle pre-prepare message
    pub fn handle_pre_prepare(
        &mut self,
        view: u64,
        sequence: u64,
        proposal: HierarchyProposal,
        digest: [u8; 32],
        sender: DroneId,
    ) -> Result<Option<PBFTMessage>> {
        // Verify sender is primary
        if sender != self.get_primary() {
            self.stats.byzantine_detections += 1;
            return Err(SwarmError::ConsensusError);
        }

        // Verify view
        if view != self.view {
            return Ok(None);
        }

        // Verify digest
        let computed_digest = proposal.hash();
        if computed_digest != digest {
            self.stats.byzantine_detections += 1;
            return Err(SwarmError::ConsensusError);
        }

        // Don't accept if already have entry for this sequence
        if let Some(existing) = self.log.get(&sequence) {
            if existing.digest != digest {
                self.stats.byzantine_detections += 1;
                return Err(SwarmError::ConsensusError);
            }
            return Ok(None);
        }

        // Create log entry
        let mut entry = PBFTLogEntry::new(sequence, view, proposal, digest);
        entry.state = PBFTState::Prepared;
        entry.add_prepare(self.node_id)?;

        self.log
            .insert(sequence, entry)
            .map_err(|_| SwarmError::BufferFull)?;

        if sequence > self.sequence {
            self.sequence = sequence;
        }

        Ok(Some(PBFTMessage::Prepare {
            view,
            sequence,
            digest,
            sender: self.node_id,
        }))
    }

    /// Handle prepare message
    pub fn handle_prepare(
        &mut self,
        view: u64,
        sequence: u64,
        digest: [u8; 32],
        sender: DroneId,
    ) -> Result<Option<PBFTMessage>> {
        if view != self.view {
            return Ok(None);
        }

        // Capture quorum before mutable borrow
        let quorum = self.quorum();

        let entry = if let Some(e) = self.log.get_mut(&sequence) {
            e
        } else {
            return Ok(None);
        };

        if entry.digest != digest {
            self.stats.byzantine_detections += 1;
            return Ok(None);
        }

        entry.add_prepare(sender)?;

        if entry.is_prepared(quorum) && entry.state == PBFTState::Prepared {
            entry.state = PBFTState::Committed;

            return Ok(Some(PBFTMessage::Commit {
                view,
                sequence,
                digest,
                sender: self.node_id,
            }));
        }

        Ok(None)
    }

    /// Handle commit message
    pub fn handle_commit(
        &mut self,
        view: u64,
        sequence: u64,
        digest: [u8; 32],
        sender: DroneId,
    ) -> Result<Option<HierarchyDecision>> {
        if view != self.view {
            return Ok(None);
        }

        // Capture quorum before mutable borrow
        let quorum = self.quorum();

        let entry = if let Some(e) = self.log.get_mut(&sequence) {
            e
        } else {
            return Ok(None);
        };

        if entry.digest != digest {
            return Ok(None);
        }

        entry.add_commit(sender)?;

        if entry.is_committed_local(quorum) && !entry.committed {
            entry.committed = true;
            entry.state = PBFTState::Committed;
            self.stats.committed_proposals += 1;

            let decision = HierarchyDecision {
                proposal: entry.proposal.clone(),
                sequence,
                vote_count: entry.commits.len() as u32,
                decided_at: entry.proposal.timestamp,
            };

            return Ok(Some(decision));
        }

        Ok(None)
    }

    /// Start view change
    pub fn start_view_change(&mut self) -> Result<PBFTMessage> {
        let new_view = self.view + 1;
        self.pending_view_change = Some(new_view);
        self.state = PBFTNodeState::ViewChange;
        self.view_change_votes.clear();
        self.view_change_votes
            .insert(self.node_id.as_u64())
            .map_err(|_| SwarmError::BufferFull)?;

        Ok(PBFTMessage::ViewChange {
            new_view,
            sender: self.node_id,
            last_sequence: self.sequence,
        })
    }

    /// Handle view change message
    pub fn handle_view_change(
        &mut self,
        new_view: u64,
        sender: DroneId,
        _last_sequence: u64,
    ) -> Result<Option<PBFTMessage>> {
        if new_view <= self.view {
            return Ok(None);
        }

        if self.pending_view_change.is_none() || self.pending_view_change != Some(new_view) {
            self.pending_view_change = Some(new_view);
            self.state = PBFTNodeState::ViewChange;
            self.view_change_votes.clear();
        }

        self.view_change_votes
            .insert(sender.as_u64())
            .map_err(|_| SwarmError::BufferFull)?;

        if self.view_change_votes.len() >= self.quorum() {
            self.view = new_view;
            self.state = PBFTNodeState::Normal;
            self.pending_view_change = None;
            self.view_change_votes.clear();
            self.stats.view_changes += 1;

            if self.is_primary() {
                return Ok(Some(PBFTMessage::NewView {
                    new_view,
                    sender: self.node_id,
                }));
            }
        }

        Ok(None)
    }

    /// Handle new view message
    pub fn handle_new_view(&mut self, new_view: u64, sender: DroneId) -> Result<()> {
        let expected_primary_idx = (new_view as usize) % self.peers.len();
        let expected_primary = self.peers.get(expected_primary_idx).copied();

        if expected_primary != Some(sender) {
            return Err(SwarmError::ConsensusError);
        }

        self.view = new_view;
        self.state = PBFTNodeState::Normal;
        self.pending_view_change = None;
        self.view_change_votes.clear();

        Ok(())
    }

    /// Check if sequence is committed
    pub fn is_committed(&self, sequence: u64) -> bool {
        self.log.get(&sequence).map(|e| e.committed).unwrap_or(false)
    }

    /// Get committed entries after last checkpoint
    pub fn get_committed_entries(&self) -> Vec<&PBFTLogEntry, MAX_LOG_ENTRIES> {
        self.log
            .values()
            .filter(|e| e.committed && e.sequence > self.last_checkpoint)
            .collect()
    }

    /// Get statistics
    pub fn stats(&self) -> &PBFTStats {
        &self.stats
    }

    /// Garbage collect old log entries
    pub fn garbage_collect(&mut self, stable_checkpoint: u64) {
        self.log.retain(|seq, _| *seq > stable_checkpoint);
        self.last_checkpoint = stable_checkpoint;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_peers() -> Vec<DroneId, MAX_PBFT_NODES> {
        let mut peers = Vec::new();
        for i in 0..4 {
            peers.push(DroneId::new(i)).ok();
        }
        peers
    }

    #[test]
    fn test_pbft_creation() {
        let peers = create_test_peers();
        let node = PBFTNode::new_default(DroneId::new(0), RegionId::new(1), peers);

        assert_eq!(node.view(), 0);
        assert_eq!(node.sequence(), 0);
        assert!(node.is_primary());
    }

    #[test]
    fn test_quorum_calculation() {
        let peers = create_test_peers();
        let node = PBFTNode::new_default(DroneId::new(0), RegionId::new(1), peers);

        assert_eq!(node.max_faulty(), 1);
        assert_eq!(node.quorum(), 3);
    }

    #[test]
    fn test_primary_rotation() {
        let peers = create_test_peers();
        let mut node = PBFTNode::new_default(DroneId::new(0), RegionId::new(1), peers);

        assert_eq!(node.get_primary(), DroneId::new(0));

        node.view = 1;
        assert_eq!(node.get_primary(), DroneId::new(1));

        node.view = 2;
        assert_eq!(node.get_primary(), DroneId::new(2));
    }

    #[test]
    fn test_propose() {
        let peers = create_test_peers();
        let mut node = PBFTNode::new_default(DroneId::new(0), RegionId::new(1), peers);

        let proposal = node.create_proposal(HierarchyCommand::LocalEmergencyStop, 1000);
        let msg = node.propose(proposal).unwrap();

        assert!(msg.is_some());
        match msg.unwrap() {
            PBFTMessage::PrePrepare { view, sequence, .. } => {
                assert_eq!(view, 0);
                assert_eq!(sequence, 1);
            }
            _ => panic!("Expected PrePrepare message"),
        }
    }

    #[test]
    fn test_non_primary_cannot_propose() {
        let peers = create_test_peers();
        let mut node = PBFTNode::new_default(DroneId::new(1), RegionId::new(1), peers);

        let proposal = node.create_proposal(HierarchyCommand::LocalEmergencyStop, 1000);
        let result = node.propose(proposal);

        assert!(result.is_err());
    }

    #[test]
    fn test_log_entry_phases() {
        let proposal = HierarchyProposal {
            id: 1,
            command: HierarchyCommand::LocalEmergencyStop,
            proposer: DroneId::new(0),
            timestamp: 1000,
        };
        let mut entry = PBFTLogEntry::new(1, 0, proposal, [0u8; 32]);

        entry.add_prepare(DroneId::new(0)).unwrap();
        entry.add_prepare(DroneId::new(1)).unwrap();
        entry.add_prepare(DroneId::new(2)).unwrap();

        assert!(entry.is_prepared(3));

        entry.add_commit(DroneId::new(0)).unwrap();
        entry.add_commit(DroneId::new(1)).unwrap();
        entry.add_commit(DroneId::new(2)).unwrap();

        assert!(entry.is_committed_local(3));
    }

    #[test]
    fn test_view_change() {
        let peers = create_test_peers();
        let mut node = PBFTNode::new_default(DroneId::new(0), RegionId::new(1), peers);

        let msg = node.start_view_change().unwrap();
        match msg {
            PBFTMessage::ViewChange { new_view, .. } => {
                assert_eq!(new_view, 1);
            }
            _ => panic!("Expected ViewChange message"),
        }

        assert_eq!(node.state(), PBFTNodeState::ViewChange);
    }
}
