//! Raft-based consensus protocol for swarm coordination
//!
//! Implements a lightweight Raft consensus adapted for drone swarms (SwarmRaft).
//! Provides:
//! - Leader election with crash fault tolerance
//! - Replicated log for consistent state
//! - Low-latency agreement suitable for real-time swarm operations
//! - Resource-constrained optimization

use crate::types::*;
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};

/// Raft node states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeState {
    /// Follower state
    Follower,
    /// Candidate state (during election)
    Candidate,
    /// Leader state
    Leader,
}

/// Raft log entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogEntry {
    /// Term when entry was created
    pub term: u64,
    /// Command index
    pub index: u64,
    /// Command data
    pub command: SwarmCommand,
}

/// Commands that can be replicated via consensus
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SwarmCommand {
    /// Assign task to drone
    AssignTask { drone: DroneId, task_id: u64 },
    /// Update mission parameters
    UpdateMission { params: Vec<u8, 256> },
    /// Add drone to swarm
    AddDrone { drone: DroneId },
    /// Remove drone from swarm
    RemoveDrone { drone: DroneId },
    /// Emergency stop
    EmergencyStop,
    /// Change formation
    ChangeFormation { formation_type: u8 },
}

/// Consensus messages for Raft protocol
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConsensusMessage {
    /// Request vote from other nodes
    RequestVote {
        term: u64,
        candidate_id: DroneId,
        last_log_index: u64,
        last_log_term: u64,
    },
    /// Vote response
    VoteReply {
        term: u64,
        vote_granted: bool,
        voter_id: DroneId,
    },
    /// Append entries (heartbeat or log replication)
    AppendEntries {
        term: u64,
        leader_id: DroneId,
        prev_log_index: u64,
        prev_log_term: u64,
        entries: Vec<LogEntry, 32>,
        leader_commit: u64,
    },
    /// Append entries response
    AppendEntriesReply {
        term: u64,
        success: bool,
        match_index: u64,
        follower_id: DroneId,
    },
}

/// Raft consensus state machine
pub struct ConsensusEngine {
    /// This node's ID
    node_id: DroneId,
    /// Current state
    state: NodeState,
    /// Current term
    current_term: u64,
    /// Candidate voted for in current term
    voted_for: Option<DroneId>,
    /// Replicated log
    log: Vec<LogEntry, 1000>,
    /// Index of highest log entry known to be committed
    commit_index: u64,
    /// Index of highest log entry applied to state machine
    last_applied: u64,
    /// For leaders: next log index to send to each follower
    next_index: FnvIndexMap<u64, u64, 128>,
    /// For leaders: highest log index known to be replicated on each follower
    match_index: FnvIndexMap<u64, u64, 128>,
    /// Election timeout (randomized)
    election_timeout_ms: u32,
    /// Last heartbeat time
    last_heartbeat: u64,
    /// Election timer
    election_timer: u64,
    /// Current leader
    current_leader: Option<DroneId>,
    /// Vote count during election
    votes_received: u8,
    /// Swarm members
    swarm_members: Vec<DroneId, 100>,
}

impl ConsensusEngine {
    /// Create a new consensus engine
    pub fn new(node_id: DroneId, election_timeout_ms: u32) -> Self {
        Self {
            node_id,
            state: NodeState::Follower,
            current_term: 0,
            voted_for: None,
            log: Vec::new(),
            commit_index: 0,
            last_applied: 0,
            next_index: FnvIndexMap::new(),
            match_index: FnvIndexMap::new(),
            election_timeout_ms,
            last_heartbeat: 0,
            election_timer: 0,
            current_leader: None,
            votes_received: 0,
            swarm_members: Vec::new(),
        }
    }

    /// Process consensus message
    pub fn process_message(&mut self, msg: ConsensusMessage) -> Result<Option<ConsensusMessage>> {
        match msg {
            ConsensusMessage::RequestVote {
                term,
                candidate_id,
                last_log_index,
                last_log_term,
            } => self.handle_request_vote(term, candidate_id, last_log_index, last_log_term),
            ConsensusMessage::VoteReply {
                term,
                vote_granted,
                voter_id,
            } => {
                self.handle_vote_reply(term, vote_granted, voter_id)?;
                Ok(None)
            }
            ConsensusMessage::AppendEntries {
                term,
                leader_id,
                prev_log_index,
                prev_log_term,
                entries,
                leader_commit,
            } => self.handle_append_entries(
                term,
                leader_id,
                prev_log_index,
                prev_log_term,
                entries,
                leader_commit,
            ),
            ConsensusMessage::AppendEntriesReply {
                term,
                success,
                match_index,
                follower_id,
            } => {
                self.handle_append_entries_reply(term, success, match_index, follower_id)?;
                Ok(None)
            }
        }
    }

    /// Propose a new command (leader only)
    pub fn propose_command(&mut self, command: SwarmCommand) -> Result<u64> {
        if self.state != NodeState::Leader {
            return Err(SwarmError::ConsensusError);
        }

        let entry = LogEntry {
            term: self.current_term,
            index: self.log.len() as u64 + 1,
            command,
        };

        let index = entry.index;
        self.log
            .push(entry)
            .map_err(|_| SwarmError::ResourceExhausted)?;

        Ok(index)
    }

    /// Handle election timeout
    pub fn tick(&mut self) -> Result<Vec<ConsensusMessage, 10>> {
        let current_time = Self::get_time();
        let mut messages = Vec::new();

        // BUG-004 FIX: Detect clock going backwards (wrap-around)
        if current_time < self.election_timer {
            // Clock wrapped or went backwards - reset timer
            self.election_timer = current_time;
            return Ok(messages);
        }

        match self.state {
            NodeState::Follower | NodeState::Candidate => {
                // Use saturating subtraction to prevent underflow
                let elapsed = current_time.saturating_sub(self.election_timer);

                if elapsed > self.election_timeout_ms as u64 {
                    // Start election
                    self.start_election()?;

                    // Send vote requests to all members
                    for member in &self.swarm_members {
                        if *member != self.node_id {
                            let msg = ConsensusMessage::RequestVote {
                                term: self.current_term,
                                candidate_id: self.node_id,
                                last_log_index: self.log.len() as u64,
                                last_log_term: self.last_log_term(),
                            };
                            messages.push(msg).map_err(|_| SwarmError::BufferFull)?;
                        }
                    }
                }
            }
            NodeState::Leader => {
                // Use saturating subtraction to prevent underflow
                let heartbeat_elapsed = current_time.saturating_sub(self.last_heartbeat);

                if heartbeat_elapsed > 50 {
                    // 50ms heartbeat
                    // Send heartbeats
                    for member in &self.swarm_members {
                        if *member != self.node_id {
                            let msg = self.create_append_entries(*member)?;
                            messages.push(msg).map_err(|_| SwarmError::BufferFull)?;
                        }
                    }
                    self.last_heartbeat = current_time;
                }
            }
        }

        Ok(messages)
    }

    /// Start election
    fn start_election(&mut self) -> Result<()> {
        self.state = NodeState::Candidate;
        self.current_term += 1;
        self.voted_for = Some(self.node_id);
        self.votes_received = 1; // Vote for self
        self.election_timer = Self::get_time();
        Ok(())
    }

    /// Handle vote request
    fn handle_request_vote(
        &mut self,
        term: u64,
        candidate_id: DroneId,
        last_log_index: u64,
        last_log_term: u64,
    ) -> Result<Option<ConsensusMessage>> {
        if term > self.current_term {
            self.become_follower(term);
        }

        let vote_granted = if term < self.current_term {
            false
        } else if let Some(voted) = self.voted_for {
            voted == candidate_id
        } else {
            // Check if candidate's log is at least as up-to-date
            let my_last_term = self.last_log_term();
            let my_last_index = self.log.len() as u64;

            if last_log_term > my_last_term
                || (last_log_term == my_last_term && last_log_index >= my_last_index)
            {
                self.voted_for = Some(candidate_id);
                true
            } else {
                false
            }
        };

        Ok(Some(ConsensusMessage::VoteReply {
            term: self.current_term,
            vote_granted,
            voter_id: self.node_id,
        }))
    }

    /// Handle vote reply
    fn handle_vote_reply(&mut self, term: u64, vote_granted: bool, _voter_id: DroneId) -> Result<()> {
        if self.state != NodeState::Candidate {
            return Ok(());
        }

        if term > self.current_term {
            self.become_follower(term);
            return Ok(());
        }

        if vote_granted && term == self.current_term {
            self.votes_received += 1;

            // Check if we have majority
            let majority = (self.swarm_members.len() / 2) + 1;
            if self.votes_received >= majority as u8 {
                self.become_leader()?;
            }
        }

        Ok(())
    }

    /// Handle append entries
    fn handle_append_entries(
        &mut self,
        term: u64,
        leader_id: DroneId,
        prev_log_index: u64,
        prev_log_term: u64,
        entries: Vec<LogEntry, 32>,
        leader_commit: u64,
    ) -> Result<Option<ConsensusMessage>> {
        if term > self.current_term {
            self.become_follower(term);
        }

        self.current_leader = Some(leader_id);
        self.election_timer = Self::get_time(); // Reset election timer

        let success = if term < self.current_term {
            false
        } else {
            // Check log consistency
            if prev_log_index > 0 {
                if let Some(entry) = self.log.get(prev_log_index as usize - 1) {
                    if entry.term != prev_log_term {
                        false
                    } else {
                        // Append entries
                        for entry in entries {
                            let index = entry.index as usize;
                            if index <= self.log.len() {
                                // Replace conflicting entry
                                self.log[index - 1] = entry;
                            } else {
                                self.log.push(entry).map_err(|_| SwarmError::ResourceExhausted)?;
                            }
                        }
                        true
                    }
                } else {
                    false
                }
            } else {
                // First entry
                for entry in entries {
                    self.log.push(entry).map_err(|_| SwarmError::ResourceExhausted)?;
                }
                true
            }
        };

        if success {
            // Update commit index
            if leader_commit > self.commit_index {
                self.commit_index = core::cmp::min(leader_commit, self.log.len() as u64);
            }
        }

        Ok(Some(ConsensusMessage::AppendEntriesReply {
            term: self.current_term,
            success,
            match_index: self.log.len() as u64,
            follower_id: self.node_id,
        }))
    }

    /// Handle append entries reply
    fn handle_append_entries_reply(
        &mut self,
        term: u64,
        success: bool,
        match_idx: u64,
        follower_id: DroneId,
    ) -> Result<()> {
        if self.state != NodeState::Leader {
            return Ok(());
        }

        if term > self.current_term {
            self.become_follower(term);
            return Ok(());
        }

        if success {
            self.match_index.insert(follower_id.as_u64(), match_idx).ok();
            self.next_index
                .insert(follower_id.as_u64(), match_idx + 1)
                .ok();

            // Update commit index
            self.update_commit_index()?;
        } else {
            // Decrement next_index and retry
            if let Some(next) = self.next_index.get_mut(&follower_id.as_u64()) {
                if *next > 1 {
                    *next -= 1;
                }
            }
        }

        Ok(())
    }

    /// Become follower
    fn become_follower(&mut self, term: u64) {
        self.state = NodeState::Follower;
        self.current_term = term;
        self.voted_for = None;
        self.current_leader = None;
        self.election_timer = Self::get_time();
    }

    /// Become leader
    fn become_leader(&mut self) -> Result<()> {
        self.state = NodeState::Leader;
        self.current_leader = Some(self.node_id);

        // Initialize next_index and match_index
        for member in &self.swarm_members {
            if *member != self.node_id {
                self.next_index
                    .insert(member.as_u64(), self.log.len() as u64 + 1)
                    .ok();
                self.match_index.insert(member.as_u64(), 0).ok();
            }
        }

        self.last_heartbeat = Self::get_time();
        Ok(())
    }

    /// Create append entries message
    fn create_append_entries(&self, follower: DroneId) -> Result<ConsensusMessage> {
        let next_idx = *self.next_index.get(&follower.as_u64()).unwrap_or(&1);
        let prev_log_index = if next_idx > 1 { next_idx - 1 } else { 0 };
        let prev_log_term = if prev_log_index > 0 {
            self.log.get(prev_log_index as usize - 1).map(|e| e.term).unwrap_or(0)
        } else {
            0
        };

        let entries = Vec::new(); // Heartbeat (no entries)

        Ok(ConsensusMessage::AppendEntries {
            term: self.current_term,
            leader_id: self.node_id,
            prev_log_index,
            prev_log_term,
            entries,
            leader_commit: self.commit_index,
        })
    }

    /// Update commit index (leader only)
    fn update_commit_index(&mut self) -> Result<()> {
        if self.state != NodeState::Leader {
            return Ok(());
        }

        // Find highest index replicated on majority
        for n in (self.commit_index + 1)..=(self.log.len() as u64) {
            let mut count = 1; // Leader has it
            for match_idx in self.match_index.values() {
                if *match_idx >= n {
                    count += 1;
                }
            }

            let majority = (self.swarm_members.len() / 2) + 1;
            if count >= majority {
                self.commit_index = n;
            }
        }

        Ok(())
    }

    /// Get last log term
    fn last_log_term(&self) -> u64 {
        self.log.last().map(|e| e.term).unwrap_or(0)
    }

    /// Add swarm member
    pub fn add_member(&mut self, drone_id: DroneId) -> Result<()> {
        if !self.swarm_members.contains(&drone_id) {
            self.swarm_members
                .push(drone_id)
                .map_err(|_| SwarmError::SwarmSizeExceeded)?;
        }
        Ok(())
    }

    /// Get current state
    pub fn state(&self) -> NodeState {
        self.state
    }

    /// Get current leader
    pub fn leader(&self) -> Option<DroneId> {
        self.current_leader
    }

    /// Get current time (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_consensus_creation() {
        let engine = ConsensusEngine::new(DroneId::new(1), 150);
        assert_eq!(engine.state(), NodeState::Follower);
    }

    #[test]
    fn test_propose_command() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);
        engine.state = NodeState::Leader; // Manually set to leader for test

        let cmd = SwarmCommand::AssignTask {
            drone: DroneId::new(2),
            task_id: 42,
        };

        let result = engine.propose_command(cmd);
        assert!(result.is_ok());
    }
}
