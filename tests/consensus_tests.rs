//! Comprehensive tests for Raft-based consensus
//!
//! Tests consensus protocol, leader election, log replication, and message handling

use drone_swarm_system::consensus::*;
use drone_swarm_system::types::*;
use heapless::Vec;

#[cfg(test)]
mod node_state_tests {
    use super::*;

    #[test]
    fn test_node_state_equality() {
        assert_eq!(NodeState::Follower, NodeState::Follower);
        assert_eq!(NodeState::Candidate, NodeState::Candidate);
        assert_eq!(NodeState::Leader, NodeState::Leader);

        assert_ne!(NodeState::Follower, NodeState::Candidate);
        assert_ne!(NodeState::Candidate, NodeState::Leader);
        assert_ne!(NodeState::Leader, NodeState::Follower);
    }

    #[test]
    fn test_node_state_clone() {
        let state1 = NodeState::Leader;
        let state2 = state1.clone();

        assert_eq!(state1, state2);
    }

    #[test]
    fn test_node_state_copy() {
        let state1 = NodeState::Candidate;
        let state2 = state1; // Copy semantics

        assert_eq!(state1, state2);
    }
}

#[cfg(test)]
mod swarm_command_tests {
    use super::*;

    #[test]
    fn test_swarm_command_assign_task() {
        let cmd = SwarmCommand::AssignTask {
            drone: DroneId::new(1),
            task_id: 42,
        };

        match cmd {
            SwarmCommand::AssignTask { drone, task_id } => {
                assert_eq!(drone, DroneId::new(1));
                assert_eq!(task_id, 42);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_update_mission() {
        let mut params: Vec<u8, 256> = Vec::new();
        params.push(1).unwrap();
        params.push(2).unwrap();

        let cmd = SwarmCommand::UpdateMission {
            params: params.clone(),
        };

        match cmd {
            SwarmCommand::UpdateMission { params: p } => {
                assert_eq!(p.len(), 2);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_add_drone() {
        let cmd = SwarmCommand::AddDrone {
            drone: DroneId::new(5),
        };

        match cmd {
            SwarmCommand::AddDrone { drone } => {
                assert_eq!(drone, DroneId::new(5));
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_remove_drone() {
        let cmd = SwarmCommand::RemoveDrone {
            drone: DroneId::new(3),
        };

        match cmd {
            SwarmCommand::RemoveDrone { drone } => {
                assert_eq!(drone, DroneId::new(3));
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_emergency_stop() {
        let cmd = SwarmCommand::EmergencyStop;

        match cmd {
            SwarmCommand::EmergencyStop => {
                // Success
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_change_formation() {
        let cmd = SwarmCommand::ChangeFormation { formation_type: 3 };

        match cmd {
            SwarmCommand::ChangeFormation { formation_type } => {
                assert_eq!(formation_type, 3);
            }
            _ => panic!("Wrong command type"),
        }
    }

    #[test]
    fn test_swarm_command_clone() {
        let cmd1 = SwarmCommand::EmergencyStop;
        let cmd2 = cmd1.clone();

        match (cmd1, cmd2) {
            (SwarmCommand::EmergencyStop, SwarmCommand::EmergencyStop) => {
                // Success
            }
            _ => panic!("Clone failed"),
        }
    }
}

#[cfg(test)]
mod log_entry_tests {
    use super::*;

    #[test]
    fn test_log_entry_creation() {
        let entry = LogEntry {
            term: 5,
            index: 10,
            command: SwarmCommand::EmergencyStop,
        };

        assert_eq!(entry.term, 5);
        assert_eq!(entry.index, 10);
    }

    #[test]
    fn test_log_entry_clone() {
        let entry1 = LogEntry {
            term: 3,
            index: 7,
            command: SwarmCommand::AddDrone {
                drone: DroneId::new(2),
            },
        };

        let entry2 = entry1.clone();

        assert_eq!(entry1.term, entry2.term);
        assert_eq!(entry1.index, entry2.index);
    }
}

#[cfg(test)]
mod consensus_message_tests {
    use super::*;

    #[test]
    fn test_consensus_message_request_vote() {
        let msg = ConsensusMessage::RequestVote {
            term: 5,
            candidate_id: DroneId::new(1),
            last_log_index: 10,
            last_log_term: 4,
        };

        match msg {
            ConsensusMessage::RequestVote {
                term,
                candidate_id,
                last_log_index,
                last_log_term,
            } => {
                assert_eq!(term, 5);
                assert_eq!(candidate_id, DroneId::new(1));
                assert_eq!(last_log_index, 10);
                assert_eq!(last_log_term, 4);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_consensus_message_vote_reply() {
        let msg = ConsensusMessage::VoteReply {
            term: 5,
            vote_granted: true,
            voter_id: DroneId::new(2),
        };

        match msg {
            ConsensusMessage::VoteReply {
                term,
                vote_granted,
                voter_id,
            } => {
                assert_eq!(term, 5);
                assert!(vote_granted);
                assert_eq!(voter_id, DroneId::new(2));
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_consensus_message_append_entries() {
        let entries: Vec<LogEntry, 32> = Vec::new();

        let msg = ConsensusMessage::AppendEntries {
            term: 5,
            leader_id: DroneId::new(1),
            prev_log_index: 9,
            prev_log_term: 4,
            entries,
            leader_commit: 8,
        };

        match msg {
            ConsensusMessage::AppendEntries {
                term,
                leader_id,
                prev_log_index,
                prev_log_term,
                entries,
                leader_commit,
            } => {
                assert_eq!(term, 5);
                assert_eq!(leader_id, DroneId::new(1));
                assert_eq!(prev_log_index, 9);
                assert_eq!(prev_log_term, 4);
                assert_eq!(entries.len(), 0);
                assert_eq!(leader_commit, 8);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_consensus_message_append_entries_reply() {
        let msg = ConsensusMessage::AppendEntriesReply {
            term: 5,
            success: true,
            match_index: 10,
            follower_id: DroneId::new(3),
        };

        match msg {
            ConsensusMessage::AppendEntriesReply {
                term,
                success,
                match_index,
                follower_id,
            } => {
                assert_eq!(term, 5);
                assert!(success);
                assert_eq!(match_index, 10);
                assert_eq!(follower_id, DroneId::new(3));
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_consensus_message_clone() {
        let msg1 = ConsensusMessage::VoteReply {
            term: 10,
            vote_granted: false,
            voter_id: DroneId::new(5),
        };

        let msg2 = msg1.clone();

        match (msg1, msg2) {
            (
                ConsensusMessage::VoteReply {
                    term: t1,
                    vote_granted: v1,
                    voter_id: id1,
                },
                ConsensusMessage::VoteReply {
                    term: t2,
                    vote_granted: v2,
                    voter_id: id2,
                },
            ) => {
                assert_eq!(t1, t2);
                assert_eq!(v1, v2);
                assert_eq!(id1, id2);
            }
            _ => panic!("Clone failed"),
        }
    }
}

#[cfg(test)]
mod consensus_engine_tests {
    use super::*;

    #[test]
    fn test_consensus_engine_new() {
        let node_id = DroneId::new(1);
        let engine = ConsensusEngine::new(node_id, 150);

        assert_eq!(engine.state(), NodeState::Follower);
        assert_eq!(engine.leader(), None);
    }

    #[test]
    fn test_consensus_engine_initial_state_follower() {
        let engine = ConsensusEngine::new(DroneId::new(1), 200);

        assert_eq!(engine.state(), NodeState::Follower);
    }

    #[test]
    fn test_consensus_engine_initial_leader_none() {
        let engine = ConsensusEngine::new(DroneId::new(1), 200);

        assert_eq!(engine.leader(), None);
    }

    #[test]
    fn test_consensus_engine_add_member() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let result = engine.add_member(DroneId::new(2));
        assert!(result.is_ok());

        let result = engine.add_member(DroneId::new(3));
        assert!(result.is_ok());
    }

    #[test]
    fn test_consensus_engine_add_duplicate_member() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        engine.add_member(DroneId::new(2)).unwrap();

        // Adding same member again
        let result = engine.add_member(DroneId::new(2));
        assert!(result.is_ok()); // Should succeed (idempotent)
    }

    #[test]
    fn test_consensus_engine_propose_command() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let cmd = SwarmCommand::EmergencyStop;
        let result = engine.propose_command(cmd);

        // Will fail because we're not leader
        assert!(result.is_err());
    }

    #[test]
    fn test_consensus_engine_tick_initial() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let result = engine.tick();

        // Should succeed
        assert!(result.is_ok());
    }

    #[test]
    fn test_consensus_engine_process_vote_request() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let msg = ConsensusMessage::RequestVote {
            term: 1,
            candidate_id: DroneId::new(2),
            last_log_index: 0,
            last_log_term: 0,
        };

        let result = engine.process_message(msg);

        // Should process successfully
        assert!(result.is_ok());
    }

    #[test]
    fn test_consensus_engine_process_vote_reply() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let msg = ConsensusMessage::VoteReply {
            term: 1,
            vote_granted: true,
            voter_id: DroneId::new(2),
        };

        let result = engine.process_message(msg);

        // Should process successfully
        assert!(result.is_ok());
    }

    #[test]
    fn test_consensus_engine_process_append_entries_heartbeat() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let entries: Vec<LogEntry, 32> = Vec::new();

        let msg = ConsensusMessage::AppendEntries {
            term: 1,
            leader_id: DroneId::new(2),
            prev_log_index: 0,
            prev_log_term: 0,
            entries,
            leader_commit: 0,
        };

        let result = engine.process_message(msg);

        // Should process successfully
        assert!(result.is_ok());

        // Should recognize leader
        assert_eq!(engine.leader(), Some(DroneId::new(2)));
    }

    #[test]
    fn test_consensus_engine_process_append_entries_reply() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        let msg = ConsensusMessage::AppendEntriesReply {
            term: 1,
            success: true,
            match_index: 0,
            follower_id: DroneId::new(2),
        };

        let result = engine.process_message(msg);

        // Should process successfully
        assert!(result.is_ok());
    }

    #[test]
    fn test_consensus_engine_state_transitions() {
        let engine = ConsensusEngine::new(DroneId::new(1), 150);

        // Initial state should be Follower
        assert_eq!(engine.state(), NodeState::Follower);

        // After tick() with timeout, might transition to Candidate
        // (This is timing-dependent, so we just verify state() works)
    }

    #[test]
    fn test_consensus_engine_multiple_members() {
        let mut engine = ConsensusEngine::new(DroneId::new(1), 150);

        // Add multiple members
        for i in 2..=5 {
            engine.add_member(DroneId::new(i)).unwrap();
        }

        // Engine should track all members
        // (We can't directly check the internal list, but this exercises the code)
    }
}
