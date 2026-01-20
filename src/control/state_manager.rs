//! State Management for Swarm Controller
//!
//! Manages local and peer drone states.

use crate::types::*;
use heapless::FnvIndexMap;

/// Maximum number of peers to track
pub const MAX_PEERS: usize = 128;

/// Manages state for local drone and peers
#[derive(Debug, Clone)]
pub struct StateManager {
    /// Local drone state
    local: DroneState,
    /// States of peer drones
    peers: FnvIndexMap<u64, DroneState, MAX_PEERS>,
    /// Stale timeout (ms) - peers older than this are considered stale
    stale_timeout_ms: u64,
}

impl StateManager {
    /// Create new state manager
    pub fn new(drone_id: DroneId, initial_position: Position) -> Self {
        Self {
            local: DroneState {
                id: drone_id,
                position: initial_position,
                velocity: Velocity {
                    vx: 0.0,
                    vy: 0.0,
                    vz: 0.0,
                },
                battery: 100,
                status: MissionStatus::Idle,
                timestamp: 0,
            },
            peers: FnvIndexMap::new(),
            stale_timeout_ms: 5000, // 5 seconds default
        }
    }

    /// Update local state
    pub fn update_local(
        &mut self,
        position: Position,
        velocity: Velocity,
        battery: u8,
        status: MissionStatus,
        timestamp: u64,
    ) {
        self.local.position = position;
        self.local.velocity = velocity;
        self.local.battery = battery;
        self.local.status = status;
        self.local.timestamp = timestamp;
    }

    /// Update peer state
    pub fn update_peer(&mut self, state: DroneState) -> Result<()> {
        self.peers
            .insert(state.id.as_u64(), state)
            .map_err(|_| SwarmError::ResourceExhausted)?;
        Ok(())
    }

    /// Remove a peer
    pub fn remove_peer(&mut self, drone_id: DroneId) -> Option<DroneState> {
        self.peers.remove(&drone_id.as_u64())
    }

    /// Get local state reference
    pub fn local(&self) -> &DroneState {
        &self.local
    }

    /// Get local state mutable reference
    pub fn local_mut(&mut self) -> &mut DroneState {
        &mut self.local
    }

    /// Get peer state
    pub fn peer(&self, drone_id: DroneId) -> Option<&DroneState> {
        self.peers.get(&drone_id.as_u64())
    }

    /// Get all peer states
    pub fn peers(&self) -> impl Iterator<Item = &DroneState> {
        self.peers.values()
    }

    /// Get total swarm size (local + peers)
    pub fn swarm_size(&self) -> usize {
        self.peers.len() + 1
    }

    /// Compute swarm centroid (center of mass)
    pub fn compute_centroid(&self) -> Position {
        let mut sum_x = self.local.position.x;
        let mut sum_y = self.local.position.y;
        let mut sum_z = self.local.position.z;
        let count = self.peers.len() + 1;

        for state in self.peers.values() {
            sum_x += state.position.x;
            sum_y += state.position.y;
            sum_z += state.position.z;
        }

        Position {
            x: sum_x / count as f32,
            y: sum_y / count as f32,
            z: sum_z / count as f32,
        }
    }

    /// Remove stale peers based on timestamp
    pub fn remove_stale_peers(&mut self, current_time_ms: u64) -> usize {
        let stale_threshold = current_time_ms.saturating_sub(self.stale_timeout_ms);
        let mut stale_ids: heapless::Vec<u64, MAX_PEERS> = heapless::Vec::new();

        for (&id, state) in self.peers.iter() {
            if state.timestamp < stale_threshold {
                stale_ids.push(id).ok();
            }
        }

        let count = stale_ids.len();
        for id in stale_ids {
            self.peers.remove(&id);
        }
        count
    }

    /// Set stale timeout
    pub fn set_stale_timeout(&mut self, timeout_ms: u64) {
        self.stale_timeout_ms = timeout_ms;
    }

    /// Get peer count
    pub fn peer_count(&self) -> usize {
        self.peers.len()
    }

    /// Check if a peer exists
    pub fn has_peer(&self, drone_id: DroneId) -> bool {
        self.peers.contains_key(&drone_id.as_u64())
    }

    /// Get drone ID
    pub fn drone_id(&self) -> DroneId {
        self.local.id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_manager_creation() {
        let pos = Position { x: 0.0, y: 0.0, z: 10.0 };
        let manager = StateManager::new(DroneId::new(1), pos);
        assert_eq!(manager.swarm_size(), 1);
        assert_eq!(manager.drone_id(), DroneId::new(1));
    }

    #[test]
    fn test_peer_management() {
        let pos = Position { x: 0.0, y: 0.0, z: 10.0 };
        let mut manager = StateManager::new(DroneId::new(1), pos);

        let peer_state = DroneState {
            id: DroneId::new(2),
            position: Position { x: 10.0, y: 0.0, z: 10.0 },
            velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
            battery: 90,
            status: MissionStatus::Idle,
            timestamp: 1000,
        };

        manager.update_peer(peer_state).unwrap();
        assert_eq!(manager.swarm_size(), 2);
        assert!(manager.has_peer(DroneId::new(2)));
    }

    #[test]
    fn test_centroid_calculation() {
        let pos = Position { x: 0.0, y: 0.0, z: 10.0 };
        let mut manager = StateManager::new(DroneId::new(1), pos);

        let peer_state = DroneState {
            id: DroneId::new(2),
            position: Position { x: 10.0, y: 10.0, z: 10.0 },
            velocity: Velocity { vx: 0.0, vy: 0.0, vz: 0.0 },
            battery: 90,
            status: MissionStatus::Idle,
            timestamp: 1000,
        };
        manager.update_peer(peer_state).unwrap();

        let centroid = manager.compute_centroid();
        assert_eq!(centroid.x, 5.0);
        assert_eq!(centroid.y, 5.0);
    }
}
