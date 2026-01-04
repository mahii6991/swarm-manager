//! Federated Learning Coordination for Swarm Intelligence
//!
//! Implements distributed machine learning without central server:
//! - Decentralized model training
//! - Secure model aggregation
//! - Byzantine fault tolerance
//! - Privacy-preserving gradient sharing
//! - Blockchain-based verification (concept)

use crate::crypto::KeyStore;
use crate::types::*;
use chacha20poly1305::{
    aead::{AeadInPlace, KeyInit},
    ChaCha20Poly1305, Nonce, Tag,
};
use ed25519_dalek::{Signature, Verifier};
use heapless::{FnvIndexMap, Vec};
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

/// Maximum model parameters (simplified)
pub const MAX_MODEL_PARAMS: usize = 1000;

/// Federated learning round
pub type Round = u64;

/// Model update from a participant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelUpdate {
    /// Participant drone ID
    pub drone_id: DroneId,
    /// Training round
    pub round: Round,
    /// Model parameters (simplified as f32 array)
    pub parameters: Vec<f32, MAX_MODEL_PARAMS>,
    /// Number of training samples used
    pub sample_count: u32,
    /// Loss metric
    pub loss: f32,
    /// Digital signature for verification
    #[serde(with = "BigArray")]
    pub signature: [u8; 64],
}

/// Aggregated global model
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GlobalModel {
    /// Current round
    pub round: Round,
    /// Aggregated parameters
    pub parameters: Vec<f32, MAX_MODEL_PARAMS>,
    /// Number of contributors
    pub contributor_count: u32,
    /// Model accuracy (if known)
    pub accuracy: f32,
}

impl GlobalModel {
    /// Create initial model
    pub fn new(param_count: usize) -> Result<Self> {
        let mut params = Vec::new();
        for _ in 0..param_count {
            params.push(0.0).map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(Self {
            round: 0,
            parameters: params,
            contributor_count: 0,
            accuracy: 0.0,
        })
    }
}

/// Federated learning coordinator
pub struct FederatedCoordinator {
    /// This drone's ID (reserved for multi-node coordination)
    #[allow(dead_code)]
    node_id: DroneId,
    /// Current global model
    global_model: GlobalModel,
    /// Pending updates for current round
    pending_updates: Vec<ModelUpdate, 100>,
    /// Current round
    current_round: Round,
    /// Minimum participants for aggregation
    min_participants: u32,
    /// Byzantine fault tolerance (BFT) enabled
    bft_enabled: bool,
    /// Update history for verification
    update_history: FnvIndexMap<u64, Vec<Round, 100>, 128>,
    /// Key store for signature verification (BUG-005 FIX)
    key_store: KeyStore,
}

impl FederatedCoordinator {
    /// Create a new federated learning coordinator
    pub fn new(node_id: DroneId, initial_model: GlobalModel, key_store: KeyStore) -> Self {
        Self {
            node_id,
            global_model: initial_model,
            pending_updates: Vec::new(),
            current_round: 0,
            min_participants: 3,
            bft_enabled: true,
            update_history: FnvIndexMap::new(),
            key_store,
        }
    }

    /// Submit local model update
    pub fn submit_update(&mut self, update: ModelUpdate) -> Result<()> {
        // Verify update is for current round
        if update.round != self.current_round {
            return Err(SwarmError::InvalidMessage);
        }

        // BUG-005 FIX: Verify signature
        let public_key = self.key_store.get_key(update.drone_id)?;

        // Serialize update data (without signature)
        let mut update_data = Vec::<u8, 2048>::new();
        update_data
            .extend_from_slice(&update.round.to_le_bytes())
            .map_err(|_| SwarmError::BufferFull)?;
        for &param in &update.parameters {
            update_data
                .extend_from_slice(&param.to_le_bytes())
                .map_err(|_| SwarmError::BufferFull)?;
        }
        update_data
            .extend_from_slice(&update.sample_count.to_le_bytes())
            .map_err(|_| SwarmError::BufferFull)?;
        update_data
            .extend_from_slice(&update.loss.to_le_bytes())
            .map_err(|_| SwarmError::BufferFull)?;

        // Verify signature
        let signature = Signature::from_bytes(&update.signature);
        public_key
            .verify(&update_data, &signature)
            .map_err(|_| SwarmError::AuthenticationFailed)?;

        // Check for duplicate submission
        if self
            .pending_updates
            .iter()
            .any(|u| u.drone_id == update.drone_id)
        {
            return Err(SwarmError::InvalidMessage);
        }

        // Byzantine detection: check if update is suspiciously different
        if self.bft_enabled && !self.is_update_valid(&update) {
            return Err(SwarmError::AuthenticationFailed);
        }

        // Record update
        self.pending_updates
            .push(update.clone())
            .map_err(|_| SwarmError::BufferFull)?;

        // Record in history
        use heapless::Entry;
        let history = match self.update_history.entry(update.drone_id.as_u64()) {
            Entry::Occupied(o) => o.into_mut(),
            Entry::Vacant(v) => v
                .insert(Vec::new())
                .map_err(|_| SwarmError::ResourceExhausted)?,
        };
        // Propagate history tracking errors - important for Byzantine detection
        history
            .push(update.round)
            .map_err(|_| SwarmError::ResourceExhausted)?;

        Ok(())
    }

    /// Aggregate model updates using Federated Averaging (FedAvg)
    pub fn aggregate_updates(&mut self) -> Result<()> {
        if (self.pending_updates.len() as u32) < self.min_participants {
            return Err(SwarmError::ConsensusError);
        }

        // Federated Averaging algorithm
        let param_count = self.global_model.parameters.len();
        let mut aggregated = vec![0.0f32; param_count];
        let mut total_samples = 0u32;

        // Weighted average based on sample count
        for update in &self.pending_updates {
            total_samples += update.sample_count;
            for (i, &param) in update.parameters.iter().enumerate() {
                aggregated[i] += param * (update.sample_count as f32);
            }
        }

        // BUG-006 FIX: Check before division to prevent division by zero
        if total_samples == 0 {
            return Err(SwarmError::InvalidMessage); // All updates have 0 samples
        }

        // Normalize by total samples
        let total_samples_f32 = total_samples as f32;
        for param in &mut aggregated {
            *param /= total_samples_f32; // Safe division
        }

        // Update global model
        self.global_model.parameters.clear();
        for param in aggregated {
            self.global_model
                .parameters
                .push(param)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        self.global_model.round = self.current_round;
        self.global_model.contributor_count = self.pending_updates.len() as u32;

        // Clear pending updates
        self.pending_updates.clear();

        // Move to next round
        self.current_round += 1;

        Ok(())
    }

    /// Byzantine fault detection: check if update is valid
    fn is_update_valid(&self, update: &ModelUpdate) -> bool {
        if self.pending_updates.len() < 2 {
            return true; // Not enough data for comparison
        }

        // Calculate median parameter values from existing updates
        let param_count = update.parameters.len();
        let mut deviations = 0;

        for i in 0..param_count {
            let mut values: Vec<f32, 100> = Vec::new();
            for existing in &self.pending_updates {
                if let Some(&val) = existing.parameters.get(i) {
                    // If we can't collect all values, stop early but continue with what we have
                    // This is acceptable for Byzantine detection (statistical comparison)
                    if values.push(val).is_err() {
                        break;
                    }
                }
            }

            if values.is_empty() {
                continue;
            }

            // Simple median calculation
            values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            let median = values[values.len() / 2];

            // Check if update deviates too much from median
            if let Some(&val) = update.parameters.get(i) {
                let deviation = libm::fabsf(val - median);
                let threshold = libm::fabsf(median) * 2.0 + 1.0; // 200% + epsilon

                if deviation > threshold {
                    deviations += 1;
                }
            }
        }

        // Reject if too many parameters deviate
        let deviation_ratio = deviations as f32 / param_count as f32;
        deviation_ratio < 0.3 // Allow up to 30% deviation
    }

    /// Get current global model
    pub fn global_model(&self) -> &GlobalModel {
        &self.global_model
    }

    /// Get current round
    pub fn current_round(&self) -> Round {
        self.current_round
    }

    /// Check if ready for aggregation
    pub fn is_ready_for_aggregation(&self) -> bool {
        (self.pending_updates.len() as u32) >= self.min_participants
    }

    /// Get pending update count
    pub fn pending_count(&self) -> usize {
        self.pending_updates.len()
    }

    /// Set minimum participants
    pub fn set_min_participants(&mut self, min: u32) {
        self.min_participants = min;
    }

    /// Enable/disable Byzantine fault tolerance
    pub fn set_bft_enabled(&mut self, enabled: bool) {
        self.bft_enabled = enabled;
    }
}

/// Local model trainer (simplified interface)
pub struct LocalTrainer {
    /// Drone ID
    drone_id: DroneId,
    /// Current model parameters
    parameters: Vec<f32, MAX_MODEL_PARAMS>,
    /// Training data count
    sample_count: u32,
}

impl LocalTrainer {
    /// Create a new local trainer
    pub fn new(drone_id: DroneId, initial_params: Vec<f32, MAX_MODEL_PARAMS>) -> Self {
        Self {
            drone_id,
            parameters: initial_params,
            sample_count: 0,
        }
    }

    /// Train on local data (simplified)
    pub fn train_step(&mut self, learning_rate: f32) -> Result<f32> {
        // Simplified training: apply random gradient update
        // In production, this would be actual ML training

        for param in &mut self.parameters {
            let gradient = Self::compute_gradient(*param);
            *param -= learning_rate * gradient;
        }

        self.sample_count += 1;

        // Return loss (simplified)
        Ok(self.compute_loss())
    }

    /// Compute gradient (placeholder)
    fn compute_gradient(param: f32) -> f32 {
        // Simplified: gradient = param * 0.1
        param * 0.1
    }

    /// Compute loss (placeholder)
    fn compute_loss(&self) -> f32 {
        // Simplified loss calculation
        let mut loss = 0.0f32;
        for &param in &self.parameters {
            loss += param * param;
        }
        loss / self.parameters.len() as f32
    }

    /// Create model update for submission
    pub fn create_update(&self, round: Round) -> Result<ModelUpdate> {
        Ok(ModelUpdate {
            drone_id: self.drone_id,
            round,
            parameters: self.parameters.clone(),
            sample_count: self.sample_count,
            loss: self.compute_loss(),
            signature: [0u8; 64], // Should be computed using crypto module
        })
    }

    /// Update local model with global parameters
    pub fn update_from_global(&mut self, global: &GlobalModel) {
        self.parameters = global.parameters.clone();
    }

    /// Get current parameters
    pub fn parameters(&self) -> &Vec<f32, MAX_MODEL_PARAMS> {
        &self.parameters
    }
}

/// Federated learning message types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FederatedMessage {
    /// Model update submission
    SubmitUpdate(ModelUpdate),
    /// Request global model
    RequestGlobalModel { round: Round },
    /// Global model response
    GlobalModelResponse(GlobalModel),
    /// Start new training round
    StartRound { round: Round },
    /// Round completion notification
    RoundComplete { round: Round },
}

/// Secure aggregation protocol (simplified)
/// In production, use secure multi-party computation
pub struct SecureAggregation {
    /// Shared secrets for secure aggregation
    secrets: FnvIndexMap<u64, [u8; 32], 128>,
}

impl SecureAggregation {
    /// Create new secure aggregation
    pub fn new() -> Self {
        Self {
            secrets: FnvIndexMap::new(),
        }
    }

    /// Add participant with shared secret
    pub fn add_participant(&mut self, drone_id: DroneId, secret: [u8; 32]) -> Result<()> {
        self.secrets
            .insert(drone_id.as_u64(), secret)
            .map_err(|_| SwarmError::ResourceExhausted)?;
        Ok(())
    }

    /// Encrypt model update for secure aggregation
    pub fn encrypt_update(&self, update: &ModelUpdate) -> Result<Vec<u8, 2048>> {
        let secret = self
            .secrets
            .get(&update.drone_id.as_u64())
            .ok_or(SwarmError::AuthenticationFailed)?;

        let cipher =
            ChaCha20Poly1305::new_from_slice(secret).map_err(|_| SwarmError::CryptoError)?;

        // Use round number for nonce
        let mut nonce_bytes = [0u8; 12];
        nonce_bytes[..8].copy_from_slice(&update.round.to_le_bytes());
        let nonce = Nonce::from_slice(&nonce_bytes);

        // Serialize parameters into buffer
        let mut buffer = Vec::<u8, 2048>::new();
        for &param in &update.parameters {
            buffer
                .extend_from_slice(&param.to_le_bytes())
                .map_err(|_| SwarmError::BufferFull)?;
        }

        // Encrypt in place
        let tag = cipher
            .encrypt_in_place_detached(nonce, &[], &mut buffer)
            .map_err(|_| SwarmError::CryptoError)?;

        // Append tag
        buffer
            .extend_from_slice(&tag)
            .map_err(|_| SwarmError::BufferFull)?;

        Ok(buffer)
    }

    /// Aggregate encrypted updates
    pub fn aggregate_encrypted(
        &self,
        updates: &[(DroneId, Vec<u8, 2048>)],
    ) -> Result<Vec<f32, MAX_MODEL_PARAMS>> {
        if updates.is_empty() {
            return Err(SwarmError::InvalidMessage);
        }

        let mut summed_params = [0.0f32; MAX_MODEL_PARAMS];
        let mut valid_count = 0;
        let param_count = MAX_MODEL_PARAMS;

        let nonce_bytes = [0u8; 12];
        let nonce = Nonce::from_slice(&nonce_bytes);

        for (drone_id, encrypted_data) in updates {
            if let Some(secret) = self.secrets.get(&drone_id.as_u64()) {
                if let Ok(cipher) = ChaCha20Poly1305::new_from_slice(secret) {
                    // Split into ciphertext and tag
                    // encrypted_data = ciphertext || tag (16 bytes)
                    if encrypted_data.len() < 16 {
                        continue;
                    }

                    let tag_pos = encrypted_data.len() - 16;
                    let tag_bytes = &encrypted_data[tag_pos..];
                    let tag = Tag::from_slice(tag_bytes);

                    // We need a mutable buffer for in-place decryption
                    let mut buffer = Vec::<u8, 2048>::new();
                    if buffer
                        .extend_from_slice(&encrypted_data[..tag_pos])
                        .is_err()
                    {
                        continue;
                    }

                    if cipher
                        .decrypt_in_place_detached(nonce, &[], &mut buffer, tag)
                        .is_ok()
                    {
                        let mut idx = 0;
                        for chunk in buffer.chunks(4) {
                            if idx >= param_count {
                                break;
                            }
                            if let Ok(bytes) = chunk.try_into() {
                                let val = f32::from_le_bytes(bytes);
                                summed_params[idx] += val;
                                idx += 1;
                            }
                        }
                        if idx > 0 {
                            valid_count += 1;
                        }
                    }
                }
            }
        }

        if valid_count == 0 {
            return Err(SwarmError::AuthenticationFailed);
        }

        let mut result = Vec::new();
        for param in summed_params.iter().take(param_count) {
            result
                .push(*param / valid_count as f32)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(result)
    }
}

impl Default for SecureAggregation {
    fn default() -> Self {
        Self::new()
    }
}
