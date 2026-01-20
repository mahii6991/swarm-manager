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

/// Loss function types for training
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LossFunction {
    /// Mean Squared Error - for regression
    MSE,
    /// Binary Cross-Entropy - for binary classification
    BinaryCrossEntropy,
    /// Categorical Cross-Entropy - for multi-class classification
    CategoricalCrossEntropy,
    /// Huber loss - robust to outliers
    Huber { delta: f32 },
}

/// Training configuration
#[derive(Debug, Clone)]
pub struct TrainingConfig {
    /// Learning rate
    pub learning_rate: f32,
    /// Momentum coefficient (0 = no momentum)
    pub momentum: f32,
    /// L2 regularization coefficient (weight decay)
    pub weight_decay: f32,
    /// Mini-batch size
    pub batch_size: usize,
    /// Loss function to use
    pub loss_function: LossFunction,
    /// Gradient clipping threshold (0 = no clipping)
    pub gradient_clip: f32,
}

impl Default for TrainingConfig {
    fn default() -> Self {
        Self {
            learning_rate: 0.01,
            momentum: 0.9,
            weight_decay: 0.0001,
            batch_size: 32,
            loss_function: LossFunction::MSE,
            gradient_clip: 1.0,
        }
    }
}

/// Maximum training samples to cache
pub const MAX_TRAINING_SAMPLES: usize = 256;

/// Training sample (input-output pair)
#[derive(Debug, Clone)]
pub struct TrainingSample {
    /// Input features
    pub input: Vec<f32, 64>,
    /// Target output
    pub target: Vec<f32, 16>,
}

/// Local model trainer with proper gradient computation
pub struct LocalTrainer {
    /// Drone ID
    drone_id: DroneId,
    /// Current model parameters (weights)
    parameters: Vec<f32, MAX_MODEL_PARAMS>,
    /// Momentum velocities for SGD with momentum
    velocities: Vec<f32, MAX_MODEL_PARAMS>,
    /// Accumulated gradients for current batch
    gradients: Vec<f32, MAX_MODEL_PARAMS>,
    /// Training configuration
    config: TrainingConfig,
    /// Training data buffer
    training_data: Vec<TrainingSample, MAX_TRAINING_SAMPLES>,
    /// Number of samples processed this round
    sample_count: u32,
    /// Current batch index
    batch_index: usize,
    /// Running loss for current epoch
    epoch_loss: f32,
    /// Number of batches in current epoch
    batch_count: u32,
}

impl LocalTrainer {
    /// Create a new local trainer
    pub fn new(drone_id: DroneId, initial_params: Vec<f32, MAX_MODEL_PARAMS>) -> Self {
        let param_len = initial_params.len();
        let mut velocities = Vec::new();
        let mut gradients = Vec::new();

        for _ in 0..param_len {
            let _ = velocities.push(0.0);
            let _ = gradients.push(0.0);
        }

        Self {
            drone_id,
            parameters: initial_params,
            velocities,
            gradients,
            config: TrainingConfig::default(),
            training_data: Vec::new(),
            sample_count: 0,
            batch_index: 0,
            epoch_loss: 0.0,
            batch_count: 0,
        }
    }

    /// Create a new local trainer with custom configuration
    pub fn with_config(
        drone_id: DroneId,
        initial_params: Vec<f32, MAX_MODEL_PARAMS>,
        config: TrainingConfig,
    ) -> Self {
        let mut trainer = Self::new(drone_id, initial_params);
        trainer.config = config;
        trainer
    }

    /// Add training sample to the local dataset
    pub fn add_sample(&mut self, sample: TrainingSample) -> Result<()> {
        self.training_data
            .push(sample)
            .map_err(|_| SwarmError::BufferFull)
    }

    /// Clear training data
    pub fn clear_training_data(&mut self) {
        self.training_data.clear();
        self.batch_index = 0;
    }

    /// Train one mini-batch using SGD with momentum
    pub fn train_batch(&mut self) -> Result<f32> {
        if self.training_data.is_empty() {
            return Err(SwarmError::InvalidParameter);
        }

        let batch_size = self.config.batch_size.min(self.training_data.len());
        let start_idx = self.batch_index % self.training_data.len();
        let end_idx = (start_idx + batch_size).min(self.training_data.len());

        // Zero gradients
        for grad in &mut self.gradients {
            *grad = 0.0;
        }

        // Compute gradients for mini-batch
        let mut batch_loss = 0.0f32;
        let actual_batch_size = end_idx - start_idx;

        for i in start_idx..end_idx {
            // Clone input to avoid borrow conflict with backward()
            let input = self.training_data[i].input.clone();
            let target = self.training_data[i].target.clone();

            // Forward pass (simple linear model: y = Wx)
            let prediction = self.forward(&input);

            // Compute loss and gradients
            let (loss, output_grad) = self.compute_loss_and_gradient(&prediction, &target);
            batch_loss += loss;

            // Backward pass (accumulate gradients)
            self.backward(&input, &output_grad)?;
        }

        // Average gradients over batch
        if actual_batch_size > 0 {
            batch_loss /= actual_batch_size as f32;
            for grad in &mut self.gradients {
                *grad /= actual_batch_size as f32;
            }
        }

        // Apply gradient clipping
        if self.config.gradient_clip > 0.0 {
            self.clip_gradients();
        }

        // Update parameters using SGD with momentum
        self.update_parameters();

        // Update tracking
        self.batch_index = end_idx;
        if self.batch_index >= self.training_data.len() {
            self.batch_index = 0;
        }
        self.sample_count += actual_batch_size as u32;
        self.epoch_loss += batch_loss;
        self.batch_count += 1;

        Ok(batch_loss)
    }

    /// Train for a full epoch (all samples)
    pub fn train_epoch(&mut self) -> Result<f32> {
        self.epoch_loss = 0.0;
        self.batch_count = 0;
        self.batch_index = 0;

        let total_batches = (self.training_data.len() + self.config.batch_size - 1)
            / self.config.batch_size;

        for _ in 0..total_batches {
            self.train_batch()?;
        }

        if self.batch_count > 0 {
            Ok(self.epoch_loss / self.batch_count as f32)
        } else {
            Ok(0.0)
        }
    }

    /// Forward pass - simple linear model
    fn forward(&self, input: &Vec<f32, 64>) -> Vec<f32, 16> {
        let mut output = Vec::new();
        let input_size = input.len();
        let output_size = 16.min(self.parameters.len() / input_size.max(1));

        for o in 0..output_size {
            let mut sum = 0.0f32;
            for (i, &x) in input.iter().enumerate() {
                let weight_idx = o * input_size + i;
                if let Some(&w) = self.parameters.get(weight_idx) {
                    sum += w * x;
                }
            }
            // Bias (stored after weights)
            let bias_idx = output_size * input_size + o;
            if let Some(&b) = self.parameters.get(bias_idx) {
                sum += b;
            }
            let _ = output.push(sum);
        }

        output
    }

    /// Compute loss and output gradient
    fn compute_loss_and_gradient(
        &self,
        prediction: &Vec<f32, 16>,
        target: &Vec<f32, 16>,
    ) -> (f32, Vec<f32, 16>) {
        let mut loss = 0.0f32;
        let mut gradient = Vec::new();

        match self.config.loss_function {
            LossFunction::MSE => {
                // Mean Squared Error: L = (1/n) * Σ(y - ŷ)²
                // Gradient: dL/dŷ = 2(ŷ - y) / n
                for (pred, tar) in prediction.iter().zip(target.iter()) {
                    let diff = pred - tar;
                    loss += diff * diff;
                    let _ = gradient.push(2.0 * diff / prediction.len() as f32);
                }
                loss /= prediction.len() as f32;
            }

            LossFunction::BinaryCrossEntropy => {
                // Binary Cross-Entropy: L = -[y*log(ŷ) + (1-y)*log(1-ŷ)]
                // Gradient: dL/dŷ = (ŷ - y) / (ŷ * (1 - ŷ))
                let epsilon = 1e-7f32;
                for (pred, tar) in prediction.iter().zip(target.iter()) {
                    // Apply sigmoid for probability
                    let p = 1.0 / (1.0 + libm::expf(-pred));
                    let p_clipped = p.max(epsilon).min(1.0 - epsilon);

                    loss -= tar * libm::logf(p_clipped)
                        + (1.0 - tar) * libm::logf(1.0 - p_clipped);

                    // Gradient through sigmoid
                    let _ = gradient.push(p - tar);
                }
                loss /= prediction.len() as f32;
            }

            LossFunction::CategoricalCrossEntropy => {
                // Softmax + Cross-Entropy
                // First apply softmax
                let mut max_val = f32::NEG_INFINITY;
                for &p in prediction.iter() {
                    if p > max_val {
                        max_val = p;
                    }
                }

                let mut exp_sum = 0.0f32;
                let mut softmax = Vec::<f32, 16>::new();
                for &p in prediction.iter() {
                    let exp_p = libm::expf(p - max_val);
                    let _ = softmax.push(exp_p);
                    exp_sum += exp_p;
                }

                let epsilon = 1e-7f32;
                for i in 0..softmax.len() {
                    softmax[i] /= exp_sum;
                    let s = softmax[i].max(epsilon);
                    if let Some(&t) = target.get(i) {
                        loss -= t * libm::logf(s);
                        // Gradient of softmax + cross-entropy: softmax - target
                        let _ = gradient.push(softmax[i] - t);
                    }
                }
            }

            LossFunction::Huber { delta } => {
                // Huber loss: quadratic for small errors, linear for large
                for (pred, tar) in prediction.iter().zip(target.iter()) {
                    let diff = pred - tar;
                    let abs_diff = libm::fabsf(diff);

                    if abs_diff <= delta {
                        loss += 0.5 * diff * diff;
                        let _ = gradient.push(diff / prediction.len() as f32);
                    } else {
                        loss += delta * (abs_diff - 0.5 * delta);
                        let grad = if diff > 0.0 { delta } else { -delta };
                        let _ = gradient.push(grad / prediction.len() as f32);
                    }
                }
                loss /= prediction.len() as f32;
            }
        }

        (loss, gradient)
    }

    /// Backward pass - compute parameter gradients
    fn backward(&mut self, input: &Vec<f32, 64>, output_grad: &Vec<f32, 16>) -> Result<()> {
        let input_size = input.len();
        let output_size = output_grad.len();

        // Gradient for weights: dL/dW = dL/dY * X^T
        for o in 0..output_size {
            if let Some(&grad_o) = output_grad.get(o) {
                for (i, &x) in input.iter().enumerate() {
                    let weight_idx = o * input_size + i;
                    if let Some(grad) = self.gradients.get_mut(weight_idx) {
                        *grad += grad_o * x;
                    }
                }
                // Gradient for bias
                let bias_idx = output_size * input_size + o;
                if let Some(grad) = self.gradients.get_mut(bias_idx) {
                    *grad += grad_o;
                }
            }
        }

        Ok(())
    }

    /// Clip gradients to prevent exploding gradients
    fn clip_gradients(&mut self) {
        let mut grad_norm = 0.0f32;
        for &g in &self.gradients {
            grad_norm += g * g;
        }
        grad_norm = libm::sqrtf(grad_norm);

        if grad_norm > self.config.gradient_clip {
            let scale = self.config.gradient_clip / grad_norm;
            for grad in &mut self.gradients {
                *grad *= scale;
            }
        }
    }

    /// Update parameters using SGD with momentum and weight decay
    fn update_parameters(&mut self) {
        let lr = self.config.learning_rate;
        let momentum = self.config.momentum;
        let weight_decay = self.config.weight_decay;

        for i in 0..self.parameters.len() {
            if let (Some(param), Some(vel), Some(grad)) = (
                self.parameters.get_mut(i),
                self.velocities.get_mut(i),
                self.gradients.get(i),
            ) {
                // Apply weight decay (L2 regularization)
                let grad_with_decay = grad + weight_decay * *param;

                // Update velocity with momentum
                *vel = momentum * *vel + grad_with_decay;

                // Update parameter
                *param -= lr * *vel;
            }
        }
    }

    /// Compute current loss on training data
    pub fn compute_training_loss(&self) -> f32 {
        if self.training_data.is_empty() {
            return 0.0;
        }

        let mut total_loss = 0.0f32;
        for sample in &self.training_data {
            let prediction = self.forward(&sample.input);
            let (loss, _) = self.compute_loss_and_gradient(&prediction, &sample.target);
            total_loss += loss;
        }

        total_loss / self.training_data.len() as f32
    }

    /// Create model update for submission
    pub fn create_update(&self, round: Round) -> Result<ModelUpdate> {
        Ok(ModelUpdate {
            drone_id: self.drone_id,
            round,
            parameters: self.parameters.clone(),
            sample_count: self.sample_count,
            loss: self.compute_training_loss(),
            signature: [0u8; 64], // Should be computed using crypto module
        })
    }

    /// Update local model with global parameters
    pub fn update_from_global(&mut self, global: &GlobalModel) {
        self.parameters = global.parameters.clone();

        // Reset velocities for new global model
        for vel in &mut self.velocities {
            *vel = 0.0;
        }
    }

    /// Get current parameters
    pub fn parameters(&self) -> &Vec<f32, MAX_MODEL_PARAMS> {
        &self.parameters
    }

    /// Get training configuration
    pub fn config(&self) -> &TrainingConfig {
        &self.config
    }

    /// Set training configuration
    pub fn set_config(&mut self, config: TrainingConfig) {
        self.config = config;
    }

    /// Get number of training samples
    pub fn training_sample_count(&self) -> usize {
        self.training_data.len()
    }

    /// Serialize model to bytes
    pub fn serialize_model(&self) -> Result<Vec<u8, 4096>> {
        let mut buffer = Vec::new();

        // Write number of parameters
        let param_count = self.parameters.len() as u32;
        buffer
            .extend_from_slice(&param_count.to_le_bytes())
            .map_err(|_| SwarmError::BufferFull)?;

        // Write parameters
        for &param in &self.parameters {
            buffer
                .extend_from_slice(&param.to_le_bytes())
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(buffer)
    }

    /// Deserialize model from bytes
    pub fn deserialize_model(&mut self, data: &[u8]) -> Result<()> {
        if data.len() < 4 {
            return Err(SwarmError::InvalidMessage);
        }

        // Read number of parameters
        let param_count =
            u32::from_le_bytes(data[..4].try_into().map_err(|_| SwarmError::InvalidMessage)?)
                as usize;

        let expected_len = 4 + param_count * 4;
        if data.len() < expected_len {
            return Err(SwarmError::InvalidMessage);
        }

        // Read parameters
        self.parameters.clear();
        self.velocities.clear();
        self.gradients.clear();

        for i in 0..param_count {
            let offset = 4 + i * 4;
            let param = f32::from_le_bytes(
                data[offset..offset + 4]
                    .try_into()
                    .map_err(|_| SwarmError::InvalidMessage)?,
            );
            self.parameters
                .push(param)
                .map_err(|_| SwarmError::BufferFull)?;
            self.velocities
                .push(0.0)
                .map_err(|_| SwarmError::BufferFull)?;
            self.gradients
                .push(0.0)
                .map_err(|_| SwarmError::BufferFull)?;
        }

        Ok(())
    }

    /// Reset training state for new round
    pub fn reset_for_new_round(&mut self) {
        self.sample_count = 0;
        self.batch_index = 0;
        self.epoch_loss = 0.0;
        self.batch_count = 0;

        // Keep velocities for momentum across rounds
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
