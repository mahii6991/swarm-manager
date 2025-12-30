//! Deep Reinforcement Learning Algorithm Selector
//!
//! Uses Q-learning to dynamically select the best optimization algorithm
//! and parameter preset based on the current optimization state.
//!
//! This module is feature-gated under `deep_rl`.

use crate::meta_heuristic::{AlgorithmId, AlgorithmMetadata, ParameterPreset};
use crate::types::{Result, SwarmError};
use heapless::Vec;

/// Maximum experience buffer size (bounded replay)
pub const MAX_EXPERIENCE_SIZE: usize = 256;

/// Number of possible actions (4 algorithms x 4 parameter presets)
pub const NUM_ACTIONS: usize = 16;

/// Number of discretized state buckets for Q-table
pub const NUM_STATE_BUCKETS: usize = 64;

/// State feature dimension
pub const STATE_DIM: usize = 8;

/// Action encoding: algorithm_id * 4 + preset_id
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Action(pub u8);

impl Action {
    /// Create action from algorithm and preset
    pub fn new(algorithm: AlgorithmId, preset: ParameterPreset) -> Self {
        let alg_id = match algorithm {
            AlgorithmId::PSO => 0,
            AlgorithmId::ACO => 1,
            AlgorithmId::GWO => 2,
            AlgorithmId::WOA => 3,
        };
        let preset_id = match preset {
            ParameterPreset::Conservative => 0,
            ParameterPreset::Balanced => 1,
            ParameterPreset::Aggressive => 2,
            ParameterPreset::Exploratory => 3,
        };
        Self((alg_id * 4 + preset_id) as u8)
    }

    /// Decode action to algorithm and preset
    pub fn decode(&self) -> (AlgorithmId, ParameterPreset) {
        let alg_id = self.0 / 4;
        let preset_id = self.0 % 4;

        let algorithm = match alg_id {
            0 => AlgorithmId::PSO,
            1 => AlgorithmId::ACO,
            2 => AlgorithmId::GWO,
            _ => AlgorithmId::WOA,
        };

        let preset = match preset_id {
            0 => ParameterPreset::Conservative,
            1 => ParameterPreset::Balanced,
            2 => ParameterPreset::Aggressive,
            _ => ParameterPreset::Exploratory,
        };

        (algorithm, preset)
    }
}

/// Optimization state features for Q-learning
#[derive(Debug, Clone, Copy, Default)]
pub struct OptimizationState {
    /// Current best fitness (normalized to 0-1)
    pub fitness: f32,
    /// Fitness improvement rate (normalized)
    pub improvement_rate: f32,
    /// Population diversity (0-1)
    pub diversity: f32,
    /// Stagnation counter (normalized to 0-1)
    pub stagnation: f32,
    /// Function evaluation budget remaining (0-1)
    pub budget_remaining: f32,
    /// Exploration ratio (0-1)
    pub exploration_ratio: f32,
    /// Current algorithm (one-hot encoded, 4 values compressed to 2 bits)
    pub current_algorithm: u8,
    /// Current preset (one-hot encoded, 4 values compressed to 2 bits)
    pub current_preset: u8,
}

impl OptimizationState {
    /// Create state from algorithm metadata and context
    pub fn from_metadata(
        metadata: &AlgorithmMetadata,
        current_fitness: f32,
        initial_fitness: f32,
        max_evaluations: u32,
        algorithm: AlgorithmId,
        preset: ParameterPreset,
    ) -> Self {
        // Normalize fitness (assuming minimization, lower is better)
        let fitness_normalized = if initial_fitness > 0.0 {
            (current_fitness / initial_fitness).min(1.0)
        } else {
            1.0
        };

        // Normalize stagnation (max 100 iterations)
        let stagnation_normalized = (metadata.stagnation_count as f32 / 100.0).min(1.0);

        // Budget remaining
        let budget = 1.0 - (metadata.evaluations as f32 / max_evaluations as f32).min(1.0);

        // Encode algorithm
        let alg_id = match algorithm {
            AlgorithmId::PSO => 0,
            AlgorithmId::ACO => 1,
            AlgorithmId::GWO => 2,
            AlgorithmId::WOA => 3,
        };

        // Encode preset
        let preset_id = match preset {
            ParameterPreset::Conservative => 0,
            ParameterPreset::Balanced => 1,
            ParameterPreset::Aggressive => 2,
            ParameterPreset::Exploratory => 3,
        };

        Self {
            fitness: fitness_normalized,
            improvement_rate: metadata.convergence_rate.min(1.0),
            diversity: metadata.diversity,
            stagnation: stagnation_normalized,
            budget_remaining: budget,
            exploration_ratio: metadata.exploration_ratio,
            current_algorithm: alg_id,
            current_preset: preset_id,
        }
    }

    /// Discretize state to bucket index for Q-table lookup
    pub fn to_bucket(&self) -> usize {
        // Use 6 key features, each discretized to 2 bits (4 levels)
        // fitness: 2 bits, stagnation: 2 bits, budget: 2 bits
        // Total: 6 bits = 64 buckets

        let fitness_bucket = (self.fitness * 3.0) as usize;
        let stagnation_bucket = (self.stagnation * 3.0) as usize;
        let budget_bucket = (self.budget_remaining * 3.0) as usize;

        let bucket = (fitness_bucket.min(3) << 4)
            | (stagnation_bucket.min(3) << 2)
            | budget_bucket.min(3);

        bucket.min(NUM_STATE_BUCKETS - 1)
    }
}

/// Experience tuple for replay buffer
#[derive(Debug, Clone, Copy)]
pub struct Experience {
    /// State when action was taken
    pub state: OptimizationState,
    /// Action taken
    pub action: Action,
    /// Reward received
    pub reward: f32,
    /// Next state after action
    pub next_state: OptimizationState,
    /// Whether episode ended
    pub done: bool,
}

/// Bounded experience replay buffer
pub struct ExperienceReplay {
    buffer: Vec<Experience, MAX_EXPERIENCE_SIZE>,
    position: usize,
    full: bool,
    seed: u64,
}

impl ExperienceReplay {
    /// Create new experience replay buffer
    pub fn new(seed: u64) -> Self {
        Self {
            buffer: Vec::new(),
            position: 0,
            full: false,
            seed,
        }
    }

    /// Add experience to buffer (circular)
    pub fn push(&mut self, experience: Experience) {
        if self.buffer.len() < MAX_EXPERIENCE_SIZE {
            self.buffer.push(experience).ok();
        } else {
            self.buffer[self.position] = experience;
            self.full = true;
        }
        self.position = (self.position + 1) % MAX_EXPERIENCE_SIZE;
    }

    /// Get buffer length
    pub fn len(&self) -> usize {
        if self.full {
            MAX_EXPERIENCE_SIZE
        } else {
            self.buffer.len()
        }
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Sample random experience (pseudo-random for no_std)
    pub fn sample(&mut self) -> Option<&Experience> {
        if self.is_empty() {
            return None;
        }

        // Simple LCG for pseudo-random sampling
        self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let idx = (self.seed as usize) % self.len();

        self.buffer.get(idx)
    }

    /// Sample batch of experiences
    pub fn sample_batch(&mut self, batch_size: usize) -> Vec<Experience, 32> {
        let mut batch = Vec::new();
        let actual_size = batch_size.min(self.len()).min(32);

        for _ in 0..actual_size {
            if let Some(exp) = self.sample() {
                batch.push(*exp).ok();
            }
        }

        batch
    }
}

/// Q-table for tabular Q-learning
pub struct QTable {
    /// Q-values: [state_bucket][action]
    q_values: [[f32; NUM_ACTIONS]; NUM_STATE_BUCKETS],
    /// Visit counts for exploration bonus
    visit_counts: [[u32; NUM_ACTIONS]; NUM_STATE_BUCKETS],
}

impl QTable {
    /// Create new Q-table initialized to zeros
    pub fn new() -> Self {
        Self {
            q_values: [[0.0; NUM_ACTIONS]; NUM_STATE_BUCKETS],
            visit_counts: [[0; NUM_ACTIONS]; NUM_STATE_BUCKETS],
        }
    }

    /// Get Q-value for state-action pair
    pub fn get(&self, state_bucket: usize, action: Action) -> f32 {
        self.q_values[state_bucket.min(NUM_STATE_BUCKETS - 1)][action.0 as usize]
    }

    /// Set Q-value for state-action pair
    pub fn set(&mut self, state_bucket: usize, action: Action, value: f32) {
        let bucket = state_bucket.min(NUM_STATE_BUCKETS - 1);
        self.q_values[bucket][action.0 as usize] = value;
        self.visit_counts[bucket][action.0 as usize] += 1;
    }

    /// Get best action for state
    pub fn best_action(&self, state_bucket: usize) -> Action {
        let bucket = state_bucket.min(NUM_STATE_BUCKETS - 1);
        let mut best_action = 0;
        let mut best_value = f32::NEG_INFINITY;

        for action in 0..NUM_ACTIONS {
            if self.q_values[bucket][action] > best_value {
                best_value = self.q_values[bucket][action];
                best_action = action;
            }
        }

        Action(best_action as u8)
    }

    /// Get maximum Q-value for state
    pub fn max_q(&self, state_bucket: usize) -> f32 {
        let bucket = state_bucket.min(NUM_STATE_BUCKETS - 1);
        self.q_values[bucket]
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// Get visit count for state-action pair
    pub fn visits(&self, state_bucket: usize, action: Action) -> u32 {
        self.visit_counts[state_bucket.min(NUM_STATE_BUCKETS - 1)][action.0 as usize]
    }
}

impl Default for QTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Q-learning hyperparameters
#[derive(Debug, Clone, Copy)]
pub struct QLearningConfig {
    /// Learning rate (alpha)
    pub alpha: f32,
    /// Discount factor (gamma)
    pub gamma: f32,
    /// Initial exploration rate (epsilon)
    pub epsilon_initial: f32,
    /// Final exploration rate
    pub epsilon_final: f32,
    /// Epsilon decay rate per step
    pub epsilon_decay: f32,
    /// Use experience replay
    pub use_replay: bool,
    /// Replay batch size
    pub batch_size: usize,
    /// UCB exploration bonus coefficient
    pub ucb_c: f32,
}

impl Default for QLearningConfig {
    fn default() -> Self {
        Self {
            alpha: 0.1,
            gamma: 0.95,
            epsilon_initial: 0.3,
            epsilon_final: 0.05,
            epsilon_decay: 0.995,
            use_replay: true,
            batch_size: 16,
            ucb_c: 1.0,
        }
    }
}

/// Algorithm selector using Q-learning
pub struct AlgorithmSelector {
    q_table: QTable,
    replay: ExperienceReplay,
    config: QLearningConfig,
    /// Current exploration rate
    epsilon: f32,
    /// Current state
    current_state: OptimizationState,
    /// Last action taken
    last_action: Option<Action>,
    /// Total steps taken
    total_steps: u64,
    /// Random seed for exploration
    seed: u64,
}

impl AlgorithmSelector {
    /// Create new algorithm selector
    pub fn new(config: QLearningConfig, seed: u64) -> Self {
        Self {
            q_table: QTable::new(),
            replay: ExperienceReplay::new(seed),
            config,
            epsilon: config.epsilon_initial,
            current_state: OptimizationState::default(),
            last_action: None,
            total_steps: 0,
            seed,
        }
    }

    /// Create with default config
    pub fn default_with_seed(seed: u64) -> Self {
        Self::new(QLearningConfig::default(), seed)
    }

    /// Select action using epsilon-greedy with UCB exploration bonus
    pub fn select_action(&mut self, state: &OptimizationState) -> (AlgorithmId, ParameterPreset) {
        let state_bucket = state.to_bucket();

        // Epsilon-greedy exploration
        self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let rand_val = (self.seed as f32) / (u64::MAX as f32);

        let action = if rand_val < self.epsilon {
            // Random exploration
            self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            Action((self.seed % NUM_ACTIONS as u64) as u8)
        } else {
            // Greedy with UCB bonus
            self.select_ucb_action(state_bucket)
        };

        self.current_state = *state;
        self.last_action = Some(action);

        action.decode()
    }

    /// Select action using UCB exploration bonus
    fn select_ucb_action(&self, state_bucket: usize) -> Action {
        let total_visits: u32 = (0..NUM_ACTIONS)
            .map(|a| self.q_table.visits(state_bucket, Action(a as u8)))
            .sum();

        let mut best_action = 0;
        let mut best_value = f32::NEG_INFINITY;

        for action_id in 0..NUM_ACTIONS {
            let action = Action(action_id as u8);
            let q_value = self.q_table.get(state_bucket, action);
            let visits = self.q_table.visits(state_bucket, action);

            // UCB bonus
            let bonus = if visits == 0 || total_visits == 0 {
                f32::INFINITY // Unexplored action
            } else {
                self.config.ucb_c
                    * libm::sqrtf(libm::logf(total_visits as f32 + 1.0) / visits as f32)
            };

            let value = q_value + bonus;

            if value > best_value {
                best_value = value;
                best_action = action_id;
            }
        }

        Action(best_action as u8)
    }

    /// Update Q-values based on observed reward
    pub fn update(&mut self, next_state: &OptimizationState, reward: f32, done: bool) -> Result<()> {
        let action = self.last_action.ok_or(SwarmError::InvalidParameter)?;

        // Store experience
        let experience = Experience {
            state: self.current_state,
            action,
            reward,
            next_state: *next_state,
            done,
        };

        if self.config.use_replay {
            self.replay.push(experience);
        }

        // Q-learning update
        let state_bucket = self.current_state.to_bucket();
        let next_bucket = next_state.to_bucket();

        let current_q = self.q_table.get(state_bucket, action);
        let max_next_q = if done { 0.0 } else { self.q_table.max_q(next_bucket) };

        let target = reward + self.config.gamma * max_next_q;
        let new_q = current_q + self.config.alpha * (target - current_q);

        self.q_table.set(state_bucket, action, new_q);

        // Experience replay updates
        if self.config.use_replay && self.replay.len() >= self.config.batch_size {
            self.replay_update()?;
        }

        // Decay epsilon
        self.epsilon = (self.epsilon * self.config.epsilon_decay).max(self.config.epsilon_final);

        self.total_steps += 1;
        self.current_state = *next_state;

        Ok(())
    }

    /// Perform batch update from experience replay
    fn replay_update(&mut self) -> Result<()> {
        let batch = self.replay.sample_batch(self.config.batch_size);

        for exp in batch.iter() {
            let state_bucket = exp.state.to_bucket();
            let next_bucket = exp.next_state.to_bucket();

            let current_q = self.q_table.get(state_bucket, exp.action);
            let max_next_q = if exp.done {
                0.0
            } else {
                self.q_table.max_q(next_bucket)
            };

            let target = exp.reward + self.config.gamma * max_next_q;
            let new_q = current_q + self.config.alpha * (target - current_q);

            self.q_table.set(state_bucket, exp.action, new_q);
        }

        Ok(())
    }

    /// Get current exploration rate
    pub fn epsilon(&self) -> f32 {
        self.epsilon
    }

    /// Get total steps taken
    pub fn total_steps(&self) -> u64 {
        self.total_steps
    }

    /// Get Q-value for state-action pair
    pub fn q_value(&self, state: &OptimizationState, action: Action) -> f32 {
        self.q_table.get(state.to_bucket(), action)
    }

    /// Get best action for current state (no exploration)
    pub fn best_action(&self, state: &OptimizationState) -> (AlgorithmId, ParameterPreset) {
        self.q_table.best_action(state.to_bucket()).decode()
    }

    /// Reset selector state (keep learned Q-values)
    pub fn reset_episode(&mut self) {
        self.current_state = OptimizationState::default();
        self.last_action = None;
    }

    /// Get replay buffer size
    pub fn replay_size(&self) -> usize {
        self.replay.len()
    }
}

/// Calculate reward from optimization progress
pub fn calculate_reward(
    prev_fitness: f32,
    curr_fitness: f32,
    metadata: &AlgorithmMetadata,
    target_fitness: f32,
) -> f32 {
    // Reward components:
    // 1. Fitness improvement (primary)
    // 2. Reaching target bonus
    // 3. Efficiency penalty (evaluations)
    // 4. Stagnation penalty

    let improvement = (prev_fitness - curr_fitness) / prev_fitness.max(1e-8);
    let improvement_reward = improvement * 10.0; // Scale improvement

    // Target bonus
    let target_bonus = if curr_fitness <= target_fitness {
        5.0
    } else {
        0.0
    };

    // Stagnation penalty
    let stagnation_penalty = if metadata.stagnation_count > 10 {
        -0.5
    } else {
        0.0
    };

    // Diversity bonus (encourage exploration)
    let diversity_bonus = metadata.diversity * 0.1;

    improvement_reward + target_bonus + stagnation_penalty + diversity_bonus
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_action_encoding() {
        let action = Action::new(AlgorithmId::PSO, ParameterPreset::Balanced);
        let (alg, preset) = action.decode();
        assert_eq!(alg, AlgorithmId::PSO);
        assert_eq!(preset, ParameterPreset::Balanced);

        let action = Action::new(AlgorithmId::GWO, ParameterPreset::Aggressive);
        let (alg, preset) = action.decode();
        assert_eq!(alg, AlgorithmId::GWO);
        assert_eq!(preset, ParameterPreset::Aggressive);
    }

    #[test]
    fn test_state_discretization() {
        let state1 = OptimizationState {
            fitness: 0.0,
            stagnation: 0.0,
            budget_remaining: 1.0,
            ..Default::default()
        };

        let state2 = OptimizationState {
            fitness: 1.0,
            stagnation: 1.0,
            budget_remaining: 0.0,
            ..Default::default()
        };

        let bucket1 = state1.to_bucket();
        let bucket2 = state2.to_bucket();

        assert_ne!(bucket1, bucket2);
        assert!(bucket1 < NUM_STATE_BUCKETS);
        assert!(bucket2 < NUM_STATE_BUCKETS);
    }

    #[test]
    fn test_q_table_operations() {
        let mut q_table = QTable::new();

        let action = Action::new(AlgorithmId::PSO, ParameterPreset::Balanced);
        q_table.set(0, action, 1.5);

        assert_eq!(q_table.get(0, action), 1.5);
        assert_eq!(q_table.visits(0, action), 1);
    }

    #[test]
    fn test_experience_replay() {
        let mut replay = ExperienceReplay::new(42);

        let exp = Experience {
            state: OptimizationState::default(),
            action: Action(0),
            reward: 1.0,
            next_state: OptimizationState::default(),
            done: false,
        };

        replay.push(exp);
        assert_eq!(replay.len(), 1);

        let sampled = replay.sample();
        assert!(sampled.is_some());
    }

    #[test]
    fn test_selector_action_selection() {
        let mut selector = AlgorithmSelector::default_with_seed(42);
        let state = OptimizationState::default();

        let (alg, preset) = selector.select_action(&state);

        // Should return valid algorithm and preset
        matches!(
            alg,
            AlgorithmId::PSO | AlgorithmId::ACO | AlgorithmId::GWO | AlgorithmId::WOA
        );
        matches!(
            preset,
            ParameterPreset::Conservative
                | ParameterPreset::Balanced
                | ParameterPreset::Aggressive
                | ParameterPreset::Exploratory
        );
    }

    #[test]
    fn test_reward_calculation() {
        let metadata = AlgorithmMetadata {
            diversity: 0.5,
            stagnation_count: 0,
            ..Default::default()
        };

        let reward = calculate_reward(100.0, 90.0, &metadata, 0.0);
        assert!(reward > 0.0); // Improvement should give positive reward

        let reward_worse = calculate_reward(90.0, 100.0, &metadata, 0.0);
        assert!(reward_worse < reward); // Getting worse should give lower reward
    }
}
