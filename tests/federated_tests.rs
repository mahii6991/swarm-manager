//! Comprehensive tests for the federated learning module
//!
//! Tests GlobalModel, FederatedCoordinator, LocalTrainer, SecureAggregation

use drone_swarm_system::crypto::KeyStore;
use drone_swarm_system::federated::*;
use drone_swarm_system::types::*;

// ═══════════════════════════════════════════════════════════════════════════
// GlobalModel Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod global_model_tests {
    use super::*;

    #[test]
    fn test_new() {
        let model = GlobalModel::new(10).unwrap();
        assert_eq!(model.round, 0);
        assert_eq!(model.parameters.len(), 10);
        assert_eq!(model.contributor_count, 0);
        assert_eq!(model.accuracy, 0.0);
    }

    #[test]
    fn test_new_with_different_sizes() {
        let model_small = GlobalModel::new(5).unwrap();
        assert_eq!(model_small.parameters.len(), 5);

        let model_medium = GlobalModel::new(100).unwrap();
        assert_eq!(model_medium.parameters.len(), 100);

        let model_large = GlobalModel::new(500).unwrap();
        assert_eq!(model_large.parameters.len(), 500);
    }

    #[test]
    fn test_parameters_initialized_to_zero() {
        let model = GlobalModel::new(10).unwrap();
        for param in &model.parameters {
            assert_eq!(*param, 0.0);
        }
    }

    #[test]
    fn test_clone() {
        let model = GlobalModel::new(10).unwrap();
        let cloned = model.clone();
        assert_eq!(model.round, cloned.round);
        assert_eq!(model.parameters.len(), cloned.parameters.len());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ModelUpdate Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod model_update_tests {
    use super::*;

    fn create_update(drone_id: u64, round: u64, params: &[f32]) -> ModelUpdate {
        let mut parameters = heapless::Vec::new();
        for &p in params {
            parameters.push(p).unwrap();
        }

        ModelUpdate {
            drone_id: DroneId::new(drone_id),
            round,
            parameters,
            sample_count: 10,
            loss: 0.5,
            signature: [0u8; 64],
        }
    }

    #[test]
    fn test_model_update_creation() {
        let update = create_update(1, 0, &[1.0, 2.0, 3.0]);

        assert_eq!(update.drone_id, DroneId::new(1));
        assert_eq!(update.round, 0);
        assert_eq!(update.parameters.len(), 3);
        assert_eq!(update.sample_count, 10);
    }

    #[test]
    fn test_model_update_clone() {
        let update = create_update(1, 5, &[1.0, 2.0]);
        let cloned = update.clone();

        assert_eq!(update.drone_id, cloned.drone_id);
        assert_eq!(update.round, cloned.round);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FederatedCoordinator Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod federated_coordinator_tests {
    use super::*;

    fn create_coordinator() -> FederatedCoordinator {
        let model = GlobalModel::new(10).unwrap();
        let key_store = KeyStore::new();
        FederatedCoordinator::new(DroneId::new(1), model, key_store)
    }

    #[test]
    fn test_new() {
        let coordinator = create_coordinator();
        assert_eq!(coordinator.current_round(), 0);
        assert!(!coordinator.is_ready_for_aggregation());
        assert_eq!(coordinator.pending_count(), 0);
    }

    #[test]
    fn test_current_round() {
        let coordinator = create_coordinator();
        assert_eq!(coordinator.current_round(), 0);
    }

    #[test]
    fn test_global_model() {
        let coordinator = create_coordinator();
        let model = coordinator.global_model();
        assert_eq!(model.round, 0);
        assert_eq!(model.parameters.len(), 10);
    }

    #[test]
    fn test_set_min_participants() {
        let mut coordinator = create_coordinator();
        coordinator.set_min_participants(5);
        // Just verify no panic
    }

    #[test]
    fn test_set_bft_enabled() {
        let mut coordinator = create_coordinator();
        coordinator.set_bft_enabled(true);
        coordinator.set_bft_enabled(false);
        // Just verify no panic
    }

    #[test]
    fn test_pending_count() {
        let coordinator = create_coordinator();
        assert_eq!(coordinator.pending_count(), 0);
    }

    #[test]
    fn test_is_ready_for_aggregation_empty() {
        let coordinator = create_coordinator();
        assert!(!coordinator.is_ready_for_aggregation());
    }

    #[test]
    fn test_aggregate_not_enough_participants() {
        let mut coordinator = create_coordinator();
        coordinator.set_min_participants(3);

        let result = coordinator.aggregate_updates();
        assert!(result.is_err()); // Not enough participants
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LocalTrainer Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod local_trainer_tests {
    use super::*;

    fn create_trainer() -> LocalTrainer {
        let params = heapless::Vec::from_slice(&[0.0f32; 10]).unwrap();
        LocalTrainer::new(DroneId::new(1), params)
    }

    fn create_trainer_with_data() -> LocalTrainer {
        let params = heapless::Vec::from_slice(&[0.0f32; 10]).unwrap();
        let mut trainer = LocalTrainer::new(DroneId::new(1), params);

        // Add training samples
        for i in 0..5 {
            let input = heapless::Vec::from_slice(&[i as f32 / 5.0; 10]).unwrap();
            let target = heapless::Vec::from_slice(&[(i as f32 / 5.0) * 2.0; 10]).unwrap();
            let sample = TrainingSample { input, target };
            trainer.add_sample(sample).ok();
        }
        trainer
    }

    #[test]
    fn test_new() {
        let trainer = create_trainer();
        assert_eq!(trainer.parameters().len(), 10);
    }

    #[test]
    fn test_train_batch() {
        let mut trainer = create_trainer_with_data();

        let loss = trainer.train_batch().unwrap();
        assert!(loss >= 0.0);
    }

    #[test]
    fn test_train_multiple_batches() {
        let mut trainer = create_trainer_with_data();

        for _ in 0..10 {
            let loss = trainer.train_batch().unwrap();
            assert!(loss >= 0.0);
        }
    }

    #[test]
    fn test_create_update() {
        let trainer = create_trainer();

        let update = trainer.create_update(0).unwrap();
        assert_eq!(update.drone_id, DroneId::new(1));
        assert_eq!(update.round, 0);
        assert_eq!(update.parameters.len(), 10);
    }

    #[test]
    fn test_update_from_global() {
        let mut trainer = create_trainer();

        let mut global_params = heapless::Vec::new();
        for i in 0..10 {
            global_params.push(i as f32).unwrap();
        }

        let global_model = GlobalModel {
            round: 5,
            parameters: global_params,
            contributor_count: 3,
            accuracy: 0.85,
        };

        trainer.update_from_global(&global_model);
        assert_eq!(trainer.parameters()[0], 0.0);
        assert_eq!(trainer.parameters()[9], 9.0);
    }

    #[test]
    fn test_parameters() {
        let trainer = create_trainer();
        let params = trainer.parameters();
        assert_eq!(params.len(), 10);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SecureAggregation Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod secure_aggregation_tests {
    use super::*;

    #[test]
    fn test_new() {
        let sa = SecureAggregation::new();
        // Just verify creation
        assert!(true);
    }

    #[test]
    fn test_default() {
        let sa = SecureAggregation::default();
        // Just verify creation
        assert!(true);
    }

    #[test]
    fn test_add_participant() {
        let mut sa = SecureAggregation::new();
        let secret = [42u8; 32];

        sa.add_participant(DroneId::new(1), secret).unwrap();
    }

    #[test]
    fn test_add_multiple_participants() {
        let mut sa = SecureAggregation::new();

        for i in 1..=10 {
            let mut secret = [0u8; 32];
            secret[0] = i as u8;
            sa.add_participant(DroneId::new(i), secret).unwrap();
        }
    }

    #[test]
    fn test_encrypt_update() {
        let mut sa = SecureAggregation::new();
        // Add participant first so we have a secret
        sa.add_participant(DroneId::new(1), [0u8; 32]).unwrap();

        let mut params = heapless::Vec::new();
        for i in 0..5 {
            params.push(i as f32).unwrap();
        }

        let update = ModelUpdate {
            drone_id: DroneId::new(1),
            round: 0,
            parameters: params,
            sample_count: 10,
            loss: 0.5,
            signature: [0u8; 64],
        };

        let encrypted = sa.encrypt_update(&update).unwrap();
        // Implementation now returns encrypted data
        assert!(!encrypted.is_empty());
    }

    #[test]
    fn test_aggregate_encrypted() {
        let mut sa = SecureAggregation::new();
        let secret = [1u8; 32];
        sa.add_participant(DroneId::new(1), secret).unwrap();

        let mut params = heapless::Vec::new();
        for i in 0..5 {
            params.push(i as f32).unwrap();
        }

        let update = ModelUpdate {
            drone_id: DroneId::new(1),
            round: 0,
            parameters: params,
            sample_count: 10,
            loss: 0.5,
            signature: [0u8; 64],
        };

        let encrypted = sa.encrypt_update(&update).unwrap();

        let updates = [(DroneId::new(1), encrypted)];
        let result = sa.aggregate_encrypted(&updates).unwrap();

        // Should recover parameters (single update average = update)
        assert!(!result.is_empty());
        assert!((result[0] - 0.0).abs() < 1e-6);
        assert!((result[4] - 4.0).abs() < 1e-6);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FederatedMessage Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod federated_message_tests {
    use super::*;

    #[test]
    fn test_submit_update_message() {
        let mut params = heapless::Vec::new();
        params.push(1.0).unwrap();

        let update = ModelUpdate {
            drone_id: DroneId::new(1),
            round: 0,
            parameters: params,
            sample_count: 10,
            loss: 0.5,
            signature: [0u8; 64],
        };

        let _msg = FederatedMessage::SubmitUpdate(update);
    }

    #[test]
    fn test_request_global_model_message() {
        let _msg = FederatedMessage::RequestGlobalModel { round: 5 };
    }

    #[test]
    fn test_global_model_response_message() {
        let model = GlobalModel::new(10).unwrap();
        let _msg = FederatedMessage::GlobalModelResponse(model);
    }

    #[test]
    fn test_start_round_message() {
        let _msg = FederatedMessage::StartRound { round: 1 };
    }

    #[test]
    fn test_round_complete_message() {
        let _msg = FederatedMessage::RoundComplete { round: 1 };
    }

    #[test]
    fn test_message_clone() {
        let msg = FederatedMessage::StartRound { round: 42 };
        let _cloned = msg.clone();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod integration_tests {
    use super::*;

    fn add_samples_to_trainer(trainer: &mut LocalTrainer, num_samples: usize, param_size: usize) {
        for i in 0..num_samples {
            let mut input = heapless::Vec::new();
            let mut target = heapless::Vec::new();
            for _ in 0..param_size {
                input.push(i as f32 / num_samples as f32).ok();
                target.push((i as f32 / num_samples as f32) * 2.0).ok();
            }
            let sample = TrainingSample { input, target };
            trainer.add_sample(sample).ok();
        }
    }

    #[test]
    fn test_local_training_workflow() {
        let params = heapless::Vec::from_slice(&[1.0f32; 10]).unwrap();
        let mut trainer = LocalTrainer::new(DroneId::new(1), params);

        // Add training samples
        add_samples_to_trainer(&mut trainer, 5, 10);

        // Train for several batches
        let mut losses = Vec::new();
        for _ in 0..5 {
            let loss = trainer.train_batch().unwrap();
            losses.push(loss);
        }

        // Create update for submission
        let update = trainer.create_update(0).unwrap();
        assert_eq!(update.drone_id, DroneId::new(1));
    }

    #[test]
    fn test_model_update_lifecycle() {
        // Create initial model
        let model = GlobalModel::new(5).unwrap();
        assert_eq!(model.round, 0);

        // Create trainer from model
        let mut trainer = LocalTrainer::new(DroneId::new(1), model.parameters.clone());

        // Add training sample
        add_samples_to_trainer(&mut trainer, 3, 5);

        // Train
        trainer.train_batch().unwrap();

        // Create update
        let update = trainer.create_update(0).unwrap();
        assert_eq!(update.parameters.len(), 5);
    }
}
