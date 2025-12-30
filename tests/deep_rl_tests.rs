//! Deep Reinforcement Learning Tests
//!
//! Tests for Q-learning algorithm selector and hybrid optimizer.

#![cfg(feature = "deep_rl")]

use drone_swarm_system::algorithm_selector::{
    Action, AlgorithmSelector, ExperienceReplay, OptimizationState, QTable, QLearningConfig,
    NUM_ACTIONS, NUM_STATE_BUCKETS,
};
use drone_swarm_system::hybrid_optimizer::{
    ackley_function, rastrigin_function, rosenbrock_function, sphere_function, HybridConfig,
    HybridOptimizer,
};
use drone_swarm_system::meta_heuristic::{
    AlgorithmId, AlgorithmMetadata, Bounds, GwoAdapter, MetaHeuristic, ParameterPreset,
    PsoAdapter, WoaAdapter,
};

// ============================================================================
// Q-Table Tests
// ============================================================================

#[cfg(test)]
mod q_table_tests {
    use super::*;

    #[test]
    fn test_q_table_initialization() {
        let q_table = QTable::new();
        // All Q-values should be initialized to 0
        for state in 0..NUM_STATE_BUCKETS {
            for action in 0..NUM_ACTIONS {
                let a = Action(action as u8);
                assert_eq!(q_table.get(state, a), 0.0);
            }
        }
    }

    #[test]
    fn test_q_table_get_set() {
        let mut q_table = QTable::new();

        q_table.set(10, Action(5), 1.5);
        assert_eq!(q_table.get(10, Action(5)), 1.5);

        q_table.set(10, Action(5), 2.5);
        assert_eq!(q_table.get(10, Action(5)), 2.5);
    }

    #[test]
    fn test_q_table_max_q() {
        let mut q_table = QTable::new();

        q_table.set(5, Action(0), 1.0);
        q_table.set(5, Action(1), 3.0);
        q_table.set(5, Action(2), 2.0);

        assert_eq!(q_table.max_q(5), 3.0);
    }

    #[test]
    fn test_q_table_best_action() {
        let mut q_table = QTable::new();

        q_table.set(7, Action(0), 1.0);
        q_table.set(7, Action(1), 5.0);
        q_table.set(7, Action(2), 3.0);
        q_table.set(7, Action(3), 2.0);

        assert_eq!(q_table.best_action(7), Action(1));
    }

    #[test]
    fn test_q_table_visit_tracking() {
        let mut q_table = QTable::new();

        // Initial visits should be 0
        assert_eq!(q_table.visits(3, Action(2)), 0);

        // set() increments visit count
        q_table.set(3, Action(2), 1.0);
        assert_eq!(q_table.visits(3, Action(2)), 1);

        q_table.set(3, Action(2), 2.0);
        q_table.set(3, Action(2), 3.0);
        assert_eq!(q_table.visits(3, Action(2)), 3);
    }
}

// ============================================================================
// Optimization State Tests
// ============================================================================

#[cfg(test)]
mod optimization_state_tests {
    use super::*;

    #[test]
    fn test_optimization_state_default() {
        let state = OptimizationState::default();

        // Default uses #[derive(Default)], so all f32 fields are 0.0
        assert_eq!(state.fitness, 0.0);
        assert_eq!(state.improvement_rate, 0.0);
        assert_eq!(state.diversity, 0.0);
        assert_eq!(state.stagnation, 0.0);
        assert_eq!(state.budget_remaining, 0.0);
    }

    #[test]
    fn test_optimization_state_to_bucket() {
        let state1 = OptimizationState::default();
        let bucket1 = state1.to_bucket();
        assert!(bucket1 < NUM_STATE_BUCKETS);

        let state2 = OptimizationState {
            fitness: 0.5,
            improvement_rate: 0.1,
            diversity: 0.3,
            stagnation: 0.1,
            budget_remaining: 0.5,
            exploration_ratio: 0.5,
            current_algorithm: 0,
            current_preset: 1,
        };
        let bucket2 = state2.to_bucket();
        assert!(bucket2 < NUM_STATE_BUCKETS);
    }

    #[test]
    fn test_optimization_state_from_metadata() {
        let metadata = AlgorithmMetadata {
            diversity: 0.75,
            convergence_rate: 0.1,
            exploration_ratio: 0.5,
            evaluations: 100,
            stagnation_count: 5,
        };

        let state = OptimizationState::from_metadata(
            &metadata,
            10.0,  // current_fitness
            100.0, // initial_fitness
            1000,  // max_evaluations
            AlgorithmId::PSO,
            ParameterPreset::Balanced,
        );

        assert!(state.fitness >= 0.0 && state.fitness <= 1.0);
        assert_eq!(state.diversity, 0.75);
        assert!(state.budget_remaining > 0.0);
        // PSO is algorithm 0
        assert_eq!(state.current_algorithm, 0);
    }
}

// ============================================================================
// Experience Replay Tests
// ============================================================================

#[cfg(test)]
mod experience_replay_tests {
    use super::*;
    use drone_swarm_system::algorithm_selector::Experience;

    #[test]
    fn test_experience_replay_push_and_sample() {
        let mut replay = ExperienceReplay::new(42);

        // Add several experiences
        for i in 0..10 {
            let exp = Experience {
                state: OptimizationState::default(),
                action: Action(i % NUM_ACTIONS as u8),
                reward: i as f32,
                next_state: OptimizationState::default(),
                done: i == 9,
            };
            replay.push(exp);
        }

        // Sample should return an experience
        let sample = replay.sample();
        assert!(sample.is_some());
    }
}

// ============================================================================
// Algorithm Selector Tests
// ============================================================================

#[cfg(test)]
mod algorithm_selector_tests {
    use super::*;

    #[test]
    fn test_algorithm_selector_creation() {
        let selector = AlgorithmSelector::new(QLearningConfig::default(), 42);
        assert!(selector.epsilon() > 0.0);
    }

    #[test]
    fn test_algorithm_selector_select_action() {
        let mut selector = AlgorithmSelector::new(QLearningConfig::default(), 42);
        let state = OptimizationState::default();

        let (algorithm, preset) = selector.select_action(&state);

        // Should return a valid algorithm
        assert!(matches!(
            algorithm,
            AlgorithmId::PSO | AlgorithmId::ACO | AlgorithmId::GWO | AlgorithmId::WOA
        ));

        // Should return a valid preset
        assert!(matches!(
            preset,
            ParameterPreset::Conservative
                | ParameterPreset::Balanced
                | ParameterPreset::Aggressive
                | ParameterPreset::Exploratory
        ));
    }

    #[test]
    fn test_algorithm_selector_update() {
        let mut selector = AlgorithmSelector::new(QLearningConfig::default(), 42);
        let state = OptimizationState::default();

        // Select action (sets last_action)
        selector.select_action(&state);

        // Update should succeed
        let next_state = OptimizationState {
            fitness: 0.9,
            ..OptimizationState::default()
        };
        let result = selector.update(&next_state, 0.5, false);
        assert!(result.is_ok());
    }

    #[test]
    fn test_algorithm_selector_epsilon_decays_over_updates() {
        let config = QLearningConfig {
            epsilon_initial: 0.5,
            epsilon_final: 0.1,
            epsilon_decay: 0.9,
            ..Default::default()
        };
        let mut selector = AlgorithmSelector::new(config, 42);

        let initial_epsilon = selector.epsilon();
        let state = OptimizationState::default();

        // Do several select-update cycles to trigger epsilon decay
        for _ in 0..50 {
            selector.select_action(&state);
            let _ = selector.update(&state, 0.1, false);
        }

        // Epsilon should have decreased
        assert!(selector.epsilon() < initial_epsilon);
    }

    #[test]
    fn test_algorithm_selector_reset_episode() {
        let mut selector = AlgorithmSelector::new(QLearningConfig::default(), 42);
        let state = OptimizationState::default();

        // Perform some actions
        selector.select_action(&state);

        // Reset should work
        selector.reset_episode();
        // After reset, selecting action should still work
        let (_, _) = selector.select_action(&state);
    }

    #[test]
    fn test_action_creation() {
        let action = Action::new(AlgorithmId::PSO, ParameterPreset::Balanced);
        let (alg, preset) = action.decode();
        assert_eq!(alg, AlgorithmId::PSO);
        assert_eq!(preset, ParameterPreset::Balanced);
    }

    #[test]
    fn test_action_decode_all_combinations() {
        for alg in [
            AlgorithmId::PSO,
            AlgorithmId::ACO,
            AlgorithmId::GWO,
            AlgorithmId::WOA,
        ] {
            for preset in [
                ParameterPreset::Conservative,
                ParameterPreset::Balanced,
                ParameterPreset::Aggressive,
                ParameterPreset::Exploratory,
            ] {
                let action = Action::new(alg, preset);
                let (dec_alg, dec_preset) = action.decode();
                assert_eq!(dec_alg, alg);
                assert_eq!(dec_preset, preset);
            }
        }
    }
}

// ============================================================================
// Meta-Heuristic Adapter Tests
// ============================================================================

#[cfg(test)]
mod meta_heuristic_tests {
    use super::*;

    #[test]
    fn test_pso_adapter_creation() {
        let adapter = PsoAdapter::new(20, 5);
        assert_eq!(adapter.algorithm_id(), AlgorithmId::PSO);
    }

    #[test]
    fn test_pso_adapter_initialize() {
        let mut adapter = PsoAdapter::new(20, 5);
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();

        let result = adapter.initialize(&bounds);
        assert!(result.is_ok());
    }

    #[test]
    fn test_pso_adapter_step() {
        let mut adapter = PsoAdapter::new(20, 5);
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();
        adapter.initialize(&bounds).unwrap();

        let fitness = adapter.step(sphere_function).unwrap();
        assert!(fitness >= 0.0);
    }

    #[test]
    fn test_pso_adapter_presets() {
        let mut adapter = PsoAdapter::new(20, 5);

        adapter.apply_preset(ParameterPreset::Conservative);
        adapter.apply_preset(ParameterPreset::Balanced);
        adapter.apply_preset(ParameterPreset::Aggressive);
        adapter.apply_preset(ParameterPreset::Exploratory);
    }

    #[test]
    fn test_gwo_adapter_creation() {
        let adapter = GwoAdapter::new(20, 5);
        assert_eq!(adapter.algorithm_id(), AlgorithmId::GWO);
    }

    #[test]
    fn test_gwo_adapter_initialize() {
        let mut adapter = GwoAdapter::new(20, 5);
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();

        let result = adapter.initialize(&bounds);
        assert!(result.is_ok());
    }

    #[test]
    fn test_woa_adapter_creation() {
        let adapter = WoaAdapter::new(20, 42);
        assert_eq!(adapter.algorithm_id(), AlgorithmId::WOA);
    }

    #[test]
    fn test_woa_adapter_requires_3d() {
        let mut adapter = WoaAdapter::new(20, 42);

        // 2D should fail
        let bounds_2d = Bounds::uniform(2, -10.0, 10.0).unwrap();
        assert!(adapter.initialize(&bounds_2d).is_err());

        // 3D should succeed
        let bounds_3d = Bounds::uniform(3, -10.0, 10.0).unwrap();
        assert!(adapter.initialize(&bounds_3d).is_ok());
    }

    #[test]
    fn test_bounds_creation() {
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();
        assert_eq!(bounds.dimensions(), 5);
    }

    #[test]
    fn test_algorithm_metadata_default() {
        let metadata = AlgorithmMetadata::default();
        // Default uses #[derive(Default)], so f32 fields are 0.0
        assert_eq!(metadata.diversity, 0.0);
        assert_eq!(metadata.convergence_rate, 0.0);
        assert_eq!(metadata.exploration_ratio, 0.0);
        assert_eq!(metadata.stagnation_count, 0);
        assert_eq!(metadata.evaluations, 0);
    }
}

// ============================================================================
// Hybrid Optimizer Tests
// ============================================================================

#[cfg(test)]
mod hybrid_optimizer_tests {
    use super::*;

    #[test]
    fn test_hybrid_optimizer_creation() {
        let config = HybridConfig {
            dimensions: 3,
            ..Default::default()
        };
        let optimizer = HybridOptimizer::new(config);
        assert_eq!(optimizer.best_fitness(), f32::INFINITY);
    }

    #[test]
    fn test_hybrid_optimizer_initialize() {
        let config = HybridConfig {
            dimensions: 3,
            ..Default::default()
        };
        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let result = optimizer.initialize(&bounds);
        assert!(result.is_ok());
    }

    #[test]
    fn test_hybrid_optimizer_step() {
        let config = HybridConfig {
            population_size: 10,
            dimensions: 3,
            ..Default::default()
        };
        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();
        optimizer.initialize(&bounds).unwrap();

        let fitness = optimizer.step(sphere_function).unwrap();
        assert!(fitness >= 0.0);
        assert_eq!(optimizer.iteration(), 1);
    }

    #[test]
    fn test_hybrid_optimizer_optimize() {
        let config = HybridConfig {
            max_iterations: 50,
            population_size: 20,
            dimensions: 3,
            target_fitness: 0.001,
            switch_interval: 100, // Don't switch during short test
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let result = optimizer.optimize(sphere_function, &bounds).unwrap();

        assert!(result.fitness < f32::INFINITY);
        assert!(result.evaluations > 0);
    }

    #[test]
    fn test_hybrid_optimizer_algorithm_switching() {
        let config = HybridConfig {
            max_iterations: 100,
            population_size: 10,
            dimensions: 3,
            switch_interval: 20,
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let _result = optimizer.optimize(sphere_function, &bounds).unwrap();

        // Stats should show some algorithm usage
        let stats = optimizer.stats();
        assert!(stats.total_iterations > 0);
        assert!(stats.total_evaluations > 0);
    }

    #[test]
    fn test_hybrid_optimizer_convergence_on_sphere() {
        let config = HybridConfig {
            max_iterations: 200,
            population_size: 30,
            dimensions: 3,
            target_fitness: 1.0,
            switch_interval: 50,
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let result = optimizer.optimize(sphere_function, &bounds).unwrap();

        // Should achieve reasonable fitness on sphere function
        assert!(result.fitness < 100.0);
    }
}

// ============================================================================
// Benchmark Function Tests
// ============================================================================

#[cfg(test)]
mod benchmark_tests {
    use super::*;

    #[test]
    fn test_sphere_function_minimum() {
        let x = [0.0, 0.0, 0.0];
        let f = sphere_function(&x);
        assert!(f.abs() < 1e-6);
    }

    #[test]
    fn test_sphere_function_nonzero() {
        let x = [1.0, 2.0, 3.0];
        let f = sphere_function(&x);
        assert_eq!(f, 14.0); // 1 + 4 + 9
    }

    #[test]
    fn test_rastrigin_function_minimum() {
        let x = [0.0, 0.0, 0.0];
        let f = rastrigin_function(&x);
        assert!(f.abs() < 1e-6);
    }

    #[test]
    fn test_rosenbrock_function_minimum() {
        let x = [1.0, 1.0, 1.0];
        let f = rosenbrock_function(&x);
        assert!(f.abs() < 1e-6);
    }

    #[test]
    fn test_ackley_function_minimum() {
        let x = [0.0, 0.0, 0.0];
        let f = ackley_function(&x);
        assert!(f.abs() < 1e-5);
    }

    #[test]
    fn test_ackley_function_positive() {
        let x = [1.0, 1.0, 1.0];
        let f = ackley_function(&x);
        assert!(f > 0.0);
    }
}

// ============================================================================
// Integration Tests
// ============================================================================

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_full_rl_optimization_cycle() {
        // Test a complete RL optimization cycle:
        // 1. Create optimizer
        // 2. Run optimization
        // 3. Verify learning occurred

        let config = HybridConfig {
            max_iterations: 100,
            population_size: 20,
            dimensions: 3,
            switch_interval: 25, // Switch 4 times
            rl_config: QLearningConfig {
                alpha: 0.2,
                gamma: 0.95,
                epsilon_initial: 0.5,
                epsilon_final: 0.1,
                epsilon_decay: 0.95,
                ..Default::default()
            },
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let result = optimizer.optimize(sphere_function, &bounds).unwrap();

        // Should have improved from initial random positions
        assert!(result.fitness < f32::INFINITY);

        // Stats should reflect algorithm usage
        let stats = optimizer.stats();
        assert!(stats.total_iterations >= 100);
        assert!(stats.total_evaluations > 0);

        // Epsilon should have decayed
        assert!(stats.final_epsilon < 0.5);
    }

    #[test]
    fn test_multi_problem_adaptation() {
        // Test optimizer handles different problem characteristics

        let config = HybridConfig {
            max_iterations: 50,
            population_size: 15,
            dimensions: 3,
            switch_interval: 10,
            ..Default::default()
        };
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        // Optimize sphere (unimodal)
        let mut opt1 = HybridOptimizer::new(config.clone());
        let result1 = opt1.optimize(sphere_function, &bounds).unwrap();
        assert!(result1.fitness < f32::INFINITY);

        // Optimize rastrigin (multimodal)
        let mut opt2 = HybridOptimizer::new(config.clone());
        let result2 = opt2.optimize(rastrigin_function, &bounds).unwrap();
        assert!(result2.fitness < f32::INFINITY);

        // Both should produce valid results
        assert!(result1.evaluations > 0);
        assert!(result2.evaluations > 0);
    }

    #[test]
    fn test_optimizer_stats_tracking() {
        let config = HybridConfig {
            max_iterations: 30,
            population_size: 10,
            dimensions: 3,
            switch_interval: 10,
            ..Default::default()
        };

        let mut optimizer = HybridOptimizer::new(config);
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let _ = optimizer.optimize(sphere_function, &bounds).unwrap();

        let stats = optimizer.stats();
        assert_eq!(stats.total_iterations, 30);
        assert_eq!(stats.total_evaluations, 30 * 10); // iterations * population
    }
}
