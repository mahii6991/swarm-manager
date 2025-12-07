//! Comprehensive tests for the PSO Advanced module

use drone_swarm_system::pso::*;
use drone_swarm_system::pso_advanced::*;

// ═══════════════════════════════════════════════════════════════════════════
// TopologyManager Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod topology_manager_tests {
    use super::*;

    #[test]
    fn test_star_topology() {
        let manager = TopologyManager::new(Topology::Star, 10).unwrap();
        let neighbors = manager.get_neighbors(0);
        assert!(!neighbors.is_empty());
    }

    #[test]
    fn test_ring_topology() {
        let manager = TopologyManager::new(Topology::Ring, 10).unwrap();
        let neighbors = manager.get_neighbors(5);
        assert!(!neighbors.is_empty());
    }

    #[test]
    fn test_von_neumann_topology() {
        let manager = TopologyManager::new(Topology::VonNeumann, 16).unwrap();
        let neighbors = manager.get_neighbors(5);
        assert!(!neighbors.is_empty());
    }

    #[test]
    fn test_pyramid_topology() {
        let manager = TopologyManager::new(Topology::Pyramid, 15).unwrap();
        let neighbors = manager.get_neighbors(0);
        assert!(!neighbors.is_empty());
    }

    #[test]
    fn test_random_topology() {
        let manager = TopologyManager::new(Topology::Random, 10).unwrap();
        let neighbors = manager.get_neighbors(0);
        assert!(neighbors.len() <= 10);
    }

    #[test]
    fn test_adapt_topology() {
        let mut manager = TopologyManager::new(Topology::Ring, 10).unwrap();
        manager.adapt_topology(0.8).unwrap();
        manager.adapt_topology(0.2).unwrap();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Constraint Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod constraint_tests {
    use super::*;

    #[test]
    fn test_inequality_constraint() {
        let constraint = Constraint::Inequality { weight: 1.0 };
        match constraint {
            Constraint::Inequality { weight } => assert_eq!(weight, 1.0),
            _ => panic!("Wrong constraint type"),
        }
    }

    #[test]
    fn test_equality_constraint() {
        let constraint = Constraint::Equality {
            tolerance: 0.001,
            weight: 10.0,
        };
        match constraint {
            Constraint::Equality { tolerance, weight } => {
                assert_eq!(tolerance, 0.001);
                assert_eq!(weight, 10.0);
            }
            _ => panic!("Wrong constraint type"),
        }
    }

    #[test]
    fn test_collision_constraint() {
        let constraint = Constraint::Collision {
            min_distance: 5.0,
            weight: 100.0,
        };
        match constraint {
            Constraint::Collision {
                min_distance,
                weight,
            } => {
                assert_eq!(min_distance, 5.0);
                assert_eq!(weight, 100.0);
            }
            _ => panic!("Wrong constraint type"),
        }
    }

    #[test]
    fn test_energy_constraint() {
        let constraint = Constraint::Energy {
            max_energy: 1000.0,
            weight: 50.0,
        };
        match constraint {
            Constraint::Energy { max_energy, weight } => {
                assert_eq!(max_energy, 1000.0);
                assert_eq!(weight, 50.0);
            }
            _ => panic!("Wrong constraint type"),
        }
    }

    #[test]
    fn test_constraint_clone() {
        let constraint = Constraint::Inequality { weight: 2.5 };
        let _cloned = constraint.clone();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PenaltyMethod Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod penalty_method_tests {
    use super::*;

    #[test]
    fn test_static_penalty() {
        let method = PenaltyMethod::Static;
        match method {
            PenaltyMethod::Static => assert!(true),
            _ => panic!("Wrong method"),
        }
    }

    #[test]
    fn test_dynamic_penalty() {
        let method = PenaltyMethod::Dynamic { growth_rate: 1.5 };
        match method {
            PenaltyMethod::Dynamic { growth_rate } => assert_eq!(growth_rate, 1.5),
            _ => panic!("Wrong method"),
        }
    }

    #[test]
    fn test_adaptive_penalty() {
        let method = PenaltyMethod::Adaptive;
        match method {
            PenaltyMethod::Adaptive => assert!(true),
            _ => panic!("Wrong method"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ConstraintHandler Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod constraint_handler_tests {
    use super::*;

    #[test]
    fn test_new() {
        let _handler = ConstraintHandler::new(PenaltyMethod::Static);
    }

    #[test]
    fn test_add_constraint() {
        let mut handler = ConstraintHandler::new(PenaltyMethod::Static);
        let constraint = Constraint::Inequality { weight: 1.0 };
        let result = handler.add_constraint(constraint);
        assert!(result.is_ok());
    }

    #[test]
    fn test_add_multiple_constraints() {
        let mut handler = ConstraintHandler::new(PenaltyMethod::Adaptive);

        handler
            .add_constraint(Constraint::Inequality { weight: 1.0 })
            .unwrap();
        handler
            .add_constraint(Constraint::Equality {
                tolerance: 0.01,
                weight: 5.0,
            })
            .unwrap();
        handler
            .add_constraint(Constraint::Collision {
                min_distance: 3.0,
                weight: 10.0,
            })
            .unwrap();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SharingStrategy Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod sharing_strategy_tests {
    use super::*;

    #[test]
    fn test_none_strategy() {
        let strategy = SharingStrategy::None;
        match strategy {
            SharingStrategy::None => assert!(true),
            _ => panic!("Wrong strategy"),
        }
    }

    #[test]
    fn test_best_solution_strategy() {
        let strategy = SharingStrategy::BestSolution;
        match strategy {
            SharingStrategy::BestSolution => assert!(true),
            _ => panic!("Wrong strategy"),
        }
    }

    #[test]
    fn test_migration_strategy() {
        let strategy = SharingStrategy::Migration { rate: 0.1 };
        match strategy {
            SharingStrategy::Migration { rate } => assert_eq!(rate, 0.1),
            _ => panic!("Wrong strategy"),
        }
    }

    #[test]
    fn test_incremental_strategy() {
        let strategy = SharingStrategy::Incremental;
        match strategy {
            SharingStrategy::Incremental => assert!(true),
            _ => panic!("Wrong strategy"),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MultiSwarmCoordinator Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod multi_swarm_coordinator_tests {
    use super::*;

    #[test]
    fn test_new() {
        let coordinator = MultiSwarmCoordinator::new(SharingStrategy::BestSolution, 10);
        assert_eq!(coordinator.num_sub_swarms(), 0);
    }

    #[test]
    fn test_different_strategies() {
        let strategies = [
            SharingStrategy::None,
            SharingStrategy::BestSolution,
            SharingStrategy::Migration { rate: 0.2 },
            SharingStrategy::Incremental,
        ];

        for strategy in strategies {
            let coordinator = MultiSwarmCoordinator::new(strategy, 5);
            assert_eq!(coordinator.num_sub_swarms(), 0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_constrained_optimization_setup() {
        let mut handler = ConstraintHandler::new(PenaltyMethod::Dynamic { growth_rate: 1.2 });

        handler
            .add_constraint(Constraint::Inequality { weight: 1.0 })
            .unwrap();
        handler
            .add_constraint(Constraint::Collision {
                min_distance: 5.0,
                weight: 100.0,
            })
            .unwrap();

        let topology = TopologyManager::new(Topology::Star, 20).unwrap();
        let neighbors = topology.get_neighbors(0);
        assert!(!neighbors.is_empty());
    }

    #[test]
    fn test_multi_swarm_setup() {
        let coordinator = MultiSwarmCoordinator::new(SharingStrategy::BestSolution, 10);
        assert_eq!(coordinator.num_sub_swarms(), 0);
    }
}
