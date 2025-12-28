//! Comprehensive tests for the Grey Wolf Optimizer (GWO) module

use drone_swarm_system::gwo::*;

// ═══════════════════════════════════════════════════════════════════════════
// GWOVariant Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod gwo_variant_tests {
    use super::*;

    #[test]
    fn test_variant_standard() {
        let variant = GWOVariant::Standard;
        assert_eq!(variant, GWOVariant::Standard);
    }

    #[test]
    fn test_variant_improved() {
        let variant = GWOVariant::Improved;
        assert_eq!(variant, GWOVariant::Improved);
    }

    #[test]
    fn test_variant_hybrid() {
        let variant = GWOVariant::Hybrid;
        assert_eq!(variant, GWOVariant::Hybrid);
    }

    #[test]
    fn test_variant_chaotic() {
        let variant = GWOVariant::Chaotic;
        assert_eq!(variant, GWOVariant::Chaotic);
    }

    #[test]
    fn test_variant_equality() {
        assert_eq!(GWOVariant::Standard, GWOVariant::Standard);
        assert_ne!(GWOVariant::Standard, GWOVariant::Improved);
    }

    #[test]
    fn test_variant_copy() {
        let variant = GWOVariant::Hybrid;
        let copied = variant; // Copy trait
        assert_eq!(variant, copied);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GWOConfig Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod gwo_config_tests {
    use super::*;

    #[test]
    fn test_default() {
        let config = GWOConfig::default();
        assert_eq!(config.variant, GWOVariant::Improved);
        assert_eq!(config.num_wolves, 30);
        assert_eq!(config.max_iterations, 100);
        assert_eq!(config.dimensions, 10);
        assert!(config.a_decay);
        assert!(config.adaptive);
    }

    #[test]
    fn test_custom_config() {
        let config = GWOConfig {
            variant: GWOVariant::Standard,
            num_wolves: 20,
            max_iterations: 200,
            dimensions: 5,
            a_decay: false,
            adaptive: false,
        };
        assert_eq!(config.variant, GWOVariant::Standard);
        assert_eq!(config.num_wolves, 20);
    }

    #[test]
    fn test_config_clone() {
        let config = GWOConfig::default();
        let cloned = config.clone();
        assert_eq!(config.max_iterations, cloned.max_iterations);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Bounds Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod bounds_tests {
    use super::*;

    #[test]
    fn test_uniform_bounds() {
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();
        assert_eq!(bounds.lower.len(), 5);
        assert_eq!(bounds.upper.len(), 5);
        assert_eq!(bounds.lower[0], -10.0);
        assert_eq!(bounds.upper[0], 10.0);
    }

    #[test]
    fn test_bounds_clone() {
        let bounds = Bounds::uniform(3, 0.0, 1.0).unwrap();
        let cloned = bounds.clone();
        assert_eq!(bounds.lower.len(), cloned.lower.len());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Wolf Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod wolf_tests {
    use super::*;

    #[test]
    fn test_wolf_new() {
        let wolf = Wolf::new(5, 0).unwrap();
        assert_eq!(wolf.position.len(), 5);
        assert_eq!(wolf.id, 0);
        assert_eq!(wolf.fitness, f32::INFINITY);
    }

    #[test]
    fn test_wolf_initialize() {
        let mut wolf = Wolf::new(3, 1).unwrap();
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();
        wolf.initialize(&bounds).unwrap();

        for i in 0..3 {
            assert!(wolf.position[i] >= -5.0);
            assert!(wolf.position[i] <= 5.0);
        }
    }

    #[test]
    fn test_wolf_update_position() {
        let mut wolf = Wolf::new(3, 0).unwrap();
        let bounds = Bounds::uniform(3, -10.0, 10.0).unwrap();
        let new_pos = [1.0, 2.0, 3.0];

        wolf.update_position(&new_pos, &bounds).unwrap();

        assert_eq!(wolf.position[0], 1.0);
        assert_eq!(wolf.position[1], 2.0);
        assert_eq!(wolf.position[2], 3.0);
    }

    #[test]
    fn test_wolf_clone() {
        let wolf = Wolf::new(5, 42).unwrap();
        let cloned = wolf.clone();
        assert_eq!(wolf.id, cloned.id);
        assert_eq!(wolf.position.len(), cloned.position.len());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// GWOOptimizer Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod gwo_optimizer_tests {
    use super::*;

    fn sphere_function(x: &[f32]) -> f32 {
        x.iter().map(|&xi| xi * xi).sum()
    }

    #[test]
    fn test_optimizer_new() {
        let config = GWOConfig {
            variant: GWOVariant::Standard,
            num_wolves: 10,
            max_iterations: 50,
            dimensions: 3,
            a_decay: true,
            adaptive: false,
        };
        let bounds = Bounds::uniform(3, -10.0, 10.0).unwrap();

        let optimizer = GWOOptimizer::new(config, bounds);
        assert!(optimizer.is_ok());
    }

    #[test]
    fn test_optimizer_optimize() {
        let config = GWOConfig {
            variant: GWOVariant::Standard,
            num_wolves: 10,
            max_iterations: 20,
            dimensions: 3,
            a_decay: true,
            adaptive: false,
        };
        let bounds = Bounds::uniform(3, -5.0, 5.0).unwrap();

        let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();
        let result = optimizer.optimize(sphere_function);
        assert!(result.is_ok());

        let best = result.unwrap();
        assert!(best.fitness.is_finite());
    }

    #[test]
    fn test_optimizer_different_variants() {
        let variants = [
            GWOVariant::Standard,
            GWOVariant::Improved,
            GWOVariant::Hybrid,
            GWOVariant::Chaotic,
        ];

        for variant in variants {
            let config = GWOConfig {
                variant,
                num_wolves: 10,
                max_iterations: 10,
                dimensions: 2,
                a_decay: true,
                adaptive: true,
            };
            let bounds = Bounds::uniform(2, -5.0, 5.0).unwrap();

            let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();
            let result = optimizer.optimize(sphere_function);
            assert!(result.is_ok());
        }
    }

    #[test]
    fn test_optimizer_convergence() {
        let config = GWOConfig {
            variant: GWOVariant::Improved,
            num_wolves: 20,
            max_iterations: 50,
            dimensions: 2,
            a_decay: true,
            adaptive: true,
        };
        let bounds = Bounds::uniform(2, -10.0, 10.0).unwrap();

        let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();
        let result = optimizer.optimize(sphere_function).unwrap();

        // Should find a reasonably good solution
        assert!(result.fitness < 100.0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_optimization_workflow() {
        // Configure
        let config = GWOConfig {
            variant: GWOVariant::Improved,
            num_wolves: 15,
            max_iterations: 30,
            dimensions: 5,
            a_decay: true,
            adaptive: true,
        };

        // Define bounds
        let bounds = Bounds::uniform(5, -10.0, 10.0).unwrap();

        // Create optimizer
        let mut optimizer = GWOOptimizer::new(config, bounds).unwrap();

        // Define cost function
        let cost_fn = |x: &[f32]| x.iter().map(|&xi| xi * xi).sum::<f32>();

        // Optimize
        let result = optimizer.optimize(cost_fn).unwrap();

        // Verify result
        assert_eq!(result.position.len(), 5);
        assert!(result.fitness.is_finite());
    }
}
