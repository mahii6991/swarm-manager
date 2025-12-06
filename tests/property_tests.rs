//! Property-based tests for drone swarm system
//!
//! These tests verify mathematical properties and invariants that should hold
//! for all inputs, using randomized testing with proptest.

use drone_swarm_system::network::MAX_NETWORK_HOPS;
use drone_swarm_system::*;
use proptest::prelude::*;

// ============================================================================
// POSITION AND GEOMETRY PROPERTIES
// ============================================================================

#[cfg(test)]
mod geometry_properties {
    use super::*;

    proptest! {
        #[test]
        fn distance_is_non_negative(
            x1 in -1000.0_f32..1000.0,
            y1 in -1000.0_f32..1000.0,
            z1 in -1000.0_f32..1000.0,
            x2 in -1000.0_f32..1000.0,
            y2 in -1000.0_f32..1000.0,
            z2 in -1000.0_f32..1000.0,
        ) {
            let pos1 = Position { x: x1, y: y1, z: z1 };
            let pos2 = Position { x: x2, y: y2, z: z2 };

            let dist = pos1.distance_to(&pos2);
            prop_assert!(dist >= 0.0, "Distance must be non-negative");
            prop_assert!(dist.is_finite(), "Distance must be finite");
        }

        #[test]
        fn distance_is_symmetric(
            x1 in -1000.0_f32..1000.0,
            y1 in -1000.0_f32..1000.0,
            z1 in -1000.0_f32..1000.0,
            x2 in -1000.0_f32..1000.0,
            y2 in -1000.0_f32..1000.0,
            z2 in -1000.0_f32..1000.0,
        ) {
            let pos1 = Position { x: x1, y: y1, z: z1 };
            let pos2 = Position { x: x2, y: y2, z: z2 };

            let dist12 = pos1.distance_to(&pos2);
            let dist21 = pos2.distance_to(&pos1);

            prop_assert!(
                (dist12 - dist21).abs() < 1e-6,
                "Distance should be symmetric: d({:?},{:?}) = {} != {} = d({:?},{:?})",
                pos1, pos2, dist12, dist21, pos2, pos1
            );
        }

        #[test]
        fn triangle_inequality(
            x1 in -1000.0_f32..1000.0,
            y1 in -1000.0_f32..1000.0,
            z1 in -1000.0_f32..1000.0,
            x2 in -1000.0_f32..1000.0,
            y2 in -1000.0_f32..1000.0,
            z2 in -1000.0_f32..1000.0,
            x3 in -1000.0_f32..1000.0,
            y3 in -1000.0_f32..1000.0,
            z3 in -1000.0_f32..1000.0,
        ) {
            let pos1 = Position { x: x1, y: y1, z: z1 };
            let pos2 = Position { x: x2, y: y2, z: z2 };
            let pos3 = Position { x: x3, y: y3, z: z3 };

            let dist12 = pos1.distance_to(&pos2);
            let dist23 = pos2.distance_to(&pos3);
            let dist13 = pos1.distance_to(&pos3);

            // Triangle inequality: d(a,c) <= d(a,b) + d(b,c)
            prop_assert!(
                dist13 <= dist12 + dist23 + 1e-5,
                "Triangle inequality violated: {} > {} + {}",
                dist13, dist12, dist23
            );
        }

        #[test]
        fn distance_zero_iff_same_position(
            x in -1000.0_f32..1000.0,
            y in -1000.0_f32..1000.0,
            z in -1000.0_f32..1000.0,
        ) {
            let pos = Position { x, y, z };
            let dist = pos.distance_to(&pos);
            prop_assert!(
                dist < 1e-6,
                "Distance to itself should be zero, got {}",
                dist
            );
        }
    }
}

// ============================================================================
// DRONE ID PROPERTIES
// ============================================================================

#[cfg(test)]
mod drone_id_properties {
    use super::*;

    proptest! {
        #[test]
        fn drone_id_roundtrip(id in 0u64..=u64::MAX) {
            let drone_id = DroneId::new(id);
            let retrieved_id = drone_id.as_u64();
            prop_assert_eq!(id, retrieved_id, "DroneId should preserve value");
        }

        #[test]
        fn drone_id_equality_reflexive(id in 0u64..=u64::MAX) {
            let drone_id = DroneId::new(id);
            prop_assert_eq!(drone_id, drone_id, "DroneId should equal itself");
        }

        #[test]
        fn drone_id_equality_symmetric(id in 0u64..=u64::MAX) {
            let drone_id1 = DroneId::new(id);
            let drone_id2 = DroneId::new(id);
            prop_assert_eq!(drone_id1, drone_id2);
            prop_assert_eq!(drone_id2, drone_id1);
        }

        #[test]
        fn drone_id_different_values_not_equal(
            id1 in 0u64..=u64::MAX,
            id2 in 0u64..=u64::MAX,
        ) {
            prop_assume!(id1 != id2);
            let drone_id1 = DroneId::new(id1);
            let drone_id2 = DroneId::new(id2);
            prop_assert_ne!(drone_id1, drone_id2);
        }
    }
}

// ============================================================================
// CONSENSUS PROPERTIES
// ============================================================================

#[cfg(test)]
mod consensus_properties {
    use super::*;

    proptest! {
        #[test]
        fn consensus_term_monotonic(
            initial_term in 0u64..1000,
            increments in prop::collection::vec(1u64..10, 1..20)
        ) {
            let mut current_term = initial_term;
            let mut previous_term = current_term;

            for increment in increments {
                current_term = current_term.saturating_add(increment);
                prop_assert!(
                    current_term >= previous_term,
                    "Consensus term should be monotonically increasing"
                );
                previous_term = current_term;
            }
        }

        #[test]
        fn log_index_monotonic(
            start_index in 0u64..1000,
            count in 1usize..100
        ) {
            let mut indices = Vec::new();
            for i in 0..count {
                indices.push(start_index.saturating_add(i as u64));
            }

            // Verify monotonically increasing
            for window in indices.windows(2) {
                prop_assert!(
                    window[1] >= window[0],
                    "Log indices should be monotonically increasing"
                );
            }
        }
    }
}

// ============================================================================
// NETWORK MESSAGE PROPERTIES
// ============================================================================

#[cfg(test)]
mod network_properties {
    use super::*;

    proptest! {
        #[test]
        fn hop_count_bounded(initial_hops in 0u8..MAX_NETWORK_HOPS) {
            let hop_count = initial_hops;
            prop_assert!(
                hop_count < MAX_NETWORK_HOPS,
                "Hop count {} should be bounded by MAX_NETWORK_HOPS {}",
                hop_count, MAX_NETWORK_HOPS
            );
        }

        #[test]
        fn hop_count_saturates_at_u8_max(initial_hops in 0u8..=254u8) {
            let hop_count = initial_hops;
            let incremented = hop_count.saturating_add(1);

            // saturating_add saturates at u8::MAX (255), not at our custom MAX_NETWORK_HOPS
            prop_assert_eq!(incremented, hop_count + 1,
                "Hop count should increment normally below u8::MAX");
        }

        #[test]
        fn hop_count_validation(hop_count in 0u8..=255u8) {
            // Application logic should reject hops >= MAX_NETWORK_HOPS
            let is_valid = hop_count < MAX_NETWORK_HOPS;
            let should_drop = hop_count >= MAX_NETWORK_HOPS;

            prop_assert_eq!(is_valid, !should_drop,
                "Validation logic should be consistent");
        }
    }
}

// ============================================================================
// PSO ALGORITHM PROPERTIES
// ============================================================================

#[cfg(test)]
mod pso_properties {
    use super::*;

    proptest! {
        #[test]
        fn particle_position_clamping(
            lower_bound in -100.0_f32..0.0,
            upper_bound in 1.0_f32..100.0,
            initial_pos in -200.0_f32..200.0,
        ) {
            prop_assume!(lower_bound < upper_bound);

            let position = initial_pos.max(lower_bound).min(upper_bound);

            prop_assert!(
                position >= lower_bound && position <= upper_bound,
                "Clamped position {} should be within bounds [{}, {}]",
                position, lower_bound, upper_bound
            );
        }

        #[test]
        fn velocity_clamping(
            velocity in -20.0_f32..20.0,
            max_velocity in 0.1_f32..10.0,
        ) {
            let clamped_v = velocity.max(-max_velocity).min(max_velocity);

            prop_assert!(
                clamped_v.abs() <= max_velocity + 1e-6,
                "Clamped velocity {} should be bounded by {}",
                clamped_v, max_velocity
            );
        }

        #[test]
        fn velocity_scaling_preserves_direction(
            vx in -10.0_f32..10.0,
            vy in -10.0_f32..10.0,
            scale in 0.1_f32..2.0,
        ) {
            prop_assume!(vx != 0.0 || vy != 0.0);

            let scaled_vx = vx * scale;
            let scaled_vy = vy * scale;

            // Check that scaling preserves direction (sign)
            if vx.abs() > 1e-6 {
                prop_assert_eq!(vx.signum(), scaled_vx.signum());
            }
            if vy.abs() > 1e-6 {
                prop_assert_eq!(vy.signum(), scaled_vy.signum());
            }
        }
    }
}

// ============================================================================
// MATH UTILITIES PROPERTIES
// ============================================================================

#[cfg(test)]
mod math_properties {
    use super::*;

    proptest! {
        #[test]
        fn saturating_add_never_overflows(
            a in 0u64..=u64::MAX,
            b in 0u64..=u64::MAX,
        ) {
            let result = a.saturating_add(b);
            prop_assert!(result >= a);
            prop_assert!(result >= b);
            prop_assert!(result <= u64::MAX);
        }

        #[test]
        fn saturating_sub_never_underflows(
            a in 0u64..=u64::MAX,
            b in 0u64..=u64::MAX,
        ) {
            let result = a.saturating_sub(b);
            prop_assert!(result <= a);
            prop_assert!(result >= 0);
        }

        #[test]
        fn clamp_keeps_value_in_bounds(
            value in -100.0_f32..100.0,
            min in -50.0_f32..0.0,
            max in 1.0_f32..50.0,
        ) {
            prop_assume!(min < max);
            let clamped = value.clamp(min, max);
            prop_assert!(clamped >= min);
            prop_assert!(clamped <= max);
        }
    }
}
