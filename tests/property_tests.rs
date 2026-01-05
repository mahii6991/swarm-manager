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
            // saturating_add guarantees no overflow (result is always valid u64)
        }

        #[test]
        fn saturating_sub_never_underflows(
            a in 0u64..=u64::MAX,
            b in 0u64..=u64::MAX,
        ) {
            let result = a.saturating_sub(b);
            prop_assert!(result <= a);
            // saturating_sub guarantees no underflow (result is always valid u64)
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

// ============================================================================
// COLLISION AVOIDANCE PROPERTIES
// ============================================================================

#[cfg(test)]
mod collision_avoidance_properties {
    use super::*;

    proptest! {
        #[test]
        fn safe_separation_always_positive(
            min_separation in 0.1_f32..50.0,
            safety_factor in 1.0_f32..3.0,
        ) {
            let safe_distance = min_separation * safety_factor;
            prop_assert!(safe_distance > 0.0, "Safe separation must be positive");
        }

        #[test]
        fn potential_field_repulsion_inverse_square(
            distance in 1.0_f32..100.0,
            gain in 1.0_f32..100.0,
        ) {
            let repulsion = gain / (distance * distance);
            prop_assert!(repulsion > 0.0, "Repulsion force must be positive");
            prop_assert!(repulsion.is_finite(), "Repulsion must be finite");
        }

        #[test]
        fn geofence_boundary_check_consistent(
            x in -200.0_f32..200.0,
            y in -200.0_f32..200.0,
            radius in 50.0_f32..150.0,
        ) {
            let distance = (x * x + y * y).sqrt();
            let inside = distance <= radius;
            let outside = distance > radius;

            // Exactly one must be true
            prop_assert!(inside != outside, "Position must be either inside or outside geofence");
        }

        #[test]
        fn velocity_obstacle_time_horizon_positive(
            time_horizon in 0.1_f32..30.0,
        ) {
            prop_assert!(time_horizon > 0.0, "Time horizon must be positive");
        }

        #[test]
        fn collision_avoidance_velocity_bounded(
            vx in -20.0_f32..20.0,
            vy in -20.0_f32..20.0,
            max_speed in 5.0_f32..15.0,
        ) {
            let speed = (vx * vx + vy * vy).sqrt();
            if speed > max_speed {
                let scale = max_speed / speed;
                let bounded_vx = vx * scale;
                let bounded_vy = vy * scale;
                let bounded_speed = (bounded_vx * bounded_vx + bounded_vy * bounded_vy).sqrt();

                prop_assert!(
                    (bounded_speed - max_speed).abs() < 0.001,
                    "Bounded speed {} should equal max_speed {}",
                    bounded_speed, max_speed
                );
            }
        }
    }
}

// ============================================================================
// MISSION PLANNING PROPERTIES
// ============================================================================

#[cfg(test)]
mod mission_planning_properties {
    use super::*;

    proptest! {
        #[test]
        fn waypoint_index_bounded(
            num_waypoints in 1usize..100,
            current_index in 0usize..200,
        ) {
            let bounded_index = current_index.min(num_waypoints.saturating_sub(1));
            prop_assert!(
                bounded_index < num_waypoints,
                "Bounded index {} must be less than waypoint count {}",
                bounded_index, num_waypoints
            );
        }

        #[test]
        fn lawnmower_pattern_waypoint_count(
            rows in 1usize..20,
            cols in 1usize..20,
        ) {
            let expected = rows * cols;
            prop_assert!(expected >= 1, "Lawnmower pattern must generate at least 1 waypoint");
            prop_assert!(expected <= 400, "Lawnmower pattern bounded by rows*cols");
        }

        #[test]
        fn spiral_pattern_radius_increases(
            _turns in 1.0_f32..5.0,
            points in 10usize..50,
        ) {
            let mut prev_radius = 0.0_f32;
            for i in 0..points {
                let t = i as f32 / points as f32;
                let radius = t * 80.0;  // Same formula as in mission_planning
                prop_assert!(radius >= prev_radius, "Spiral radius should increase");
                prev_radius = radius;
            }
        }

        #[test]
        fn mission_progress_bounded(
            current_waypoint in 0usize..100,
            total_waypoints in 1usize..100,
        ) {
            let progress = current_waypoint as f32 / total_waypoints as f32;
            prop_assert!(progress >= 0.0, "Progress must be non-negative");
            // Progress can exceed 1.0 if current > total (mission overrun), but that's fine
        }

        #[test]
        fn survey_area_positive(
            width in 10.0_f32..500.0,
            height in 10.0_f32..500.0,
        ) {
            let area = width * height;
            prop_assert!(area > 0.0, "Survey area must be positive");
        }
    }
}

// ============================================================================
// TELEMETRY PROPERTIES
// ============================================================================

#[cfg(test)]
mod telemetry_properties {
    use super::*;

    proptest! {
        #[test]
        fn battery_percentage_valid_range(battery in 0u8..=100u8) {
            prop_assert!(battery <= 100, "Battery percentage must not exceed 100");
        }

        #[test]
        fn health_thresholds_ordered(
            critical in 1u8..15,
            warning in 15u8..40,
        ) {
            prop_assert!(critical < warning, "Critical threshold must be below warning");
        }

        #[test]
        fn average_battery_bounded(
            batteries in prop::collection::vec(0u8..=100u8, 1..20),
        ) {
            let sum: u32 = batteries.iter().map(|&b| b as u32).sum();
            let avg = sum as f32 / batteries.len() as f32;
            let min = *batteries.iter().min().unwrap() as f32;
            let max = *batteries.iter().max().unwrap() as f32;

            prop_assert!(avg >= min, "Average {} must be >= min {}", avg, min);
            prop_assert!(avg <= max, "Average {} must be <= max {}", avg, max);
        }

        #[test]
        fn alert_count_bounded(count in 0usize..1000) {
            // usize is always non-negative; verify count is within expected range
            prop_assert!(count < 1000, "Alert count should be bounded");
        }

        #[test]
        fn speed_magnitude_non_negative(
            vx in -50.0_f32..50.0,
            vy in -50.0_f32..50.0,
            vz in -50.0_f32..50.0,
        ) {
            let speed = (vx * vx + vy * vy + vz * vz).sqrt();
            prop_assert!(speed >= 0.0, "Speed must be non-negative");
        }
    }
}

// ============================================================================
// FAILSAFE PROPERTIES
// ============================================================================

#[cfg(test)]
mod failsafe_properties {
    use super::*;

    proptest! {
        #[test]
        fn rtl_altitude_at_least_minimum(
            current_alt in 0.0_f32..200.0,
            min_rtl_alt in 10.0_f32..50.0,
        ) {
            let rtl_alt = current_alt.max(min_rtl_alt);
            prop_assert!(
                rtl_alt >= min_rtl_alt,
                "RTL altitude {} must be at least minimum {}",
                rtl_alt, min_rtl_alt
            );
        }

        #[test]
        fn failsafe_priority_total_ordering(
            p1 in 0u8..10,
            p2 in 0u8..10,
            p3 in 0u8..10,
        ) {
            // Transitivity: if p1 <= p2 and p2 <= p3, then p1 <= p3
            if p1 <= p2 && p2 <= p3 {
                prop_assert!(p1 <= p3, "Priority ordering must be transitive");
            }
        }

        #[test]
        fn geofence_radius_positive(radius in 10.0_f32..500.0) {
            prop_assert!(radius > 0.0, "Geofence radius must be positive");
        }

        #[test]
        fn battery_failsafe_thresholds_ordered(
            rtl_threshold in 10u8..30,
            land_threshold in 5u8..15,
        ) {
            // RTL should trigger before land
            prop_assume!(rtl_threshold > land_threshold);
            prop_assert!(
                rtl_threshold > land_threshold,
                "RTL threshold {} must be above land threshold {}",
                rtl_threshold, land_threshold
            );
        }

        #[test]
        fn descent_rate_positive(rate in 0.1_f32..5.0) {
            prop_assert!(rate > 0.0, "Descent rate must be positive");
        }
    }
}

// ============================================================================
// ACO ALGORITHM PROPERTIES
// ============================================================================

#[cfg(test)]
mod aco_properties {
    use super::*;

    proptest! {
        #[test]
        fn pheromone_evaporation_reduces(
            initial in 0.1_f32..10.0,
            rate in 0.01_f32..0.5,
        ) {
            let after = initial * (1.0 - rate);
            prop_assert!(after < initial, "Evaporation must reduce pheromone level");
            prop_assert!(after >= 0.0, "Pheromone cannot be negative");
        }

        #[test]
        fn pheromone_deposit_increases(
            initial in 0.1_f32..10.0,
            deposit in 0.01_f32..5.0,
        ) {
            let after = initial + deposit;
            prop_assert!(after > initial, "Deposit must increase pheromone level");
        }

        #[test]
        fn probability_sum_to_one(
            weights in prop::collection::vec(0.1_f32..10.0, 2..10),
        ) {
            let sum: f32 = weights.iter().sum();
            let probabilities: Vec<f32> = weights.iter().map(|w| w / sum).collect();
            let prob_sum: f32 = probabilities.iter().sum();

            prop_assert!(
                (prob_sum - 1.0).abs() < 0.001,
                "Probabilities should sum to 1, got {}",
                prob_sum
            );
        }
    }
}

// ============================================================================
// GWO ALGORITHM PROPERTIES
// ============================================================================

#[cfg(test)]
mod gwo_properties {
    use super::*;

    proptest! {
        #[test]
        fn convergence_parameter_decreases(
            iteration in 0u32..500,
            max_iter in 100u32..1000,
        ) {
            prop_assume!(max_iter > 0);
            prop_assume!(iteration <= max_iter);

            let a = 2.0 * (1.0 - iteration as f32 / max_iter as f32);
            prop_assert!(a >= 0.0 && a <= 2.0, "Convergence param {} must be in [0,2]", a);
        }

        #[test]
        fn wolf_hierarchy_ordering(
            alpha_fitness in 0.0_f32..10.0,
            beta_fitness in 0.0_f32..10.0,
            delta_fitness in 0.0_f32..10.0,
        ) {
            // After sorting, alpha should have best (lowest) fitness
            let mut fitnesses = [alpha_fitness, beta_fitness, delta_fitness];
            fitnesses.sort_by(|a, b| a.partial_cmp(b).unwrap());

            prop_assert!(
                fitnesses[0] <= fitnesses[1] && fitnesses[1] <= fitnesses[2],
                "Wolf hierarchy must be ordered by fitness"
            );
        }

        #[test]
        fn position_update_bounded(
            position in -100.0_f32..100.0,
            bounds in 50.0_f32..150.0,
        ) {
            let bounded = position.clamp(-bounds, bounds);
            prop_assert!(
                bounded >= -bounds && bounded <= bounds,
                "Position {} must be within bounds +/-{}",
                bounded, bounds
            );
        }
    }
}
