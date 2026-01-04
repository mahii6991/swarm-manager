//! Comprehensive tests for the mission planning system
//!
//! Tests cover:
//! - Waypoint creation and configuration
//! - Mission state machine transitions
//! - Survey pattern generation
//! - Edge cases and boundary conditions

use drone_swarm_system::mission_planning::{
    generate_survey_mission, survey_to_mission, AcceptanceMode, Mission, MissionState,
    SurveyArea, SurveyPattern, Waypoint, WaypointAction, WaypointType, MAX_WAYPOINTS,
};

// ============================================================================
// Waypoint Creation Tests
// ============================================================================

mod waypoint_tests {
    use super::*;

    #[test]
    fn test_goto_waypoint() {
        let wp = Waypoint::goto([10.0, 20.0, -30.0]);
        assert_eq!(wp.wp_type, WaypointType::Goto);
        assert_eq!(wp.position, [10.0, 20.0, -30.0]);
        assert!(wp.heading.is_nan()); // Don't care heading
        assert!(wp.enabled);
    }

    #[test]
    fn test_goto_with_heading() {
        let wp = Waypoint::goto_with_heading([10.0, 20.0, -30.0], 90.0);
        assert_eq!(wp.wp_type, WaypointType::Goto);
        assert!((wp.heading - 90.0_f32.to_radians()).abs() < 0.001);
        assert_eq!(wp.acceptance, AcceptanceMode::StopAt);
    }

    #[test]
    fn test_takeoff_waypoint() {
        let wp = Waypoint::takeoff(15.0);
        assert_eq!(wp.wp_type, WaypointType::Takeoff);
        assert_eq!(wp.position[2], -15.0); // Altitude is negative z in NED
    }

    #[test]
    fn test_takeoff_negative_altitude() {
        // Should handle negative altitude gracefully (use abs)
        let wp = Waypoint::takeoff(-15.0);
        assert_eq!(wp.position[2], -15.0); // Still negative z
    }

    #[test]
    fn test_land_waypoint() {
        let wp = Waypoint::land([100.0, 200.0, -50.0]);
        assert_eq!(wp.wp_type, WaypointType::Land);
        assert_eq!(wp.position[0], 100.0);
        assert_eq!(wp.position[1], 200.0);
        assert_eq!(wp.position[2], 0.0); // Z forced to 0 for landing
    }

    #[test]
    fn test_loiter_waypoint() {
        let wp = Waypoint::loiter([10.0, 10.0, -20.0], 30.0);
        assert_eq!(wp.wp_type, WaypointType::Loiter);
        assert_eq!(wp.loiter_time, 30.0);
        match wp.action {
            WaypointAction::Hover { duration_secs } => assert_eq!(duration_secs, 30.0),
            _ => panic!("Expected Hover action"),
        }
    }

    #[test]
    fn test_rtl_waypoint() {
        let wp = Waypoint::rtl();
        assert_eq!(wp.wp_type, WaypointType::ReturnToLaunch);
        assert_eq!(wp.acceptance, AcceptanceMode::StopAt);
    }

    #[test]
    fn test_photo_waypoint() {
        let wp = Waypoint::photo([50.0, 50.0, -25.0]);
        assert_eq!(wp.wp_type, WaypointType::Goto);
        assert_eq!(wp.action, WaypointAction::TakePhoto);
        assert_eq!(wp.acceptance, AcceptanceMode::StopAt);
    }

    #[test]
    fn test_waypoint_with_speed() {
        let wp = Waypoint::goto([10.0, 0.0, -10.0]).with_speed(8.0);
        assert_eq!(wp.speed, 8.0);
    }

    #[test]
    fn test_waypoint_with_acceptance() {
        let wp = Waypoint::goto([10.0, 0.0, -10.0]).with_acceptance(3.0);
        match wp.acceptance {
            AcceptanceMode::Custom { radius } => assert_eq!(radius, 3.0),
            _ => panic!("Expected Custom acceptance mode"),
        }
    }

    #[test]
    fn test_acceptance_radius() {
        let pass_through = Waypoint::goto([0.0, 0.0, 0.0]);
        assert_eq!(pass_through.acceptance_radius(), 5.0);

        let stop_at = Waypoint::goto_with_heading([0.0, 0.0, 0.0], 0.0);
        assert_eq!(stop_at.acceptance_radius(), 1.0);

        let custom = Waypoint::goto([0.0, 0.0, 0.0]).with_acceptance(7.5);
        assert_eq!(custom.acceptance_radius(), 7.5);
    }
}

// ============================================================================
// Mission State Machine Tests
// ============================================================================

mod mission_state_tests {
    use super::*;

    #[test]
    fn test_new_mission() {
        let mission = Mission::new();
        assert_eq!(mission.id, 0);
        assert_eq!(mission.state(), MissionState::Idle);
        assert_eq!(mission.waypoint_count(), 0);
        assert_eq!(mission.current_index(), 0);
    }

    #[test]
    fn test_mission_with_id() {
        let mission = Mission::with_id(42);
        assert_eq!(mission.id, 42);
    }

    #[test]
    fn test_add_waypoints() {
        let mut mission = Mission::new();
        assert!(mission.add_waypoint(Waypoint::takeoff(10.0)));
        assert!(mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0])));
        assert!(mission.add_waypoint(Waypoint::land([0.0, 0.0, 0.0])));

        assert_eq!(mission.waypoint_count(), 3);
    }

    #[test]
    fn test_max_waypoints() {
        let mut mission = Mission::new();

        for i in 0..MAX_WAYPOINTS {
            let pos = [i as f32, 0.0, -10.0];
            assert!(mission.add_waypoint(Waypoint::goto(pos)));
        }

        assert_eq!(mission.waypoint_count(), MAX_WAYPOINTS);

        // Adding one more should fail
        assert!(!mission.add_waypoint(Waypoint::goto([0.0, 0.0, 0.0])));
    }

    #[test]
    fn test_mission_start() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        mission.start();
        assert_eq!(mission.state(), MissionState::Active);
        assert_eq!(mission.current_index(), 0);
    }

    #[test]
    fn test_empty_mission_start() {
        let mut mission = Mission::new();
        mission.start();
        // Should stay idle with no waypoints
        assert_eq!(mission.state(), MissionState::Idle);
    }

    #[test]
    fn test_mission_pause_resume() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.start();

        mission.pause();
        assert_eq!(mission.state(), MissionState::Paused);

        mission.resume();
        assert_eq!(mission.state(), MissionState::Active);
    }

    #[test]
    fn test_pause_when_not_active() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        // Try to pause when idle
        mission.pause();
        assert_eq!(mission.state(), MissionState::Idle); // Should stay idle
    }

    #[test]
    fn test_resume_when_not_paused() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.start();

        // Try to resume when active
        mission.resume();
        assert_eq!(mission.state(), MissionState::Active); // Should stay active
    }

    #[test]
    fn test_mission_abort() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.start();

        mission.abort();
        assert_eq!(mission.state(), MissionState::Aborted);
    }

    #[test]
    fn test_mission_reset() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.start();
        mission.advance();

        mission.reset();
        assert_eq!(mission.state(), MissionState::Idle);
        assert_eq!(mission.current_index(), 0);
    }

    #[test]
    fn test_mission_advance() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([30.0, 0.0, -10.0]));
        mission.start();

        assert_eq!(mission.current_index(), 0);
        assert!(mission.advance());
        assert_eq!(mission.current_index(), 1);
        assert!(mission.advance());
        assert_eq!(mission.current_index(), 2);
        assert!(!mission.advance()); // No more waypoints
        assert_eq!(mission.state(), MissionState::Completed);
    }

    #[test]
    fn test_advance_when_not_active() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        // Don't start

        assert!(!mission.advance());
        assert_eq!(mission.current_index(), 0);
    }

    #[test]
    fn test_mission_completion() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.start();

        mission.advance();
        assert_eq!(mission.state(), MissionState::Completed);
    }
}

// ============================================================================
// Mission Repeat Tests
// ============================================================================

mod repeat_tests {
    use super::*;

    #[test]
    fn test_mission_repeat_infinite() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.set_repeat(true, 0); // Infinite repeats
        mission.start();

        // First pass
        assert!(mission.advance()); // Restarts at waypoint 0
        assert_eq!(mission.current_index(), 0);
        assert_eq!(mission.state(), MissionState::Active);

        // Second pass
        assert!(mission.advance());
        assert_eq!(mission.current_index(), 0);
    }

    #[test]
    fn test_mission_repeat_limited() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.set_repeat(true, 2); // 2 repeats
        mission.start();

        // First pass
        assert!(mission.advance());
        // Second pass
        assert!(mission.advance());
        // Third attempt should complete
        assert!(!mission.advance());
        assert_eq!(mission.state(), MissionState::Completed);
    }

    #[test]
    fn test_mission_no_repeat() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.set_repeat(false, 0);
        mission.start();

        assert!(!mission.advance());
        assert_eq!(mission.state(), MissionState::Completed);
    }
}

// ============================================================================
// Waypoint Access Tests
// ============================================================================

mod waypoint_access_tests {
    use super::*;

    #[test]
    fn test_get_waypoint() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));

        let wp = mission.get_waypoint(1).unwrap();
        assert_eq!(wp.position[0], 20.0);

        assert!(mission.get_waypoint(5).is_none());
    }

    #[test]
    fn test_get_waypoint_mut() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        let wp = mission.get_waypoint_mut(0).unwrap();
        wp.position[0] = 100.0;

        assert_eq!(mission.get_waypoint(0).unwrap().position[0], 100.0);
    }

    #[test]
    fn test_current_waypoint() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.start();

        assert_eq!(mission.current_waypoint().unwrap().position[0], 10.0);

        mission.advance();
        assert_eq!(mission.current_waypoint().unwrap().position[0], 20.0);
    }

    #[test]
    fn test_insert_waypoint() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([30.0, 0.0, -10.0]));

        // Insert in middle
        assert!(mission.insert_waypoint(1, Waypoint::goto([20.0, 0.0, -10.0])));

        assert_eq!(mission.waypoint_count(), 3);
        assert_eq!(mission.get_waypoint(0).unwrap().position[0], 10.0);
        assert_eq!(mission.get_waypoint(1).unwrap().position[0], 20.0);
        assert_eq!(mission.get_waypoint(2).unwrap().position[0], 30.0);
    }

    #[test]
    fn test_insert_at_beginning() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));

        assert!(mission.insert_waypoint(0, Waypoint::goto([10.0, 0.0, -10.0])));

        assert_eq!(mission.get_waypoint(0).unwrap().position[0], 10.0);
        assert_eq!(mission.get_waypoint(1).unwrap().position[0], 20.0);
    }

    #[test]
    fn test_insert_at_end() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        assert!(mission.insert_waypoint(1, Waypoint::goto([20.0, 0.0, -10.0])));

        assert_eq!(mission.get_waypoint(1).unwrap().position[0], 20.0);
    }

    #[test]
    fn test_insert_invalid_index() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        // Index 5 is beyond current size
        assert!(!mission.insert_waypoint(5, Waypoint::goto([20.0, 0.0, -10.0])));
    }

    #[test]
    fn test_remove_waypoint() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([30.0, 0.0, -10.0]));

        let removed = mission.remove_waypoint(1).unwrap();
        assert_eq!(removed.position[0], 20.0);
        assert_eq!(mission.waypoint_count(), 2);
    }

    #[test]
    fn test_remove_invalid_index() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));

        assert!(mission.remove_waypoint(5).is_none());
    }

    #[test]
    fn test_jump_to() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([30.0, 0.0, -10.0]));
        mission.start();

        assert!(mission.jump_to(2));
        assert_eq!(mission.current_index(), 2);

        assert!(!mission.jump_to(10)); // Invalid index
        assert_eq!(mission.current_index(), 2); // Unchanged
    }

    #[test]
    fn test_clear_mission() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));
        mission.start();
        mission.advance();

        mission.clear();

        assert_eq!(mission.waypoint_count(), 0);
        assert_eq!(mission.current_index(), 0);
        assert_eq!(mission.state(), MissionState::Idle);
    }
}

// ============================================================================
// Distance and Time Calculation Tests
// ============================================================================

mod calculation_tests {
    use super::*;

    #[test]
    fn test_total_distance() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([10.0, 10.0, -10.0]));

        let dist = mission.total_distance();
        assert!((dist - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_total_distance_single_waypoint() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));

        let dist = mission.total_distance();
        assert_eq!(dist, 0.0);
    }

    #[test]
    fn test_total_distance_empty_mission() {
        let mission = Mission::new();
        assert_eq!(mission.total_distance(), 0.0);
    }

    #[test]
    fn test_distance_to_current() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.start();

        let dist = mission.distance_to_current([0.0, 0.0, -10.0]);
        assert!((dist - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_distance_to_current_no_waypoint() {
        let mission = Mission::new();
        let dist = mission.distance_to_current([0.0, 0.0, 0.0]);
        assert_eq!(dist, 0.0);
    }

    #[test]
    fn test_waypoint_reached() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]).with_acceptance(2.0));
        mission.start();

        assert!(!mission.check_waypoint_reached([0.0, 0.0, -10.0])); // Far
        assert!(mission.check_waypoint_reached([10.0, 0.5, -10.0])); // Close
        assert!(mission.check_waypoint_reached([10.0, 0.0, -10.0])); // Exact
    }

    #[test]
    fn test_estimated_time() {
        let mut mission = Mission::new();
        mission.set_cruise_speed(10.0);
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([100.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::loiter([100.0, 0.0, -10.0], 30.0));

        let time = mission.estimated_time(0.0); // Use cruise speed
        // 100m at 10m/s = 10s + 30s loiter = 40s
        assert!((time - 40.0).abs() < 0.1);
    }

    #[test]
    fn test_estimated_time_custom_speed() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([50.0, 0.0, -10.0]));

        let time = mission.estimated_time(5.0); // 5 m/s
        // 50m at 5m/s = 10s
        assert!((time - 10.0).abs() < 0.1);
    }
}

// ============================================================================
// Home Position Tests
// ============================================================================

mod home_tests {
    use super::*;

    #[test]
    fn test_set_home() {
        let mut mission = Mission::new();
        mission.set_home([100.0, 200.0, -5.0]);
        assert_eq!(mission.home(), [100.0, 200.0, -5.0]);
    }

    #[test]
    fn test_default_home() {
        let mission = Mission::new();
        assert_eq!(mission.home(), [0.0, 0.0, 0.0]);
    }
}

// ============================================================================
// Survey Pattern Tests
// ============================================================================

mod survey_tests {
    use super::*;

    fn create_test_area() -> SurveyArea {
        SurveyArea::default() // 100x100m square
    }

    #[test]
    fn test_lawnmower_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);

        assert!(!points.is_empty());

        // All points should be at survey altitude
        for p in &points {
            assert!((p[2] + area.altitude).abs() < 0.01);
        }
    }

    #[test]
    fn test_expanding_square_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::ExpandingSquare);

        assert!(!points.is_empty());

        // First point should be at center
        let cx = 50.0;
        let cy = 50.0;
        assert!((points[0][0] - cx).abs() < 0.01);
        assert!((points[0][1] - cy).abs() < 0.01);
    }

    #[test]
    fn test_spiral_inward_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Spiral { inward: true });

        assert!(!points.is_empty());

        // Inward spiral should start from edge and end near center
        let last = points.last().unwrap();
        let cx = 50.0;
        let cy = 50.0;
        let dist_to_center = ((last[0] - cx).powi(2) + (last[1] - cy).powi(2)).sqrt();

        // Last point should be closer to center than first
        let first = &points[0];
        let first_dist = ((first[0] - cx).powi(2) + (first[1] - cy).powi(2)).sqrt();
        assert!(dist_to_center < first_dist);
    }

    #[test]
    fn test_spiral_outward_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Spiral { inward: false });

        assert!(!points.is_empty());

        // Outward spiral should start near center
        let first = &points[0];
        let cx = 50.0;
        let cy = 50.0;
        let first_dist = ((first[0] - cx).powi(2) + (first[1] - cy).powi(2)).sqrt();
        assert!(first_dist < 20.0); // Should start near center
    }

    #[test]
    fn test_sector_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Sector { angle_deg: 90.0 });

        assert!(!points.is_empty());

        // First point should be at center
        let cx = 50.0;
        let cy = 50.0;
        assert!((points[0][0] - cx).abs() < 0.01);
        assert!((points[0][1] - cy).abs() < 0.01);
    }

    #[test]
    fn test_parallel_track_pattern() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::ParallelTrack);

        // Should be same as lawnmower
        assert!(!points.is_empty());
    }

    #[test]
    fn test_survey_to_mission() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);
        let mission = survey_to_mission(&points, false);

        assert_eq!(mission.waypoint_count(), points.len());
    }

    #[test]
    fn test_survey_to_mission_with_photos() {
        let area = create_test_area();
        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);
        let mission = survey_to_mission(&points, true);

        // All waypoints should have TakePhoto action
        for i in 0..mission.waypoint_count() {
            assert_eq!(
                mission.get_waypoint(i).unwrap().action,
                WaypointAction::TakePhoto
            );
        }
    }

    #[test]
    fn test_survey_area_default() {
        let area = SurveyArea::default();
        assert_eq!(area.vertices.len(), 4);
        assert_eq!(area.altitude, 30.0);
        assert_eq!(area.spacing, 20.0);
    }

    #[test]
    fn test_survey_empty_area() {
        use heapless::Vec;
        let area = SurveyArea {
            vertices: Vec::new(),
            altitude: 30.0,
            spacing: 20.0,
            overshoot: 10.0,
            entry_angle: 0.0,
        };

        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);
        assert!(points.is_empty());
    }

    #[test]
    fn test_survey_with_rotation() {
        let mut area = SurveyArea::default();
        area.entry_angle = 45.0; // 45 degree rotation

        let points = generate_survey_mission(&area, SurveyPattern::Lawnmower);
        assert!(!points.is_empty());
    }
}

// ============================================================================
// Edge Case Tests
// ============================================================================

mod edge_case_tests {
    use super::*;

    #[test]
    fn test_default_mission() {
        let mission = Mission::default();
        assert_eq!(mission.state(), MissionState::Idle);
        assert_eq!(mission.waypoint_count(), 0);
    }

    #[test]
    fn test_waypoint_type_equality() {
        assert_eq!(WaypointType::Goto, WaypointType::Goto);
        assert_ne!(WaypointType::Goto, WaypointType::Land);
    }

    #[test]
    fn test_acceptance_mode_equality() {
        assert_eq!(AcceptanceMode::PassThrough, AcceptanceMode::PassThrough);
        assert_ne!(AcceptanceMode::PassThrough, AcceptanceMode::StopAt);
    }

    #[test]
    fn test_mission_state_equality() {
        assert_eq!(MissionState::Active, MissionState::Active);
        assert_ne!(MissionState::Active, MissionState::Paused);
    }

    #[test]
    fn test_3d_distance_calculation() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, 0.0]));
        mission.add_waypoint(Waypoint::goto([3.0, 4.0, 0.0]));

        // 3-4-5 triangle
        assert!((mission.total_distance() - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_3d_distance_with_altitude() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, 0.0]));
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));

        // Pure vertical
        assert!((mission.total_distance() - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_many_waypoints() {
        let mut mission = Mission::new();

        // Add many waypoints
        for i in 0..50 {
            mission.add_waypoint(Waypoint::goto([i as f32, 0.0, -10.0]));
        }

        mission.start();

        // Advance through all
        while mission.state() == MissionState::Active {
            mission.advance();
        }

        assert_eq!(mission.state(), MissionState::Completed);
    }
}
