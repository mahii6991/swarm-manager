//! Safety and Telemetry Panel
//!
//! Displays collision avoidance zones, mission waypoints,
//! telemetry health status, and failsafe indicators.

use crate::state::SimulationState;
use egui::{Color32, RichText, Ui};

pub fn show(ui: &mut Ui, state: &mut SimulationState) {
    ui.heading("Safety & Telemetry");
    ui.add_space(5.0);

    // Collision Avoidance Section
    egui::CollapsingHeader::new("🛡 Collision Avoidance")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(
                &mut state.safety_state.collision_avoidance_enabled,
                "Enable Collision Avoidance",
            );

            ui.add_space(3.0);
            ui.add(
                egui::Slider::new(&mut state.safety_state.min_separation, 1.0..=20.0)
                    .text("Min Separation (m)"),
            );
            ui.add(
                egui::Slider::new(&mut state.safety_state.avoidance_radius, 5.0..=50.0)
                    .text("Avoidance Radius (m)"),
            );

            ui.add_space(5.0);

            // Show collision warnings
            let warnings = state.get_collision_warnings();
            if warnings.is_empty() {
                ui.label(RichText::new("✓ No collision warnings").color(Color32::GREEN));
            } else {
                ui.label(
                    RichText::new(format!("⚠ {} collision warnings", warnings.len()))
                        .color(Color32::YELLOW),
                );
                for (d1, d2, dist) in warnings.iter().take(3) {
                    ui.label(format!("  D{} ↔ D{}: {:.1}m", d1, d2, dist));
                }
            }

            ui.add_space(3.0);
            ui.checkbox(
                &mut state.safety_state.show_avoidance_zones,
                "Show Avoidance Zones",
            );
        });

    ui.add_space(10.0);

    // Mission Planning Section
    egui::CollapsingHeader::new("📍 Mission")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut state.mission_state.show_waypoints, "Show Waypoints");
            ui.checkbox(&mut state.mission_state.show_path, "Show Mission Path");

            ui.add_space(5.0);

            let mission_status = if state.mission_state.is_active {
                RichText::new("● Active").color(Color32::GREEN)
            } else {
                RichText::new("○ Idle").color(Color32::GRAY)
            };
            ui.horizontal(|ui| {
                ui.label("Status:");
                ui.label(mission_status);
            });

            if !state.mission_state.waypoints.is_empty() {
                ui.label(format!(
                    "Waypoints: {}/{}",
                    state.mission_state.current_waypoint, state.mission_state.waypoints.len()
                ));

                // Progress bar
                let progress = state.mission_state.current_waypoint as f32
                    / state.mission_state.waypoints.len() as f32;
                ui.add(egui::ProgressBar::new(progress).text("Progress"));
            }

            ui.add_space(5.0);

            ui.horizontal(|ui| {
                if ui.button("Add Waypoint").clicked() {
                    state.add_random_waypoint();
                }
                if ui.button("Clear").clicked() {
                    state.mission_state.waypoints.clear();
                    state.mission_state.current_waypoint = 0;
                }
            });

            ui.horizontal(|ui| {
                if state.mission_state.is_active {
                    if ui.button("⏸ Pause").clicked() {
                        state.mission_state.is_active = false;
                    }
                } else {
                    if ui.button("▶ Start").clicked() && !state.mission_state.waypoints.is_empty() {
                        state.mission_state.is_active = true;
                    }
                }
                if ui.button("⏹ Reset").clicked() {
                    state.mission_state.current_waypoint = 0;
                    state.mission_state.is_active = false;
                }
            });

            // Survey pattern buttons
            ui.add_space(5.0);
            ui.label("Survey Patterns:");
            ui.horizontal(|ui| {
                if ui.small_button("Lawnmower").clicked() {
                    state.generate_survey_pattern("lawnmower");
                }
                if ui.small_button("Spiral").clicked() {
                    state.generate_survey_pattern("spiral");
                }
                if ui.small_button("Square").clicked() {
                    state.generate_survey_pattern("square");
                }
            });
        });

    ui.add_space(10.0);

    // Telemetry Health Section
    egui::CollapsingHeader::new("📊 Telemetry")
        .default_open(true)
        .show(ui, |ui| {
            let stats = state.get_swarm_telemetry();

            // Overall health indicator
            let health_color = match stats.health_status.as_str() {
                "Healthy" => Color32::GREEN,
                "Warning" => Color32::YELLOW,
                "Critical" => Color32::RED,
                _ => Color32::GRAY,
            };
            ui.horizontal(|ui| {
                ui.label("Swarm Health:");
                ui.label(RichText::new(&stats.health_status).color(health_color));
            });

            ui.add_space(3.0);

            // Battery status
            ui.horizontal(|ui| {
                ui.label("🔋 Battery:");
                let battery_color = if stats.avg_battery > 50.0 {
                    Color32::GREEN
                } else if stats.avg_battery > 20.0 {
                    Color32::YELLOW
                } else {
                    Color32::RED
                };
                ui.label(
                    RichText::new(format!("Avg: {:.0}%", stats.avg_battery)).color(battery_color),
                );
                ui.label(format!("Min: {}%", stats.min_battery));
            });

            // Speed
            ui.horizontal(|ui| {
                ui.label("🚀 Speed:");
                ui.label(format!(
                    "Avg: {:.1} m/s | Max: {:.1} m/s",
                    stats.avg_speed, stats.max_speed
                ));
            });

            // Communication
            ui.horizontal(|ui| {
                ui.label("📡 Comm:");
                let comm_color = if stats.connected_drones == stats.total_drones {
                    Color32::GREEN
                } else {
                    Color32::YELLOW
                };
                ui.label(
                    RichText::new(format!(
                        "{}/{} connected",
                        stats.connected_drones, stats.total_drones
                    ))
                    .color(comm_color),
                );
            });

            ui.add_space(3.0);
            ui.checkbox(&mut state.telemetry_state.show_health_overlay, "Show Health Overlay");
        });

    ui.add_space(10.0);

    // Failsafe Section
    egui::CollapsingHeader::new("⚠ Failsafe")
        .default_open(true)
        .show(ui, |ui| {
            // Active failsafe status
            if state.failsafe_state.active_failsafe.is_some() {
                let failsafe = state.failsafe_state.active_failsafe.as_ref().unwrap();
                ui.horizontal(|ui| {
                    ui.label(RichText::new("⚠ FAILSAFE ACTIVE").color(Color32::RED));
                });
                ui.label(RichText::new(failsafe).color(Color32::YELLOW));

                if ui.button("Clear Failsafe").clicked() {
                    state.failsafe_state.active_failsafe = None;
                }
            } else {
                ui.label(RichText::new("✓ No active failsafe").color(Color32::GREEN));
            }

            ui.add_space(5.0);

            // Failsafe triggers (for testing)
            ui.label("Test Failsafe:");
            ui.horizontal(|ui| {
                if ui.small_button("Low Battery").clicked() {
                    state.trigger_failsafe("Low Battery - RTL");
                }
                if ui.small_button("GPS Lost").clicked() {
                    state.trigger_failsafe("GPS Lost - Hold");
                }
                if ui.small_button("Geofence").clicked() {
                    state.trigger_failsafe("Geofence Breach - RTL");
                }
            });

            ui.add_space(5.0);

            // Geofence settings
            ui.checkbox(&mut state.failsafe_state.geofence_enabled, "Enable Geofence");
            if state.failsafe_state.geofence_enabled {
                ui.add(
                    egui::Slider::new(&mut state.failsafe_state.geofence_radius, 50.0..=500.0)
                        .text("Geofence Radius (m)"),
                );
                ui.checkbox(
                    &mut state.failsafe_state.show_geofence,
                    "Show Geofence Boundary",
                );
            }
        });

    ui.add_space(10.0);

    // Alerts Section
    if !state.alerts.is_empty() {
        egui::CollapsingHeader::new(format!("🔔 Alerts ({})", state.alerts.len()))
            .default_open(true)
            .show(ui, |ui| {
                for alert in state.alerts.iter().take(5) {
                    let color = match alert.severity.as_str() {
                        "Critical" => Color32::RED,
                        "Warning" => Color32::YELLOW,
                        _ => Color32::LIGHT_GRAY,
                    };
                    ui.label(RichText::new(&alert.message).color(color));
                }
                if state.alerts.len() > 5 {
                    ui.label(format!("... and {} more", state.alerts.len() - 5));
                }
                if ui.button("Clear All").clicked() {
                    state.alerts.clear();
                }
            });
    }
}
