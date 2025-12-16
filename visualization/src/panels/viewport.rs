//! Main 2D viewport showing drone positions

use crate::renderers::drone_renderer;
use crate::state::SimulationState;
use crate::themes;
use egui::{Color32, Pos2, Rect, Sense, Stroke, Ui, Vec2};

pub fn show(ui: &mut Ui, state: &mut SimulationState) {
    let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
    let rect = response.rect;

    // Handle pan (only when not clicking on a drone)
    if response.dragged() {
        let delta = response.drag_delta();
        state.viewport.center -= Vec2::new(
            delta.x / state.viewport.zoom,
            -delta.y / state.viewport.zoom,
        );
    }

    // Handle zoom
    let scroll = ui.input(|i| i.raw_scroll_delta.y);
    if scroll != 0.0 {
        let zoom_factor = if scroll > 0.0 { 1.1 } else { 0.9 };
        state.viewport.zoom = (state.viewport.zoom * zoom_factor).clamp(0.5, 10.0);
    }

    // Handle click to select drone
    if response.clicked() {
        if let Some(click_pos) = response.interact_pointer_pos() {
            // Transform function: screen to world
            let screen_to_world = |screen_pos: Pos2| -> Pos2 {
                let offset = screen_pos - rect.center();
                Pos2::new(
                    state.viewport.center.x + offset.x / state.viewport.zoom,
                    state.viewport.center.y - offset.y / state.viewport.zoom,
                )
            };

            let world_click = screen_to_world(click_pos);

            // Find closest drone within click radius
            let click_radius = 15.0 / state.viewport.zoom; // Adjust for zoom
            let mut closest_drone: Option<u64> = None;
            let mut closest_dist = f32::MAX;

            for drone in &state.drones {
                let dist = (drone.position - world_click).length();
                if dist < click_radius && dist < closest_dist {
                    closest_dist = dist;
                    closest_drone = Some(drone.id);
                }
            }

            state.selected_drone = closest_drone;
        }
    }

    // Draw background
    painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 22, 25));

    // Draw grid
    if state.viewport.show_grid {
        draw_grid(&painter, rect, state);
    }

    // Transform function: world to screen
    let world_to_screen = |world_pos: Pos2| -> Pos2 {
        let offset = world_pos - state.viewport.center;
        Pos2::new(
            rect.center().x + offset.x * state.viewport.zoom,
            rect.center().y - offset.y * state.viewport.zoom, // Y-flip
        )
    };

    // Draw geofence boundary
    if state.failsafe_state.geofence_enabled && state.failsafe_state.show_geofence {
        let center_screen = world_to_screen(Pos2::ZERO);
        let radius = state.failsafe_state.geofence_radius * state.viewport.zoom;
        painter.circle_stroke(
            center_screen,
            radius,
            Stroke::new(2.0, Color32::from_rgba_premultiplied(255, 100, 100, 150)),
        );
        // Draw warning zone (inner ring at 80%)
        painter.circle_stroke(
            center_screen,
            radius * 0.8,
            Stroke::new(1.0, Color32::from_rgba_premultiplied(255, 200, 100, 100)),
        );
    }

    // Draw mission waypoints and path
    if state.mission_state.show_waypoints && !state.mission_state.waypoints.is_empty() {
        let waypoints = &state.mission_state.waypoints;

        // Draw path lines
        if state.mission_state.show_path && waypoints.len() > 1 {
            for i in 1..waypoints.len() {
                let from = world_to_screen(waypoints[i - 1]);
                let to = world_to_screen(waypoints[i]);
                let color = if i <= state.mission_state.current_waypoint {
                    Color32::from_rgba_premultiplied(100, 255, 100, 150) // Completed
                } else {
                    Color32::from_rgba_premultiplied(100, 150, 255, 150) // Pending
                };
                painter.line_segment([from, to], Stroke::new(2.0, color));
            }
        }

        // Draw waypoint markers
        for (i, wp) in waypoints.iter().enumerate() {
            let screen_pos = world_to_screen(*wp);
            let (color, size) = if i < state.mission_state.current_waypoint {
                (Color32::from_rgb(100, 200, 100), 6.0) // Completed
            } else if i == state.mission_state.current_waypoint {
                (Color32::from_rgb(255, 200, 50), 10.0) // Current
            } else {
                (Color32::from_rgb(100, 150, 255), 6.0) // Pending
            };

            painter.circle_filled(screen_pos, size, color);
            painter.text(
                screen_pos + Vec2::new(10.0, -10.0),
                egui::Align2::LEFT_BOTTOM,
                format!("{}", i + 1),
                egui::FontId::monospace(10.0),
                color,
            );
        }
    }

    // Draw collision avoidance zones
    if state.safety_state.collision_avoidance_enabled && state.safety_state.show_avoidance_zones {
        for drone in &state.drones {
            let screen_pos = world_to_screen(drone.position);
            let radius = state.safety_state.avoidance_radius * state.viewport.zoom;

            // Draw avoidance zone
            painter.circle_stroke(
                screen_pos,
                radius,
                Stroke::new(1.0, Color32::from_rgba_premultiplied(255, 150, 50, 80)),
            );
        }

        // Highlight collision warnings
        let warnings = state.get_collision_warnings();
        for (d1_id, d2_id, _dist) in warnings {
            if let Some(d1) = state.drones.iter().find(|d| d.id == d1_id) {
                if let Some(d2) = state.drones.iter().find(|d| d.id == d2_id) {
                    let p1 = world_to_screen(d1.position);
                    let p2 = world_to_screen(d2.position);
                    painter.line_segment(
                        [p1, p2],
                        Stroke::new(2.0, Color32::from_rgb(255, 50, 50)),
                    );
                }
            }
        }
    }

    // Draw drone trails
    if state.viewport.show_trails {
        for drone in &state.drones {
            if drone.trail.len() > 1 {
                let points: Vec<Pos2> = drone.trail.iter().map(|p| world_to_screen(*p)).collect();
                for i in 1..points.len() {
                    let alpha = (i as f32 / points.len() as f32 * 80.0) as u8;
                    let color = Color32::from_rgba_premultiplied(100, 150, 255, alpha);
                    painter.line_segment([points[i - 1], points[i]], Stroke::new(1.5, color));
                }
            }
        }
    }

    // Draw drones
    let selected_id = state.selected_drone;
    for drone in &state.drones {
        let screen_pos = world_to_screen(drone.position);

        // Skip if outside viewport
        if !rect.contains(screen_pos) {
            continue;
        }

        // Draw target position indicator
        let target_screen = world_to_screen(drone.target_position);
        painter.circle_stroke(target_screen, 5.0, Stroke::new(1.0, themes::drone::TARGET));

        // Draw velocity vector
        if state.viewport.show_velocities && drone.velocity.length() > 0.1 {
            let vel_end = screen_pos + drone.velocity * state.viewport.zoom * 3.0;
            painter.arrow(
                screen_pos,
                vel_end - screen_pos,
                Stroke::new(1.5, themes::drone::VELOCITY),
            );
        }

        // Highlight selected drone
        let is_selected = Some(drone.id) == selected_id;
        if is_selected {
            // Draw selection ring
            painter.circle_stroke(
                screen_pos,
                18.0,
                Stroke::new(2.0, Color32::from_rgb(0, 255, 255)),
            );
            painter.circle_stroke(
                screen_pos,
                22.0,
                Stroke::new(1.0, Color32::from_rgb(0, 200, 200)),
            );

            // Draw drone info tooltip
            let info_pos = screen_pos + Vec2::new(25.0, -20.0);
            let info_rect = egui::Rect::from_min_size(info_pos, Vec2::new(100.0, 60.0));
            painter.rect_filled(
                info_rect,
                4.0,
                Color32::from_rgba_premultiplied(0, 0, 0, 220),
            );
            painter.rect_stroke(
                info_rect,
                4.0,
                Stroke::new(1.0, Color32::from_rgb(0, 200, 200)),
            );

            painter.text(
                info_pos + Vec2::new(5.0, 5.0),
                egui::Align2::LEFT_TOP,
                format!("Drone #{}", drone.id),
                egui::FontId::monospace(10.0),
                Color32::from_rgb(0, 255, 255),
            );
            painter.text(
                info_pos + Vec2::new(5.0, 18.0),
                egui::Align2::LEFT_TOP,
                format!("Battery: {}%", drone.battery),
                egui::FontId::monospace(9.0),
                themes::ui::TEXT,
            );
            painter.text(
                info_pos + Vec2::new(5.0, 30.0),
                egui::Align2::LEFT_TOP,
                format!("Status: {:?}", drone.status),
                egui::FontId::monospace(9.0),
                themes::ui::TEXT,
            );
            painter.text(
                info_pos + Vec2::new(5.0, 42.0),
                egui::Align2::LEFT_TOP,
                format!("Alt: {:.1}m", drone.altitude),
                egui::FontId::monospace(9.0),
                themes::ui::TEXT_DIM,
            );
        }

        // Draw drone
        drone_renderer::draw_drone(&painter, screen_pos, drone, state.viewport.zoom);
    }

    // Draw info overlay
    let info_rect = Rect::from_min_size(rect.min + Vec2::new(10.0, 10.0), Vec2::new(220.0, 95.0));
    painter.rect_filled(
        info_rect,
        4.0,
        Color32::from_rgba_premultiplied(0, 0, 0, 180),
    );

    let text_pos = info_rect.min + Vec2::new(10.0, 10.0);
    painter.text(
        text_pos,
        egui::Align2::LEFT_TOP,
        format!("Drones: {}", state.drones.len()),
        egui::FontId::monospace(12.0),
        themes::ui::TEXT,
    );
    painter.text(
        text_pos + Vec2::new(0.0, 16.0),
        egui::Align2::LEFT_TOP,
        format!("Formation: {:?}", state.formation),
        egui::FontId::monospace(12.0),
        themes::ui::TEXT,
    );
    painter.text(
        text_pos + Vec2::new(0.0, 32.0),
        egui::Align2::LEFT_TOP,
        format!("Zoom: {:.1}x", state.viewport.zoom),
        egui::FontId::monospace(12.0),
        themes::ui::TEXT_DIM,
    );
    painter.text(
        text_pos + Vec2::new(0.0, 48.0),
        egui::Align2::LEFT_TOP,
        "Drag=pan | Scroll=zoom | Click=select",
        egui::FontId::monospace(10.0),
        themes::ui::TEXT_DIM,
    );
    painter.text(
        text_pos + Vec2::new(0.0, 60.0),
        egui::Align2::LEFT_TOP,
        "Space=play | R=reset | D=demo",
        egui::FontId::monospace(10.0),
        themes::ui::TEXT_DIM,
    );

    // Legend
    let legend_rect = Rect::from_min_size(
        Pos2::new(rect.max.x - 120.0, rect.min.y + 10.0),
        Vec2::new(110.0, 90.0),
    );
    painter.rect_filled(
        legend_rect,
        4.0,
        Color32::from_rgba_premultiplied(0, 0, 0, 180),
    );

    let legend_items = [
        ("Active", themes::drone::ACTIVE),
        ("Returning", themes::drone::RETURNING),
        ("Emergency", themes::drone::EMERGENCY),
        ("Failed", themes::drone::FAILED),
    ];

    for (i, (label, color)) in legend_items.iter().enumerate() {
        let y = legend_rect.min.y + 12.0 + i as f32 * 18.0;
        painter.circle_filled(Pos2::new(legend_rect.min.x + 15.0, y), 5.0, *color);
        painter.text(
            Pos2::new(legend_rect.min.x + 28.0, y),
            egui::Align2::LEFT_CENTER,
            *label,
            egui::FontId::monospace(10.0),
            themes::ui::TEXT,
        );
    }
}

fn draw_grid(painter: &egui::Painter, rect: Rect, state: &SimulationState) {
    let grid_spacing = 20.0 * state.viewport.zoom;
    let major_spacing = 100.0 * state.viewport.zoom;

    let center = rect.center();
    let offset_x = (state.viewport.center.x * state.viewport.zoom) % grid_spacing;
    let offset_y = (state.viewport.center.y * state.viewport.zoom) % grid_spacing;

    // Minor grid lines
    let mut x = rect.min.x - offset_x;
    while x < rect.max.x {
        painter.line_segment(
            [Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)],
            Stroke::new(0.5, themes::ui::GRID),
        );
        x += grid_spacing;
    }

    let mut y = rect.min.y + offset_y;
    while y < rect.max.y {
        painter.line_segment(
            [Pos2::new(rect.min.x, y), Pos2::new(rect.max.x, y)],
            Stroke::new(0.5, themes::ui::GRID),
        );
        y += grid_spacing;
    }

    // Major grid lines
    let major_offset_x = (state.viewport.center.x * state.viewport.zoom) % major_spacing;
    let major_offset_y = (state.viewport.center.y * state.viewport.zoom) % major_spacing;

    let mut x = rect.min.x - major_offset_x;
    while x < rect.max.x {
        painter.line_segment(
            [Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)],
            Stroke::new(1.0, themes::ui::GRID_MAJOR),
        );
        x += major_spacing;
    }

    let mut y = rect.min.y + major_offset_y;
    while y < rect.max.y {
        painter.line_segment(
            [Pos2::new(rect.min.x, y), Pos2::new(rect.max.x, y)],
            Stroke::new(1.0, themes::ui::GRID_MAJOR),
        );
        y += major_spacing;
    }

    // Origin axes
    let origin = Pos2::new(
        center.x - state.viewport.center.x * state.viewport.zoom,
        center.y + state.viewport.center.y * state.viewport.zoom,
    );

    if rect.contains(Pos2::new(origin.x, rect.center().y)) {
        painter.line_segment(
            [
                Pos2::new(origin.x, rect.min.y),
                Pos2::new(origin.x, rect.max.y),
            ],
            Stroke::new(1.5, themes::ui::AXIS_Y),
        );
    }
    if rect.contains(Pos2::new(rect.center().x, origin.y)) {
        painter.line_segment(
            [
                Pos2::new(rect.min.x, origin.y),
                Pos2::new(rect.max.x, origin.y),
            ],
            Stroke::new(1.5, themes::ui::AXIS_X),
        );
    }
}
