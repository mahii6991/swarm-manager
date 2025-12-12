//! Drone rendering utilities

use crate::state::{DroneStatus, DroneVisual};
use crate::themes;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw a single drone at the given screen position
pub fn draw_drone(painter: &Painter, screen_pos: Pos2, drone: &DroneVisual, _zoom: f32) {
    let size = 8.0 + (drone.altitude / 10.0).clamp(0.0, 5.0);

    // Get color based on status
    let color = match drone.status {
        DroneStatus::Active => themes::drone::ACTIVE,
        DroneStatus::Idle => themes::drone::IDLE,
        DroneStatus::Returning => themes::drone::RETURNING,
        DroneStatus::Emergency => themes::drone::EMERGENCY,
        DroneStatus::Failed => themes::drone::FAILED,
    };

    // Draw outer glow for active drones
    if drone.status == DroneStatus::Active {
        painter.circle_filled(
            screen_pos,
            size + 4.0,
            Color32::from_rgba_premultiplied(color.r(), color.g(), color.b(), 30),
        );
    }

    // Draw drone body (triangle pointing in velocity direction)
    let angle = if drone.velocity.length() > 0.1 {
        drone.velocity.angle()
    } else {
        -std::f32::consts::FRAC_PI_2 // Point up by default
    };

    let points = drone_triangle(screen_pos, size, angle);
    painter.add(egui::Shape::convex_polygon(
        points.to_vec(),
        color,
        Stroke::new(1.0, color.linear_multiply(1.2)),
    ));

    // Draw center dot
    painter.circle_filled(screen_pos, 2.0, Color32::WHITE);

    // Draw battery indicator bar
    let bar_width = size * 1.5;
    let bar_height = 3.0;
    let bar_pos = screen_pos + Vec2::new(-bar_width / 2.0, size + 5.0);

    // Background
    painter.rect_filled(
        egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width, bar_height)),
        1.0,
        Color32::from_rgb(40, 40, 40),
    );

    // Fill based on battery level
    let fill_width = bar_width * (drone.battery as f32 / 100.0);
    let fill_color = if drone.battery > 50 {
        themes::drone::ACTIVE
    } else if drone.battery > 20 {
        themes::drone::RETURNING
    } else {
        themes::drone::EMERGENCY
    };

    painter.rect_filled(
        egui::Rect::from_min_size(bar_pos, Vec2::new(fill_width, bar_height)),
        1.0,
        fill_color,
    );

    // Draw ID label
    painter.text(
        screen_pos + Vec2::new(0.0, -size - 8.0),
        egui::Align2::CENTER_BOTTOM,
        format!("D{}", drone.id),
        egui::FontId::monospace(9.0),
        Color32::from_rgba_premultiplied(200, 200, 200, 180),
    );
}

/// Generate triangle points for drone shape
fn drone_triangle(center: Pos2, size: f32, angle: f32) -> [Pos2; 3] {
    let front = Vec2::new(angle.cos(), angle.sin()) * size;
    let back_angle1 = angle + std::f32::consts::FRAC_PI_2 + std::f32::consts::FRAC_PI_4;
    let back_angle2 = angle - std::f32::consts::FRAC_PI_2 - std::f32::consts::FRAC_PI_4;

    let back1 = Vec2::new(back_angle1.cos(), back_angle1.sin()) * size * 0.6;
    let back2 = Vec2::new(back_angle2.cos(), back_angle2.sin()) * size * 0.6;

    [center + front, center + back1, center + back2]
}
