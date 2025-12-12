//! ACO (Ant Colony Optimization) rendering

use crate::state::ACOVisualState;
use crate::themes;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw ACO visualization
pub fn draw<F>(painter: &Painter, aco: &ACOVisualState, world_to_screen: F, _scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    // Draw obstacles
    for obstacle in &aco.obstacles {
        let center = world_to_screen(obstacle.center);
        // Note: we need to scale the radius too, but since we don't have direct access
        // to scale here, we'll use a fixed visual size
        painter.circle_filled(center, obstacle.radius * 1.5, themes::aco::OBSTACLE);
        painter.circle_stroke(
            center,
            obstacle.radius * 1.5,
            Stroke::new(2.0, Color32::from_rgb(150, 70, 70)),
        );
    }

    // Draw pheromone trails
    for trail in &aco.pheromones {
        let from = world_to_screen(trail.from);
        let to = world_to_screen(trail.to);

        // Color based on strength
        let color = interpolate_color(
            themes::aco::PHEROMONE_LOW,
            themes::aco::PHEROMONE_HIGH,
            trail.strength.clamp(0.0, 1.0),
        );

        painter.line_segment([from, to], Stroke::new(1.0 + trail.strength * 2.0, color));
    }

    // Draw best path
    if aco.best_path.len() > 1 {
        let points: Vec<Pos2> = aco.best_path.iter().map(|p| world_to_screen(*p)).collect();

        for i in 1..points.len() {
            painter.line_segment(
                [points[i - 1], points[i]],
                Stroke::new(3.0, themes::aco::BEST_PATH),
            );
        }

        // Draw waypoint markers
        for (i, point) in points.iter().enumerate() {
            let is_endpoint = i == 0 || i == points.len() - 1;
            let size = if is_endpoint { 8.0 } else { 4.0 };
            let color = if i == 0 {
                themes::aco::START
            } else if i == points.len() - 1 {
                themes::aco::GOAL
            } else {
                themes::aco::BEST_PATH
            };
            painter.circle_filled(*point, size, color);
        }
    }

    // Draw start and goal markers
    let start = world_to_screen(aco.start);
    let goal = world_to_screen(aco.goal);

    // Start marker (blue square)
    let start_size = 10.0;
    painter.rect_filled(
        egui::Rect::from_center_size(start, Vec2::splat(start_size * 2.0)),
        2.0,
        themes::aco::START,
    );
    painter.text(
        start + Vec2::new(0.0, -15.0),
        egui::Align2::CENTER_BOTTOM,
        "START",
        egui::FontId::monospace(9.0),
        themes::aco::START,
    );

    // Goal marker (red circle with X)
    painter.circle_filled(goal, 12.0, themes::aco::GOAL);
    painter.circle_stroke(goal, 12.0, Stroke::new(2.0, Color32::WHITE));
    painter.text(
        goal,
        egui::Align2::CENTER_CENTER,
        "X",
        egui::FontId::monospace(12.0),
        Color32::WHITE,
    );
    painter.text(
        goal + Vec2::new(0.0, -18.0),
        egui::Align2::CENTER_BOTTOM,
        "GOAL",
        egui::FontId::monospace(9.0),
        themes::aco::GOAL,
    );

    // Draw ants
    for ant in &aco.ants {
        let pos = world_to_screen(ant.position);
        painter.circle_filled(pos, 4.0, themes::aco::ANT);
        painter.circle_stroke(pos, 4.0, Stroke::new(1.0, Color32::from_rgb(255, 150, 100)));
    }
}

/// Interpolate between two colors
fn interpolate_color(from: Color32, to: Color32, t: f32) -> Color32 {
    Color32::from_rgba_premultiplied(
        (from.r() as f32 + (to.r() as f32 - from.r() as f32) * t) as u8,
        (from.g() as f32 + (to.g() as f32 - from.g() as f32) * t) as u8,
        (from.b() as f32 + (to.b() as f32 - from.b() as f32) * t) as u8,
        (from.a() as f32 + (to.a() as f32 - from.a() as f32) * t) as u8,
    )
}
