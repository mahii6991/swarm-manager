//! Federated Learning visualization rendering

use crate::state::FederatedVisualState;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw Federated Learning visualization
pub fn draw<F>(painter: &Painter, state: &FederatedVisualState, world_to_screen: F, _scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    let aggregator_pos = state
        .aggregator
        .and_then(|id| state.nodes.get(id).map(|n| world_to_screen(n.position)));

    // Draw connections to aggregator
    if let Some(agg_pos) = aggregator_pos {
        for node in &state.nodes {
            if Some(node.id) != state.aggregator {
                let pos = world_to_screen(node.position);

                // Draw connection line
                let color = if node.is_selected {
                    Color32::from_rgba_premultiplied(100, 200, 255, 150)
                } else {
                    Color32::from_rgba_premultiplied(100, 100, 100, 50)
                };
                painter.line_segment([pos, agg_pos], Stroke::new(1.5, color));

                // Draw data flow animation if selected
                if node.is_selected {
                    let progress = (state.round as f32 * 0.1) % 1.0;
                    let flow_pos = Pos2::new(
                        pos.x + (agg_pos.x - pos.x) * progress,
                        pos.y + (agg_pos.y - pos.y) * progress,
                    );
                    painter.circle_filled(flow_pos, 4.0, Color32::from_rgb(100, 200, 255));
                }
            }
        }
    }

    // Draw nodes
    for node in &state.nodes {
        let pos = world_to_screen(node.position);
        let is_aggregator = Some(node.id) == state.aggregator;

        let (color, size) = if is_aggregator {
            (Color32::from_rgb(255, 200, 50), 16.0) // Gold for aggregator
        } else if node.is_selected {
            (Color32::from_rgb(100, 200, 255), 12.0) // Blue for selected
        } else {
            (Color32::from_rgb(100, 150, 100), 10.0) // Green for idle
        };

        // Outer glow
        painter.circle_filled(
            pos,
            size + 4.0,
            Color32::from_rgba_premultiplied(color.r(), color.g(), color.b(), 40),
        );

        // Main node
        painter.circle_filled(pos, size, color);
        painter.circle_stroke(pos, size, Stroke::new(1.5, Color32::WHITE));

        // Node label
        let label = if is_aggregator {
            "AGG"
        } else {
            &format!("N{}", node.id)
        };
        painter.text(
            pos,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::monospace(8.0),
            Color32::WHITE,
        );

        // Training progress bar
        if !is_aggregator && node.training_progress > 0.0 {
            let bar_width = 20.0;
            let bar_height = 4.0;
            let bar_pos = pos + Vec2::new(-bar_width / 2.0, size + 5.0);

            painter.rect_filled(
                egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width, bar_height)),
                1.0,
                Color32::from_rgb(40, 40, 40),
            );
            painter.rect_filled(
                egui::Rect::from_min_size(
                    bar_pos,
                    Vec2::new(bar_width * node.training_progress, bar_height),
                ),
                1.0,
                Color32::from_rgb(100, 200, 100),
            );
        }
    }

    // Draw global model indicator in center
    let center = world_to_screen(Pos2::ZERO);
    painter.text(
        center + Vec2::new(0.0, 70.0),
        egui::Align2::CENTER_CENTER,
        format!(
            "Round: {} | Accuracy: {:.1}%",
            state.round,
            state.current_accuracy * 100.0
        ),
        egui::FontId::monospace(10.0),
        Color32::from_rgb(200, 200, 200),
    );

    // Draw model weights visualization
    let weights_start = center + Vec2::new(-40.0, 80.0);
    for (i, weight) in state.global_model.iter().enumerate() {
        let x = weights_start.x + i as f32 * 18.0;
        let height = weight * 30.0;
        painter.rect_filled(
            egui::Rect::from_min_max(
                Pos2::new(x, weights_start.y + 30.0 - height),
                Pos2::new(x + 12.0, weights_start.y + 30.0),
            ),
            2.0,
            Color32::from_rgb(100, 180, 255),
        );
    }
    painter.text(
        weights_start + Vec2::new(40.0, 40.0),
        egui::Align2::CENTER_TOP,
        "Global Model",
        egui::FontId::monospace(8.0),
        Color32::from_rgb(150, 150, 150),
    );
}
