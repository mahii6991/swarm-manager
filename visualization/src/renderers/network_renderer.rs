//! Network topology rendering

use crate::state::NetworkTopology;
use crate::themes;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw network topology
pub fn draw<F>(painter: &Painter, network: &NetworkTopology, world_to_screen: F)
where
    F: Fn(Pos2) -> Pos2,
{
    // Draw edges first (so nodes appear on top)
    for edge in &network.edges {
        let from_node = network.nodes.iter().find(|n| n.id == edge.from);
        let to_node = network.nodes.iter().find(|n| n.id == edge.to);

        if let (Some(from), Some(to)) = (from_node, to_node) {
            let from_pos = world_to_screen(from.position);
            let to_pos = world_to_screen(to.position);

            // Draw edge with color based on link quality
            let stroke = themes::network::link_stroke(edge.link_quality);
            painter.line_segment([from_pos, to_pos], stroke);

            // Draw RTT label at midpoint for good links
            if edge.link_quality > 0.5 {
                let mid = Pos2::new((from_pos.x + to_pos.x) / 2.0, (from_pos.y + to_pos.y) / 2.0);
                painter.text(
                    mid,
                    egui::Align2::CENTER_CENTER,
                    format!("{}ms", edge.rtt_ms),
                    egui::FontId::monospace(8.0),
                    Color32::from_rgba_premultiplied(150, 150, 150, 150),
                );
            }
        }
    }

    // Draw nodes
    for node in &network.nodes {
        let pos = world_to_screen(node.position);

        // Node size based on neighbor count
        let base_size = 8.0;
        let size = base_size + node.neighbor_count as f32 * 1.5;

        // Outer glow
        painter.circle_filled(
            pos,
            size + 3.0,
            Color32::from_rgba_premultiplied(100, 150, 200, 30),
        );

        // Main node
        painter.circle_filled(pos, size, themes::network::NODE);
        painter.circle_stroke(pos, size, Stroke::new(1.5, themes::network::NODE_HIGHLIGHT));

        // Node ID label
        painter.text(
            pos,
            egui::Align2::CENTER_CENTER,
            format!("{}", node.id),
            egui::FontId::monospace(9.0),
            Color32::WHITE,
        );

        // Neighbor count indicator
        if node.neighbor_count > 0 {
            let badge_pos = pos + Vec2::new(size + 2.0, -size - 2.0);
            painter.circle_filled(badge_pos, 7.0, Color32::from_rgb(60, 60, 70));
            painter.text(
                badge_pos,
                egui::Align2::CENTER_CENTER,
                format!("{}", node.neighbor_count),
                egui::FontId::monospace(8.0),
                Color32::from_rgb(200, 200, 200),
            );
        }
    }
}
