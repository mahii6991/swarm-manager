//! Network topology visualization panel

use crate::renderers::network_renderer;
use crate::state::SimulationState;
use crate::themes;
use egui::{Color32, Pos2, Stroke, Ui, Vec2};

pub fn show(ui: &mut Ui, state: &SimulationState) {
    ui.heading("Network Topology");
    ui.separator();

    let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
    let rect = response.rect;

    // Background
    painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 22, 25));

    if state.network.nodes.is_empty() {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "No network data",
            egui::FontId::monospace(14.0),
            themes::ui::TEXT_DIM,
        );
        return;
    }

    // Calculate scale to fit network in view
    let padding = 30.0;
    let available = rect.shrink(padding);

    // Find bounds of all nodes
    let (min_x, max_x, min_y, max_y) = state.network.nodes.iter().fold(
        (f32::MAX, f32::MIN, f32::MAX, f32::MIN),
        |(min_x, max_x, min_y, max_y), node| {
            (
                min_x.min(node.position.x),
                max_x.max(node.position.x),
                min_y.min(node.position.y),
                max_y.max(node.position.y),
            )
        },
    );

    let world_width = (max_x - min_x).max(1.0);
    let world_height = (max_y - min_y).max(1.0);
    let world_center = Pos2::new((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);

    let scale_x = available.width() / world_width;
    let scale_y = available.height() / world_height;
    let scale = scale_x.min(scale_y) * 0.9;

    let screen_center = available.center();

    let world_to_screen = |world_pos: Pos2| -> Pos2 {
        let offset = world_pos - world_center;
        Pos2::new(
            screen_center.x + offset.x * scale,
            screen_center.y - offset.y * scale,
        )
    };

    // Draw network
    network_renderer::draw(&painter, &state.network, world_to_screen);

    // Info text
    let info = format!(
        "Nodes: {} | Edges: {}",
        state.network.nodes.len(),
        state.network.edges.len()
    );
    painter.text(
        rect.min + Vec2::new(10.0, 10.0),
        egui::Align2::LEFT_TOP,
        info,
        egui::FontId::monospace(11.0),
        themes::ui::TEXT,
    );

    // Legend
    let legend_y = rect.max.y - 25.0;
    painter.text(
        Pos2::new(rect.min.x + 10.0, legend_y),
        egui::Align2::LEFT_CENTER,
        "Link Quality:",
        egui::FontId::monospace(9.0),
        themes::ui::TEXT_DIM,
    );

    // Draw quality gradient legend
    let legend_start = rect.min.x + 80.0;
    let legend_width = 60.0;
    for i in 0..20 {
        let t = i as f32 / 20.0;
        let x = legend_start + t * legend_width;
        let color = themes::network::link_color(t);
        painter.line_segment(
            [Pos2::new(x, legend_y - 5.0), Pos2::new(x, legend_y + 5.0)],
            Stroke::new(3.0, color),
        );
    }

    painter.text(
        Pos2::new(legend_start - 5.0, legend_y),
        egui::Align2::RIGHT_CENTER,
        "Bad",
        egui::FontId::monospace(8.0),
        themes::ui::TEXT_DIM,
    );
    painter.text(
        Pos2::new(legend_start + legend_width + 5.0, legend_y),
        egui::Align2::LEFT_CENTER,
        "Good",
        egui::FontId::monospace(8.0),
        themes::ui::TEXT_DIM,
    );
}
