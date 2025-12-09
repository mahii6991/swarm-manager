//! Algorithm visualization panel with tabs for PSO, ACO, GWO, Federated, Consensus, DE

use egui::{Ui, Pos2, Vec2, Rect, Color32, Stroke};
use crate::state::{SimulationState, AlgorithmType};
use crate::themes;
use crate::renderers::{pso_renderer, aco_renderer, gwo_renderer, federated_renderer, consensus_renderer, de_renderer};

pub fn show(ui: &mut Ui, state: &mut SimulationState) {
    ui.horizontal(|ui| {
        ui.heading("Algorithm Visualization");
        ui.separator();

        // Tab buttons - Swarm Intelligence
        if ui.selectable_label(state.active_algorithm == AlgorithmType::PSO, "PSO").clicked() {
            state.active_algorithm = AlgorithmType::PSO;
        }
        if ui.selectable_label(state.active_algorithm == AlgorithmType::ACO, "ACO").clicked() {
            state.active_algorithm = AlgorithmType::ACO;
        }
        if ui.selectable_label(state.active_algorithm == AlgorithmType::GWO, "GWO").clicked() {
            state.active_algorithm = AlgorithmType::GWO;
        }
        ui.separator();
        // Distributed Systems
        if ui.selectable_label(state.active_algorithm == AlgorithmType::Federated, "FL").clicked() {
            state.active_algorithm = AlgorithmType::Federated;
        }
        if ui.selectable_label(state.active_algorithm == AlgorithmType::Consensus, "Raft").clicked() {
            state.active_algorithm = AlgorithmType::Consensus;
        }
        ui.separator();
        // Evolutionary
        if ui.selectable_label(state.active_algorithm == AlgorithmType::DE, "DE").clicked() {
            state.active_algorithm = AlgorithmType::DE;
        }
    });

    ui.separator();

    let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
    let rect = response.rect;

    // Background
    painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 22, 25));

    // Draw bounds
    let bounds = 100.0;
    let scale = (rect.width().min(rect.height()) - 40.0) / (bounds * 2.0);
    let center = rect.center();

    let world_to_screen = |world_pos: Pos2| -> Pos2 {
        Pos2::new(
            center.x + world_pos.x * scale,
            center.y - world_pos.y * scale,
        )
    };

    // Draw grid
    draw_algorithm_grid(&painter, rect, center, scale, bounds);

    // Draw based on active algorithm
    match state.active_algorithm {
        AlgorithmType::PSO => {
            if let Some(ref pso) = state.pso_state {
                pso_renderer::draw(&painter, pso, world_to_screen, scale);

                // Info text
                let info = format!(
                    "PSO | Iteration: {} | Best: {:.4} | Particles: {}",
                    pso.iteration, pso.gbest_cost, pso.particles.len()
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
        AlgorithmType::ACO => {
            if let Some(ref aco) = state.aco_state {
                aco_renderer::draw(&painter, aco, world_to_screen, scale);

                let info = format!(
                    "ACO | Iteration: {} | Ants: {} | Path: {} waypoints",
                    aco.iteration, aco.ants.len(), aco.best_path.len()
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
        AlgorithmType::GWO => {
            if let Some(ref gwo) = state.gwo_state {
                gwo_renderer::draw(&painter, gwo, world_to_screen, scale);

                let alpha_fitness = gwo.alpha.as_ref().map(|a| a.fitness).unwrap_or(0.0);
                let info = format!(
                    "GWO | Iteration: {} | Wolves: {} | Alpha: {:.4}",
                    gwo.iteration, gwo.wolves.len(), alpha_fitness
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
        AlgorithmType::Federated => {
            if let Some(ref fed) = state.federated_state {
                federated_renderer::draw(&painter, fed, world_to_screen, scale);

                let info = format!(
                    "Federated Learning | Round: {} | Nodes: {} | Accuracy: {:.1}%",
                    fed.round, fed.nodes.len(), fed.current_accuracy * 100.0
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
        AlgorithmType::Consensus => {
            if let Some(ref cons) = state.consensus_state {
                consensus_renderer::draw(&painter, cons, world_to_screen, scale);

                let leader_str = cons.leader_id.map(|l| format!("N{}", l)).unwrap_or_else(|| "None".to_string());
                let info = format!(
                    "SwarmRaft | Term: {} | Leader: {} | Committed: {}",
                    cons.term, leader_str, cons.committed_index
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
        AlgorithmType::DE => {
            if let Some(ref de) = state.de_state {
                de_renderer::draw(&painter, de, world_to_screen, scale);

                let info = format!(
                    "Differential Evolution | Iter: {} | Pop: {} | Best: {:.4}",
                    de.iteration, de.population.len(), de.best_fitness
                );
                painter.text(
                    rect.min + Vec2::new(10.0, 10.0),
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::monospace(11.0),
                    themes::ui::TEXT,
                );
            }
        }
    }
}

fn draw_algorithm_grid(painter: &egui::Painter, rect: Rect, center: Pos2, scale: f32, bounds: f32) {
    let grid_spacing = 20.0 * scale;

    // Draw grid lines
    let mut x = center.x - bounds * scale;
    while x <= center.x + bounds * scale {
        let alpha = if (x - center.x).abs() < 1.0 { 100 } else { 30 };
        painter.line_segment(
            [Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)],
            Stroke::new(0.5, Color32::from_rgba_premultiplied(50, 50, 60, alpha)),
        );
        x += grid_spacing;
    }

    let mut y = center.y - bounds * scale;
    while y <= center.y + bounds * scale {
        let alpha = if (y - center.y).abs() < 1.0 { 100 } else { 30 };
        painter.line_segment(
            [Pos2::new(rect.min.x, y), Pos2::new(rect.max.x, y)],
            Stroke::new(0.5, Color32::from_rgba_premultiplied(50, 50, 60, alpha)),
        );
        y += grid_spacing;
    }

    // Draw axes
    painter.line_segment(
        [Pos2::new(center.x, rect.min.y), Pos2::new(center.x, rect.max.y)],
        Stroke::new(1.0, themes::ui::AXIS_Y),
    );
    painter.line_segment(
        [Pos2::new(rect.min.x, center.y), Pos2::new(rect.max.x, center.y)],
        Stroke::new(1.0, themes::ui::AXIS_X),
    );

    // Draw bounds rectangle
    let bounds_rect = Rect::from_center_size(center, Vec2::splat(bounds * 2.0 * scale));
    painter.rect_stroke(bounds_rect, 0.0, Stroke::new(1.0, themes::ui::BOUNDS));
}
