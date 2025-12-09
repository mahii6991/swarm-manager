//! Differential Evolution visualization rendering

use egui::{Painter, Pos2, Vec2, Color32, Stroke};
use crate::state::DEVisualState;

/// Draw Differential Evolution visualization
pub fn draw<F>(painter: &Painter, state: &DEVisualState, world_to_screen: F, _scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    let center = world_to_screen(Pos2::ZERO);

    // Draw crossover visualization for active trials
    if state.iteration % 2 == 0 {
        // Show crossover happening
        for individual in &state.population {
            if individual.is_trial {
                let pos = world_to_screen(individual.position);
                let pulse = ((state.iteration as f32 * 0.3) % 1.0) * 10.0;
                painter.circle_stroke(
                    pos,
                    8.0 + pulse,
                    Stroke::new(1.0, Color32::from_rgba_premultiplied(255, 200, 100, 100)),
                );
            }
        }
    }

    // Draw individuals
    for (idx, individual) in state.population.iter().enumerate() {
        let pos = world_to_screen(individual.position);
        let is_best = (individual.fitness - state.best_fitness).abs() < 0.001;

        // Determine color based on state
        let (color, size) = if is_best {
            (Color32::from_rgb(255, 215, 0), 14.0) // Gold for best
        } else if individual.is_trial {
            (Color32::from_rgb(255, 150, 50), 10.0) // Orange for trial vectors
        } else {
            // Color based on fitness (better = more blue)
            let fitness_ratio = if state.best_fitness > 0.001 {
                (state.best_fitness / individual.fitness.max(0.001)).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let r = (100.0 + 100.0 * (1.0 - fitness_ratio)) as u8;
            let g = (150.0 + 50.0 * fitness_ratio) as u8;
            let b = (150.0 + 105.0 * fitness_ratio) as u8;
            (Color32::from_rgb(r, g, b), 10.0)
        };

        // Outer glow for best
        if is_best {
            painter.circle_filled(
                pos,
                size + 6.0,
                Color32::from_rgba_premultiplied(255, 215, 0, 50),
            );
        }

        // Main circle
        painter.circle_filled(pos, size, color);
        painter.circle_stroke(pos, size, Stroke::new(1.5, Color32::WHITE));

        // Show fitness value for best individual
        if is_best {
            painter.text(
                pos,
                egui::Align2::CENTER_CENTER,
                "BEST",
                egui::FontId::monospace(7.0),
                Color32::BLACK,
            );
        }

        // Individual index
        painter.text(
            pos + Vec2::new(0.0, size + 6.0),
            egui::Align2::CENTER_TOP,
            format!("#{}", idx),
            egui::FontId::monospace(8.0),
            Color32::from_rgb(150, 150, 150),
        );
    }

    // Draw parameter info
    painter.text(
        center + Vec2::new(0.0, 70.0),
        egui::Align2::CENTER_CENTER,
        format!("Iter: {} | Best: {:.4} | Pop: {}",
            state.iteration, state.best_fitness, state.population.len()),
        egui::FontId::monospace(10.0),
        Color32::from_rgb(200, 200, 200),
    );

    // Draw DE parameters visualization
    let params_start = center + Vec2::new(-60.0, 85.0);

    // F (mutation factor) bar
    painter.text(
        params_start,
        egui::Align2::LEFT_CENTER,
        "F:",
        egui::FontId::monospace(9.0),
        Color32::from_rgb(150, 150, 150),
    );
    let f_bar_pos = params_start + Vec2::new(15.0, -3.0);
    painter.rect_filled(
        egui::Rect::from_min_size(f_bar_pos, Vec2::new(40.0, 6.0)),
        1.0,
        Color32::from_rgb(40, 40, 40),
    );
    painter.rect_filled(
        egui::Rect::from_min_size(f_bar_pos, Vec2::new(40.0 * state.mutation_factor, 6.0)),
        1.0,
        Color32::from_rgb(255, 150, 50),
    );

    // CR (crossover rate) bar
    let cr_start = params_start + Vec2::new(65.0, 0.0);
    painter.text(
        cr_start,
        egui::Align2::LEFT_CENTER,
        "CR:",
        egui::FontId::monospace(9.0),
        Color32::from_rgb(150, 150, 150),
    );
    let cr_bar_pos = cr_start + Vec2::new(20.0, -3.0);
    painter.rect_filled(
        egui::Rect::from_min_size(cr_bar_pos, Vec2::new(40.0, 6.0)),
        1.0,
        Color32::from_rgb(40, 40, 40),
    );
    painter.rect_filled(
        egui::Rect::from_min_size(cr_bar_pos, Vec2::new(40.0 * state.crossover_rate, 6.0)),
        1.0,
        Color32::from_rgb(100, 200, 255),
    );

    // Convergence indicator based on fitness history
    let diversity = if state.fitness_history.len() > 10 {
        let recent: f32 = state.fitness_history.iter().rev().take(5).sum::<f32>() / 5.0;
        let older: f32 = state.fitness_history.iter().rev().skip(5).take(5).sum::<f32>() / 5.0;
        if older > 0.001 { (recent / older).clamp(0.0, 1.0) } else { 1.0 }
    } else {
        1.0
    };

    let conv_label = if diversity > 0.95 {
        "Exploring"
    } else if diversity > 0.5 {
        "Converging"
    } else {
        "Converged"
    };

    painter.text(
        center + Vec2::new(0.0, 105.0),
        egui::Align2::CENTER_CENTER,
        format!("Status: {}", conv_label),
        egui::FontId::monospace(9.0),
        Color32::from_rgb(150, 180, 150),
    );
}
