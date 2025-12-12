//! PSO (Particle Swarm Optimization) rendering

use crate::state::PSOVisualState;
use crate::themes;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw PSO visualization
pub fn draw<F>(painter: &Painter, pso: &PSOVisualState, world_to_screen: F, scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    // Draw pbest connections (faint dotted lines)
    for particle in &pso.particles {
        let pos = world_to_screen(particle.position);
        let pbest = world_to_screen(particle.pbest_position);

        // Only draw if different from current position
        if (particle.position - particle.pbest_position).length() > 1.0 {
            painter.line_segment(
                [pos, pbest],
                Stroke::new(0.5, Color32::from_rgba_premultiplied(150, 200, 150, 50)),
            );
            // Small marker at pbest
            painter.circle_filled(pbest, 2.0, themes::pso::PBEST);
        }
    }

    // Draw velocity vectors
    for particle in &pso.particles {
        let pos = world_to_screen(particle.position);
        if particle.velocity.length() > 0.1 {
            let vel_end = pos + particle.velocity * scale * 2.0;
            painter.arrow(pos, vel_end - pos, Stroke::new(1.0, themes::pso::VELOCITY));
        }
    }

    // Draw particles
    for particle in &pso.particles {
        let pos = world_to_screen(particle.position);
        painter.circle_filled(pos, 5.0, themes::pso::PARTICLE);
        painter.circle_stroke(pos, 5.0, Stroke::new(1.0, Color32::WHITE));
    }

    // Draw global best with highlight
    let gbest = world_to_screen(pso.gbest_position);

    // Outer ring
    painter.circle_stroke(gbest, 15.0, Stroke::new(2.0, themes::pso::GBEST_RING));
    painter.circle_stroke(gbest, 20.0, Stroke::new(1.0, themes::pso::GBEST_RING));

    // Inner star marker
    draw_star(painter, gbest, 8.0, themes::pso::GBEST);

    // Label
    painter.text(
        gbest + Vec2::new(0.0, -25.0),
        egui::Align2::CENTER_BOTTOM,
        format!("Best: {:.4}", pso.gbest_cost),
        egui::FontId::monospace(10.0),
        themes::pso::GBEST,
    );
}

/// Draw a star shape
fn draw_star(painter: &Painter, center: Pos2, size: f32, color: Color32) {
    let points = 5;
    let outer_radius = size;
    let inner_radius = size * 0.4;

    let mut vertices = Vec::new();
    for i in 0..(points * 2) {
        let angle =
            (i as f32 / (points * 2) as f32) * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        let radius = if i % 2 == 0 {
            outer_radius
        } else {
            inner_radius
        };
        vertices.push(center + Vec2::new(angle.cos() * radius, angle.sin() * radius));
    }

    painter.add(egui::Shape::convex_polygon(
        vertices,
        color,
        Stroke::new(1.0, Color32::WHITE),
    ));
}
