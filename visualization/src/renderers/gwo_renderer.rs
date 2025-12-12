//! GWO (Grey Wolf Optimizer) rendering

use crate::state::{GWOVisualState, WolfRank};
use crate::themes;
use egui::{Color32, Painter, Pos2, Stroke, Vec2};

/// Draw GWO visualization
pub fn draw<F>(painter: &Painter, gwo: &GWOVisualState, world_to_screen: F, _scale: f32)
where
    F: Fn(Pos2) -> Pos2,
{
    // Draw connections from omega wolves to leaders
    if let (Some(ref alpha), Some(ref beta), Some(ref delta)) = (&gwo.alpha, &gwo.beta, &gwo.delta)
    {
        let alpha_pos = world_to_screen(alpha.position);
        let beta_pos = world_to_screen(beta.position);
        let delta_pos = world_to_screen(delta.position);

        for wolf in &gwo.wolves {
            if wolf.rank == WolfRank::Omega {
                let pos = world_to_screen(wolf.position);

                // Draw faint lines to all three leaders
                painter.line_segment(
                    [pos, alpha_pos],
                    Stroke::new(0.5, Color32::from_rgba_premultiplied(255, 215, 0, 30)),
                );
                painter.line_segment(
                    [pos, beta_pos],
                    Stroke::new(0.5, Color32::from_rgba_premultiplied(192, 192, 192, 20)),
                );
                painter.line_segment(
                    [pos, delta_pos],
                    Stroke::new(0.5, Color32::from_rgba_premultiplied(205, 127, 50, 20)),
                );
            }
        }
    }

    // Draw omega wolves first (so leaders appear on top)
    for wolf in &gwo.wolves {
        if wolf.rank == WolfRank::Omega {
            let pos = world_to_screen(wolf.position);
            draw_wolf(painter, pos, wolf.rank, wolf.fitness);
        }
    }

    // Draw delta
    if let Some(ref delta) = gwo.delta {
        let pos = world_to_screen(delta.position);
        draw_wolf(painter, pos, WolfRank::Delta, delta.fitness);
        painter.text(
            pos + Vec2::new(0.0, -20.0),
            egui::Align2::CENTER_BOTTOM,
            format!("δ {:.2}", delta.fitness),
            egui::FontId::monospace(9.0),
            themes::gwo::DELTA,
        );
    }

    // Draw beta
    if let Some(ref beta) = gwo.beta {
        let pos = world_to_screen(beta.position);
        draw_wolf(painter, pos, WolfRank::Beta, beta.fitness);
        painter.text(
            pos + Vec2::new(0.0, -22.0),
            egui::Align2::CENTER_BOTTOM,
            format!("β {:.2}", beta.fitness),
            egui::FontId::monospace(10.0),
            themes::gwo::BETA,
        );
    }

    // Draw alpha (leader) with special highlight
    if let Some(ref alpha) = gwo.alpha {
        let pos = world_to_screen(alpha.position);

        // Outer glow rings
        painter.circle_stroke(pos, 25.0, Stroke::new(1.0, themes::gwo::ALPHA_RING));
        painter.circle_stroke(pos, 20.0, Stroke::new(2.0, themes::gwo::ALPHA_RING));

        draw_wolf(painter, pos, WolfRank::Alpha, alpha.fitness);

        painter.text(
            pos + Vec2::new(0.0, -25.0),
            egui::Align2::CENTER_BOTTOM,
            format!("α {:.2}", alpha.fitness),
            egui::FontId::monospace(11.0),
            themes::gwo::ALPHA,
        );
    }

    // Draw convergence parameter indicator
    let a = gwo.convergence_param;
    painter.text(
        world_to_screen(Pos2::new(-90.0, 90.0)),
        egui::Align2::LEFT_TOP,
        format!("a = {:.2}", a),
        egui::FontId::monospace(10.0),
        themes::ui::TEXT,
    );

    // Draw a small bar showing convergence progress
    let bar_pos = world_to_screen(Pos2::new(-90.0, 85.0));
    let bar_width = 50.0;
    let bar_height = 6.0;

    painter.rect_filled(
        egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width, bar_height)),
        2.0,
        Color32::from_rgb(40, 40, 40),
    );

    let progress = 1.0 - (a / 2.0); // a goes from 2 to 0
    painter.rect_filled(
        egui::Rect::from_min_size(bar_pos, Vec2::new(bar_width * progress, bar_height)),
        2.0,
        themes::gwo::ALPHA,
    );
}

/// Draw a single wolf
fn draw_wolf(painter: &Painter, pos: Pos2, rank: WolfRank, _fitness: f32) {
    let (color, size) = match rank {
        WolfRank::Alpha => (themes::gwo::ALPHA, 12.0),
        WolfRank::Beta => (themes::gwo::BETA, 10.0),
        WolfRank::Delta => (themes::gwo::DELTA, 9.0),
        WolfRank::Omega => (themes::gwo::OMEGA, 6.0),
    };

    // Draw wolf as a diamond shape for leaders, circle for omega
    if rank == WolfRank::Omega {
        painter.circle_filled(pos, size, color);
        painter.circle_stroke(
            pos,
            size,
            Stroke::new(1.0, Color32::from_rgb(150, 150, 150)),
        );
    } else {
        // Diamond shape
        let points = vec![
            pos + Vec2::new(0.0, -size),
            pos + Vec2::new(size * 0.7, 0.0),
            pos + Vec2::new(0.0, size),
            pos + Vec2::new(-size * 0.7, 0.0),
        ];
        painter.add(egui::Shape::convex_polygon(
            points,
            color,
            Stroke::new(2.0, Color32::WHITE),
        ));
    }
}
