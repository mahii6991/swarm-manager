//! Color themes and styling

use egui::{Color32, Stroke, Visuals};

/// Dark theme optimized for visualization
pub fn dark_theme() -> Visuals {
    let mut visuals = Visuals::dark();
    visuals.window_fill = Color32::from_rgb(25, 25, 30);
    visuals.panel_fill = Color32::from_rgb(30, 30, 35);
    visuals.faint_bg_color = Color32::from_rgb(35, 35, 40);
    visuals.extreme_bg_color = Color32::from_rgb(20, 20, 25);
    visuals
}

// ============ Drone Colors ============

pub mod drone {
    use super::*;

    pub const ACTIVE: Color32 = Color32::from_rgb(0, 200, 100);
    pub const IDLE: Color32 = Color32::from_rgb(100, 100, 100);
    pub const RETURNING: Color32 = Color32::from_rgb(255, 200, 0);
    pub const EMERGENCY: Color32 = Color32::from_rgb(255, 80, 80);
    pub const FAILED: Color32 = Color32::from_rgb(80, 80, 80);

    pub const TRAIL: Color32 = Color32::from_rgba_premultiplied(100, 150, 255, 80);
    pub const VELOCITY: Color32 = Color32::from_rgb(255, 255, 100);
    pub const TARGET: Color32 = Color32::from_rgba_premultiplied(255, 100, 100, 100);
}

// ============ PSO Colors ============

pub mod pso {
    use super::*;

    pub const PARTICLE: Color32 = Color32::from_rgb(100, 150, 255);
    pub const VELOCITY: Color32 = Color32::from_rgba_premultiplied(100, 150, 255, 100);
    pub const PBEST: Color32 = Color32::from_rgb(150, 200, 150);
    pub const GBEST: Color32 = Color32::from_rgb(255, 215, 0); // Gold
    pub const GBEST_RING: Color32 = Color32::from_rgba_premultiplied(255, 215, 0, 100);
}

// ============ ACO Colors ============

pub mod aco {
    use super::*;

    pub const ANT: Color32 = Color32::from_rgb(200, 100, 50);
    pub const PHEROMONE_LOW: Color32 = Color32::from_rgba_premultiplied(50, 100, 50, 50);
    pub const PHEROMONE_HIGH: Color32 = Color32::from_rgba_premultiplied(100, 255, 100, 150);
    pub const OBSTACLE: Color32 = Color32::from_rgb(100, 50, 50);
    pub const BEST_PATH: Color32 = Color32::from_rgb(50, 255, 50);
    pub const START: Color32 = Color32::from_rgb(50, 150, 255);
    pub const GOAL: Color32 = Color32::from_rgb(255, 100, 100);
}

// ============ GWO Colors ============

pub mod gwo {
    use super::*;

    pub const ALPHA: Color32 = Color32::from_rgb(255, 215, 0); // Gold
    pub const BETA: Color32 = Color32::from_rgb(192, 192, 192); // Silver
    pub const DELTA: Color32 = Color32::from_rgb(205, 127, 50); // Bronze
    pub const OMEGA: Color32 = Color32::from_rgb(100, 100, 100); // Gray

    pub const ALPHA_RING: Color32 = Color32::from_rgba_premultiplied(255, 215, 0, 80);
}

// ============ Network Colors ============

pub mod network {
    use super::*;

    pub const NODE: Color32 = Color32::from_rgb(100, 150, 200);
    pub const NODE_HIGHLIGHT: Color32 = Color32::from_rgb(150, 200, 255);

    pub fn link_color(quality: f32) -> Color32 {
        // Green (good) to Red (bad)
        let r = ((1.0 - quality) * 255.0) as u8;
        let g = (quality * 200.0) as u8;
        Color32::from_rgb(r, g, 50)
    }

    pub fn link_stroke(quality: f32) -> Stroke {
        Stroke::new(1.0 + quality * 2.0, link_color(quality))
    }
}

// ============ UI Colors ============

pub mod ui {
    use super::*;

    pub const GRID: Color32 = Color32::from_rgba_premultiplied(50, 50, 60, 100);
    pub const GRID_MAJOR: Color32 = Color32::from_rgba_premultiplied(70, 70, 80, 150);
    pub const AXIS_X: Color32 = Color32::from_rgb(200, 80, 80);
    pub const AXIS_Y: Color32 = Color32::from_rgb(80, 200, 80);

    pub const BOUNDS: Color32 = Color32::from_rgba_premultiplied(100, 100, 150, 50);
    pub const TEXT: Color32 = Color32::from_rgb(200, 200, 200);
    pub const TEXT_DIM: Color32 = Color32::from_rgb(120, 120, 120);
}

// ============ Graph Colors ============

pub mod graph {
    use super::*;

    pub const COST_LINE: Color32 = Color32::from_rgb(100, 200, 255);
    pub const FITNESS_LINE: Color32 = Color32::from_rgb(255, 150, 100);
    pub const GRID: Color32 = Color32::from_rgba_premultiplied(50, 50, 60, 100);
}
