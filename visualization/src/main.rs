//! Drone Swarm Visualization
//!
//! Real-time visualization for drone swarm algorithms including:
//! - 2D drone position viewport with formations
//! - PSO/ACO/GWO algorithm visualization
//! - Network topology display
//! - Interactive parameter tuning

mod app;
mod panels;
mod renderers;
mod simulation;
mod state;
mod themes;

use app::DroneSwarmApp;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 600.0])
            .with_title("Drone Swarm Visualization"),
        ..Default::default()
    };

    eframe::run_native(
        "Drone Swarm Visualization",
        native_options,
        Box::new(|cc| Ok(Box::new(DroneSwarmApp::new(cc)))),
    )
}
