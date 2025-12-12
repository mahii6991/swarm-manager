//! Metrics and statistics panel with graphs

use crate::state::{AlgorithmType, DroneStatus, SimulationState};
use crate::themes;
use egui::Ui;
use egui_plot::{Line, Plot, PlotPoints};

pub fn show(ui: &mut Ui, state: &SimulationState) {
    ui.heading("Metrics");
    ui.add_space(5.0);

    // Drone Statistics
    ui.collapsing("Drone Stats", |ui| {
        let active = state
            .drones
            .iter()
            .filter(|d| d.status == DroneStatus::Active)
            .count();
        let returning = state
            .drones
            .iter()
            .filter(|d| d.status == DroneStatus::Returning)
            .count();
        let emergency = state
            .drones
            .iter()
            .filter(|d| d.status == DroneStatus::Emergency)
            .count();

        ui.horizontal(|ui| {
            ui.label("Total:");
            ui.label(format!("{}", state.drones.len()));
        });
        ui.horizontal(|ui| {
            ui.colored_label(themes::drone::ACTIVE, "Active:");
            ui.label(format!("{}", active));
        });
        ui.horizontal(|ui| {
            ui.colored_label(themes::drone::RETURNING, "Returning:");
            ui.label(format!("{}", returning));
        });
        ui.horizontal(|ui| {
            ui.colored_label(themes::drone::EMERGENCY, "Emergency:");
            ui.label(format!("{}", emergency));
        });

        // Average battery
        if !state.drones.is_empty() {
            let avg_battery: f32 = state.drones.iter().map(|d| d.battery as f32).sum::<f32>()
                / state.drones.len() as f32;
            ui.add_space(5.0);
            ui.horizontal(|ui| {
                ui.label("Avg Battery:");
                let color = if avg_battery > 50.0 {
                    themes::drone::ACTIVE
                } else if avg_battery > 20.0 {
                    themes::drone::RETURNING
                } else {
                    themes::drone::EMERGENCY
                };
                ui.colored_label(color, format!("{:.0}%", avg_battery));
            });
        }
    });

    ui.add_space(10.0);

    // Network Statistics
    ui.collapsing("Network Stats", |ui| {
        ui.horizontal(|ui| {
            ui.label("Nodes:");
            ui.label(format!("{}", state.network.nodes.len()));
        });
        ui.horizontal(|ui| {
            ui.label("Edges:");
            ui.label(format!("{}", state.network.edges.len()));
        });

        if !state.network.edges.is_empty() {
            let avg_quality: f32 = state
                .network
                .edges
                .iter()
                .map(|e| e.link_quality)
                .sum::<f32>()
                / state.network.edges.len() as f32;
            ui.horizontal(|ui| {
                ui.label("Avg Link Quality:");
                ui.label(format!("{:.2}", avg_quality));
            });
        }
    });

    ui.add_space(10.0);

    // Simulation Stats
    ui.collapsing("Simulation", |ui| {
        ui.horizontal(|ui| {
            ui.label("Time Step:");
            ui.label(format!("{}", state.time_step));
        });
        ui.horizontal(|ui| {
            ui.label("Speed:");
            ui.label(format!("{:.1}x", state.simulation_speed));
        });
        ui.horizontal(|ui| {
            ui.label("Status:");
            if state.is_running {
                ui.colored_label(themes::drone::ACTIVE, "Running");
            } else {
                ui.colored_label(themes::drone::RETURNING, "Paused");
            }
        });
    });

    ui.add_space(10.0);

    // Cost/Fitness Graph
    ui.collapsing("Convergence Graph", |ui| {
        let data: Vec<[f64; 2]> = match state.active_algorithm {
            AlgorithmType::PSO => state
                .pso_state
                .as_ref()
                .map(|pso| {
                    pso.cost_history
                        .iter()
                        .enumerate()
                        .map(|(i, &c)| [i as f64, c as f64])
                        .collect()
                })
                .unwrap_or_default(),
            AlgorithmType::GWO => state
                .gwo_state
                .as_ref()
                .map(|gwo| {
                    gwo.fitness_history
                        .iter()
                        .enumerate()
                        .map(|(i, &f)| [i as f64, f as f64])
                        .collect()
                })
                .unwrap_or_default(),
            AlgorithmType::ACO => {
                // ACO doesn't have a direct cost history
                vec![]
            }
            AlgorithmType::Federated => state
                .federated_state
                .as_ref()
                .map(|fed| {
                    fed.accuracy_history
                        .iter()
                        .enumerate()
                        .map(|(i, &a)| [i as f64, a as f64 * 100.0])
                        .collect()
                })
                .unwrap_or_default(),
            AlgorithmType::Consensus => {
                // Consensus doesn't have convergence history
                vec![]
            }
            AlgorithmType::DE => state
                .de_state
                .as_ref()
                .map(|de| {
                    de.fitness_history
                        .iter()
                        .enumerate()
                        .map(|(i, &f)| [i as f64, f as f64])
                        .collect()
                })
                .unwrap_or_default(),
        };

        if data.is_empty() {
            ui.label("No data yet");
        } else {
            let line = Line::new(PlotPoints::new(data))
                .color(match state.active_algorithm {
                    AlgorithmType::PSO => themes::graph::COST_LINE,
                    AlgorithmType::GWO => themes::graph::FITNESS_LINE,
                    AlgorithmType::ACO => themes::graph::COST_LINE,
                    AlgorithmType::Federated => themes::graph::FITNESS_LINE,
                    AlgorithmType::Consensus => themes::graph::COST_LINE,
                    AlgorithmType::DE => themes::graph::FITNESS_LINE,
                })
                .name(match state.active_algorithm {
                    AlgorithmType::PSO => "Cost",
                    AlgorithmType::GWO => "Fitness",
                    AlgorithmType::ACO => "Path Length",
                    AlgorithmType::Federated => "Accuracy %",
                    AlgorithmType::Consensus => "Term",
                    AlgorithmType::DE => "Best Fitness",
                });

            Plot::new("convergence_plot")
                .height(120.0)
                .show_axes(true)
                .show_grid(true)
                .allow_drag(false)
                .allow_zoom(false)
                .show(ui, |plot_ui| {
                    plot_ui.line(line);
                });
        }
    });
}
