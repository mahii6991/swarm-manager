//! Main application state and UI orchestration

use crate::panels::{algorithm, controls, metrics, network, safety, viewport};
use crate::state::SimulationState;
use crate::themes;
use eframe::egui;

/// Main application struct implementing eframe::App
pub struct DroneSwarmApp {
    /// Simulation state containing all visualizable data
    pub state: SimulationState,
    /// Show the settings window
    show_settings: bool,
    /// Show the about window
    show_about: bool,
}

impl DroneSwarmApp {
    /// Create a new application instance
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            state: SimulationState::new(),
            show_settings: false,
            show_about: false,
        }
    }
}

impl eframe::App for DroneSwarmApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Apply dark theme
        ctx.set_visuals(themes::dark_theme());

        // Request continuous repaint when simulation is running
        if self.state.is_running {
            self.state.step();
            ctx.request_repaint();
        }

        // Top menu bar
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Reset Simulation").clicked() {
                        self.state = SimulationState::new();
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });

                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.state.viewport.show_grid, "Show Grid");
                    ui.checkbox(&mut self.state.viewport.show_trails, "Show Trails");
                    ui.checkbox(&mut self.state.viewport.show_velocities, "Show Velocities");
                });

                ui.menu_button("Simulation", |ui| {
                    if self.state.is_running {
                        if ui.button("Pause (Space)").clicked() {
                            self.state.is_running = false;
                            ui.close_menu();
                        }
                    } else {
                        if ui.button("Play (Space)").clicked() {
                            self.state.is_running = true;
                            ui.close_menu();
                        }
                    }
                    if ui.button("Step Once").clicked() {
                        self.state.step();
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Reset (R)").clicked() {
                        self.state.reset();
                        ui.close_menu();
                    }
                });

                ui.menu_button("Demo", |ui| {
                    if self.state.is_demo_active() {
                        ui.label(format!(
                            "Current: {}",
                            self.state
                                .demo_mode
                                .as_ref()
                                .map(|d| d.scenario_name())
                                .unwrap_or("None")
                        ));
                        ui.separator();
                        if ui.button("Stop Demo (D)").clicked() {
                            self.state.stop_demo();
                            ui.close_menu();
                        }
                    } else {
                        ui.label("Automated demo showcasing all features");
                        ui.separator();
                        if ui.button("Start Demo (D)").clicked() {
                            self.state.start_demo();
                            ui.close_menu();
                        }
                    }
                });

                ui.menu_button("Help", |ui| {
                    if ui.button("About").clicked() {
                        self.show_about = true;
                        ui.close_menu();
                    }
                });

                // Spacer
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let status = if self.state.is_running {
                        "Running"
                    } else {
                        "Paused"
                    };
                    ui.label(format!(
                        "Status: {} | Drones: {} | Step: {}",
                        status,
                        self.state.drones.len(),
                        self.state.time_step
                    ));
                });
            });
        });

        // Bottom status bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("FPS: {:.0}", ctx.input(|i| 1.0 / i.predicted_dt)));
                ui.separator();
                ui.label(format!("Formation: {:?}", self.state.formation));
                ui.separator();
                ui.label(format!("Algorithm: {:?}", self.state.active_algorithm));
                ui.separator();
                if let Some(ref pso) = self.state.pso_state {
                    ui.label(format!("PSO Best: {:.4}", pso.gbest_cost));
                }
                // Demo mode indicator
                if let Some(ref demo) = self.state.demo_mode {
                    ui.separator();
                    ui.label(
                        egui::RichText::new(format!("DEMO: {}", demo.scenario_name()))
                            .color(egui::Color32::from_rgb(255, 200, 100)),
                    );
                }
                // Selected drone indicator
                if let Some(drone_id) = self.state.selected_drone {
                    ui.separator();
                    if let Some(drone) = self.state.drones.iter().find(|d| d.id == drone_id) {
                        ui.label(format!(
                            "Selected: D{} | Pos: ({:.0}, {:.0}) | Battery: {}% | {:?}",
                            drone_id,
                            drone.position.x,
                            drone.position.y,
                            drone.battery,
                            drone.status
                        ));
                    }
                }
            });
        });

        // Left panel - Safety & Mission
        egui::SidePanel::left("left_panel")
            .min_width(250.0)
            .max_width(320.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    safety::show(ui, &mut self.state);
                });
            });

        // Right panel - Controls and Metrics
        egui::SidePanel::right("right_panel")
            .min_width(280.0)
            .max_width(350.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    // Parameter Controls
                    controls::show(ui, &mut self.state);

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // Metrics Panel
                    metrics::show(ui, &self.state);
                });
            });

        // Main central area
        egui::CentralPanel::default().show(ctx, |ui| {
            // Split vertically: top for viewport, bottom for algorithm and network
            let available = ui.available_size();
            let top_height = available.y * 0.55;

            ui.vertical(|ui| {
                // Main Viewport (top)
                ui.allocate_ui(egui::vec2(available.x, top_height), |ui| {
                    egui::Frame::canvas(ui.style()).show(ui, |ui| {
                        viewport::show(ui, &mut self.state);
                    });
                });

                ui.add_space(5.0);

                // Bottom section: Algorithm viz (left) and Network (right)
                ui.horizontal(|ui| {
                    let bottom_width = available.x;

                    // Algorithm Visualization (left half)
                    ui.allocate_ui(
                        egui::vec2(bottom_width * 0.6, available.y - top_height - 10.0),
                        |ui| {
                            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                                algorithm::show(ui, &mut self.state);
                            });
                        },
                    );

                    // Network Topology (right half)
                    ui.allocate_ui(
                        egui::vec2(bottom_width * 0.4 - 10.0, available.y - top_height - 10.0),
                        |ui| {
                            egui::Frame::canvas(ui.style()).show(ui, |ui| {
                                network::show(ui, &self.state);
                            });
                        },
                    );
                });
            });
        });

        // Handle keyboard shortcuts
        ctx.input(|i| {
            if i.key_pressed(egui::Key::Space) {
                self.state.is_running = !self.state.is_running;
            }
            if i.key_pressed(egui::Key::R) {
                self.state.reset();
            }
            if i.key_pressed(egui::Key::D) {
                if self.state.is_demo_active() {
                    self.state.stop_demo();
                } else {
                    self.state.start_demo();
                }
            }
            if i.key_pressed(egui::Key::Escape) {
                self.state.selected_drone = None;
            }
        });

        // About window
        if self.show_about {
            egui::Window::new("About")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.heading("Drone Swarm Visualization");
                    ui.label("Version 0.1.0");
                    ui.add_space(10.0);
                    ui.label("Real-time visualization for drone swarm algorithms");
                    ui.label("including PSO, ACO, and GWO optimization.");
                    ui.add_space(10.0);
                    ui.hyperlink_to(
                        "GitHub Repository",
                        "https://github.com/mahii6991/drone-swarm-system",
                    );
                    ui.add_space(10.0);
                    if ui.button("Close").clicked() {
                        self.show_about = false;
                    }
                });
        }
    }
}
