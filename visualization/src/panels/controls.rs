//! Parameter control panel

use crate::state::{AlgorithmType, FormationType, SimulationState};
use egui::Ui;

pub fn show(ui: &mut Ui, state: &mut SimulationState) {
    ui.heading("Controls");
    ui.add_space(5.0);

    // Simulation Controls - Always visible at top
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.label("Simulation");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if state.is_running {
                    ui.label(egui::RichText::new("● Running").color(egui::Color32::GREEN));
                } else {
                    ui.label(egui::RichText::new("● Paused").color(egui::Color32::GRAY));
                }
            });
        });
        ui.add_space(3.0);

        ui.horizontal(|ui| {
            // Large play/pause button
            let play_text = if state.is_running {
                "⏸ Pause"
            } else {
                "▶ Play"
            };
            let play_color = if state.is_running {
                egui::Color32::from_rgb(200, 150, 50)
            } else {
                egui::Color32::from_rgb(50, 200, 100)
            };
            if ui
                .add_sized(
                    [80.0, 28.0],
                    egui::Button::new(egui::RichText::new(play_text).color(play_color)),
                )
                .clicked()
            {
                state.is_running = !state.is_running;
            }
            if ui
                .add_sized([55.0, 28.0], egui::Button::new("Reset"))
                .clicked()
            {
                state.reset();
            }
            if ui
                .add_sized([50.0, 28.0], egui::Button::new("Step"))
                .clicked()
            {
                state.step();
            }
        });

        ui.add_space(5.0);
        ui.add(egui::Slider::new(&mut state.simulation_speed, 0.1..=5.0).text("Speed"));
        ui.label(format!("Time Step: {}", state.time_step));
    });

    ui.add_space(10.0);

    // Formation Controls - Default open
    egui::CollapsingHeader::new("Formation")
        .default_open(true)
        .show(ui, |ui| {
            let mut formation_changed = false;

            egui::ComboBox::from_label("Type")
                .selected_text(format!("{:?}", state.formation))
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_value(&mut state.formation, FormationType::Circle, "Circle")
                        .clicked()
                    {
                        formation_changed = true;
                    }
                    if ui
                        .selectable_value(&mut state.formation, FormationType::Grid, "Grid")
                        .clicked()
                    {
                        formation_changed = true;
                    }
                    if ui
                        .selectable_value(&mut state.formation, FormationType::Line, "Line")
                        .clicked()
                    {
                        formation_changed = true;
                    }
                    if ui
                        .selectable_value(
                            &mut state.formation,
                            FormationType::VFormation,
                            "V-Formation",
                        )
                        .clicked()
                    {
                        formation_changed = true;
                    }
                    if ui
                        .selectable_value(&mut state.formation, FormationType::Random, "Random")
                        .clicked()
                    {
                        formation_changed = true;
                    }
                });

            ui.add_space(5.0);

            let mut drone_count_changed = false;
            let mut drone_count = state.formation_params.drone_count as i32;
            if ui
                .add(egui::Slider::new(&mut drone_count, 1..=100).text("Drones"))
                .changed()
            {
                state.formation_params.drone_count = drone_count as usize;
                drone_count_changed = true;
            }

            match state.formation {
                FormationType::Circle => {
                    let mut radius = state.formation_params.circle_radius as i32;
                    if ui
                        .add(egui::Slider::new(&mut radius, 20..=200).text("Radius"))
                        .changed()
                    {
                        state.formation_params.circle_radius = radius as u32;
                    }
                }
                FormationType::Grid => {
                    let mut spacing = state.formation_params.grid_spacing as i32;
                    if ui
                        .add(egui::Slider::new(&mut spacing, 10..=100).text("Spacing"))
                        .changed()
                    {
                        state.formation_params.grid_spacing = spacing as u32;
                    }
                }
                FormationType::Line => {
                    let mut spacing = state.formation_params.line_spacing as i32;
                    if ui
                        .add(egui::Slider::new(&mut spacing, 10..=100).text("Spacing"))
                        .changed()
                    {
                        state.formation_params.line_spacing = spacing as u32;
                    }
                }
                FormationType::VFormation => {
                    let mut spacing = state.formation_params.v_spacing as i32;
                    if ui
                        .add(egui::Slider::new(&mut spacing, 10..=100).text("Spacing"))
                        .changed()
                    {
                        state.formation_params.v_spacing = spacing as u32;
                    }
                }
                FormationType::Random => {}
            }

            if ui.button("Apply Formation").clicked() || formation_changed || drone_count_changed {
                state.spawn_drones(state.formation_params.drone_count);
            }
        });

    ui.add_space(10.0);

    // Algorithm Selection - Default open
    egui::CollapsingHeader::new("Algorithm")
        .default_open(true)
        .show(ui, |ui| {
            egui::ComboBox::from_label("Active")
                .selected_text(format!("{:?}", state.active_algorithm))
                .show_ui(ui, |ui| {
                    ui.label("Swarm Intelligence");
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::PSO,
                        "PSO - Particle Swarm",
                    );
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::ACO,
                        "ACO - Ant Colony",
                    );
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::GWO,
                        "GWO - Grey Wolf",
                    );
                    ui.separator();
                    ui.label("Distributed Systems");
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::Federated,
                        "FL - Federated Learning",
                    );
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::Consensus,
                        "Raft - SwarmRaft Consensus",
                    );
                    ui.separator();
                    ui.label("Evolutionary");
                    ui.selectable_value(
                        &mut state.active_algorithm,
                        AlgorithmType::DE,
                        "DE - Differential Evolution",
                    );
                });

            ui.add_space(5.0);

            // Show current algorithm stats inline
            match state.active_algorithm {
                AlgorithmType::PSO => {
                    if let Some(ref pso) = state.pso_state {
                        ui.label(format!(
                            "Iteration: {} | Best: {:.4}",
                            pso.iteration, pso.gbest_cost
                        ));
                    }
                }
                AlgorithmType::ACO => {
                    if let Some(ref aco) = state.aco_state {
                        ui.label(format!(
                            "Iteration: {} | Path: {} nodes",
                            aco.iteration,
                            aco.best_path.len()
                        ));
                    }
                }
                AlgorithmType::GWO => {
                    if let Some(ref gwo) = state.gwo_state {
                        let alpha_fit = gwo.alpha.as_ref().map(|a| a.fitness).unwrap_or(0.0);
                        ui.label(format!(
                            "Iteration: {} | Alpha: {:.4}",
                            gwo.iteration, alpha_fit
                        ));
                    }
                }
                AlgorithmType::Federated => {
                    if let Some(ref fed) = state.federated_state {
                        ui.label(format!(
                            "Round: {} | Accuracy: {:.1}%",
                            fed.round,
                            fed.current_accuracy * 100.0
                        ));
                    }
                }
                AlgorithmType::Consensus => {
                    if let Some(ref cons) = state.consensus_state {
                        let leader = cons
                            .leader_id
                            .map(|l| format!("N{}", l))
                            .unwrap_or_else(|| "None".to_string());
                        ui.label(format!("Term: {} | Leader: {}", cons.term, leader));
                    }
                }
                AlgorithmType::DE => {
                    if let Some(ref de) = state.de_state {
                        ui.label(format!(
                            "Iteration: {} | Best: {:.4}",
                            de.iteration, de.best_fitness
                        ));
                    }
                }
            }
        });

    ui.add_space(10.0);

    // PSO Parameters
    if let Some(ref mut pso) = state.pso_state {
        egui::CollapsingHeader::new("PSO Parameters")
            .default_open(false)
            .show(ui, |ui| {
                ui.add(egui::Slider::new(&mut pso.cognitive, 0.0..=4.0).text("Cognitive (c1)"));
                ui.add(egui::Slider::new(&mut pso.social, 0.0..=4.0).text("Social (c2)"));
                ui.add(egui::Slider::new(&mut pso.inertia, 0.0..=1.0).text("Inertia (w)"));

                ui.add_space(5.0);
                ui.label(format!("Particles: {}", pso.particles.len()));
                ui.label(format!("Best Cost: {:.6}", pso.gbest_cost));
            });
    }

    // ACO Parameters
    if let Some(ref mut aco) = state.aco_state {
        egui::CollapsingHeader::new("ACO Parameters")
            .default_open(false)
            .show(ui, |ui| {
                ui.add(
                    egui::Slider::new(&mut aco.evaporation_rate, 0.01..=0.5).text("Evaporation"),
                );
                ui.add(egui::Slider::new(&mut aco.alpha, 0.1..=5.0).text("Alpha (α)"));
                ui.add(egui::Slider::new(&mut aco.beta, 0.1..=5.0).text("Beta (β)"));

                ui.add_space(5.0);
                ui.label(format!("Ants: {}", aco.ants.len()));
            });
    }

    // GWO Parameters
    if let Some(ref gwo) = state.gwo_state {
        egui::CollapsingHeader::new("GWO Parameters")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(format!("Wolves: {}", gwo.wolves.len()));
                ui.label(format!("Convergence (a): {:.3}", gwo.convergence_param));

                if let Some(ref alpha) = gwo.alpha {
                    ui.label(format!("Alpha Fitness: {:.4}", alpha.fitness));
                }
                if let Some(ref beta) = gwo.beta {
                    ui.label(format!("Beta Fitness: {:.4}", beta.fitness));
                }
                if let Some(ref delta) = gwo.delta {
                    ui.label(format!("Delta Fitness: {:.4}", delta.fitness));
                }
            });
    }

    // Federated Learning Parameters
    if let Some(ref fed) = state.federated_state {
        egui::CollapsingHeader::new("Federated Learning")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(format!("Nodes: {}", fed.nodes.len()));
                ui.label(format!("Round: {}", fed.round));
                ui.label(format!("Accuracy: {:.1}%", fed.current_accuracy * 100.0));
                let aggregator = fed
                    .aggregator
                    .map(|a| format!("N{}", a))
                    .unwrap_or_else(|| "None".to_string());
                ui.label(format!("Aggregator: {}", aggregator));
                ui.add_space(3.0);
                ui.label("Global Model Weights:");
                ui.horizontal(|ui| {
                    for (i, w) in fed.global_model.iter().enumerate() {
                        ui.label(format!("w{}: {:.2}", i, w));
                    }
                });
            });
    }

    // Consensus Parameters
    if let Some(ref cons) = state.consensus_state {
        egui::CollapsingHeader::new("SwarmRaft Consensus")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(format!("Nodes: {}", cons.nodes.len()));
                ui.label(format!("Term: {}", cons.term));
                let leader = cons
                    .leader_id
                    .map(|l| format!("N{}", l))
                    .unwrap_or_else(|| "None".to_string());
                ui.label(format!("Leader: {}", leader));
                ui.label(format!("Log Entries: {}", cons.log_entries.len()));
                ui.label(format!("Committed: {}", cons.committed_index));
                ui.label(format!("Messages in flight: {}", cons.messages.len()));
            });
    }

    // DE Parameters
    if let Some(ref mut de) = state.de_state {
        egui::CollapsingHeader::new("Differential Evolution")
            .default_open(false)
            .show(ui, |ui| {
                ui.add(egui::Slider::new(&mut de.mutation_factor, 0.1..=2.0).text("Mutation (F)"));
                ui.add(egui::Slider::new(&mut de.crossover_rate, 0.0..=1.0).text("Crossover (CR)"));
                ui.add_space(5.0);
                ui.label(format!("Population: {}", de.population.len()));
                ui.label(format!("Iteration: {}", de.iteration));
                ui.label(format!("Best Fitness: {:.6}", de.best_fitness));
            });
    }

    ui.add_space(10.0);

    // Viewport Options
    egui::CollapsingHeader::new("Viewport")
        .default_open(false)
        .show(ui, |ui| {
            ui.checkbox(&mut state.viewport.show_grid, "Show Grid");
            ui.checkbox(&mut state.viewport.show_trails, "Show Trails");
            ui.checkbox(&mut state.viewport.show_velocities, "Show Velocities");

            ui.add_space(5.0);
            ui.add(egui::Slider::new(&mut state.viewport.zoom, 0.5..=10.0).text("Zoom"));

            if ui.button("Reset View").clicked() {
                state.viewport.center = egui::Pos2::ZERO;
                state.viewport.zoom = 2.0;
            }
        });

    ui.add_space(10.0);

    // Demo Mode
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.label("Demo Mode");
            if state.is_demo_active() {
                if let Some(ref demo) = state.demo_mode {
                    ui.label(
                        egui::RichText::new(demo.scenario_name())
                            .color(egui::Color32::from_rgb(255, 200, 100)),
                    );
                }
            }
        });
        ui.horizontal(|ui| {
            if state.is_demo_active() {
                if ui.button("Stop Demo").clicked() {
                    state.stop_demo();
                }
            } else {
                if ui.button("Start Demo").clicked() {
                    state.start_demo();
                }
            }
            ui.label("(Press D)");
        });
    });
}
