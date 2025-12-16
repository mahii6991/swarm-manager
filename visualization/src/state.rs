//! Simulation state management

use egui::{Pos2, Vec2};

// Helper enum for demo actions (internal use)
enum DemoAction {
    ChangeFormation(usize),
    StartPSO,
    StartACO,
    StartGWO,
    StartScaleTest,
    IncreaseDrones,
    RestartDemo,
}

/// Main simulation state containing all visualizable data
pub struct SimulationState {
    // Drone data
    pub drones: Vec<DroneVisual>,
    pub formation: FormationType,
    pub formation_params: FormationParams,

    // Algorithm states
    pub pso_state: Option<PSOVisualState>,
    pub aco_state: Option<ACOVisualState>,
    pub gwo_state: Option<GWOVisualState>,
    pub federated_state: Option<FederatedVisualState>,
    pub consensus_state: Option<ConsensusVisualState>,
    pub de_state: Option<DEVisualState>,
    pub active_algorithm: AlgorithmType,

    // Network state
    pub network: NetworkTopology,

    // Simulation control
    pub is_running: bool,
    pub simulation_speed: f32,
    pub time_step: u64,

    // Viewport state
    pub viewport: ViewportState,

    // Selection state
    pub selected_drone: Option<u64>,

    // Demo mode
    pub demo_mode: Option<DemoMode>,

    // Safety & Mission states
    pub safety_state: SafetyState,
    pub mission_state: MissionState,
    pub telemetry_state: TelemetryState,
    pub failsafe_state: FailsafeState,
    pub alerts: Vec<Alert>,
}

impl SimulationState {
    pub fn new() -> Self {
        let mut state = Self {
            drones: Vec::new(),
            formation: FormationType::Circle,
            formation_params: FormationParams::default(),
            pso_state: Some(PSOVisualState::new(30)),
            aco_state: Some(ACOVisualState::new()),
            gwo_state: Some(GWOVisualState::new(20)),
            federated_state: Some(FederatedVisualState::new(8)),
            consensus_state: Some(ConsensusVisualState::new(5)),
            de_state: Some(DEVisualState::new(25)),
            active_algorithm: AlgorithmType::PSO,
            network: NetworkTopology::new(),
            is_running: false,
            simulation_speed: 1.0,
            time_step: 0,
            viewport: ViewportState::default(),
            selected_drone: None,
            demo_mode: None,
            safety_state: SafetyState::default(),
            mission_state: MissionState::default(),
            telemetry_state: TelemetryState::default(),
            failsafe_state: FailsafeState::default(),
            alerts: Vec::new(),
        };

        // Initialize drones
        state.spawn_drones(10);
        state
    }

    /// Start demo mode with automated scenarios
    pub fn start_demo(&mut self) {
        self.demo_mode = Some(DemoMode::new());
        self.is_running = true;
    }

    /// Stop demo mode
    pub fn stop_demo(&mut self) {
        self.demo_mode = None;
    }

    /// Check if demo mode is active
    pub fn is_demo_active(&self) -> bool {
        self.demo_mode.is_some()
    }

    /// Update demo mode
    fn update_demo(&mut self) {
        // Extract demo state to avoid borrow issues
        let demo_action = if let Some(ref mut demo) = self.demo_mode {
            demo.step += 1;

            // Determine what action to take based on scenario
            match demo.current_scenario {
                DemoScenario::FormationShowcase => {
                    // Cycle through formations every 200 steps
                    if demo.step % 200 == 0 {
                        demo.formation_index = (demo.formation_index + 1) % 5;
                        Some(DemoAction::ChangeFormation(demo.formation_index))
                    } else if demo.step > 1000 {
                        demo.step = 0;
                        demo.current_scenario = DemoScenario::PSOConvergence;
                        Some(DemoAction::StartPSO)
                    } else {
                        None
                    }
                }
                DemoScenario::PSOConvergence => {
                    if demo.step > 600 {
                        demo.step = 0;
                        demo.current_scenario = DemoScenario::ACOPathfinding;
                        Some(DemoAction::StartACO)
                    } else {
                        None
                    }
                }
                DemoScenario::ACOPathfinding => {
                    if demo.step > 600 {
                        demo.step = 0;
                        demo.current_scenario = DemoScenario::GWOHunting;
                        Some(DemoAction::StartGWO)
                    } else {
                        None
                    }
                }
                DemoScenario::GWOHunting => {
                    if demo.step > 600 {
                        demo.step = 0;
                        demo.current_scenario = DemoScenario::ScaleTest;
                        Some(DemoAction::StartScaleTest)
                    } else {
                        None
                    }
                }
                DemoScenario::ScaleTest => {
                    if demo.step % 100 == 0 && self.formation_params.drone_count < 100 {
                        Some(DemoAction::IncreaseDrones)
                    } else if demo.step > 500 {
                        demo.step = 0;
                        demo.current_scenario = DemoScenario::FormationShowcase;
                        demo.formation_index = 0;
                        Some(DemoAction::RestartDemo)
                    } else {
                        None
                    }
                }
            }
        } else {
            None
        };

        // Execute the action outside the borrow
        if let Some(action) = demo_action {
            match action {
                DemoAction::ChangeFormation(index) => {
                    self.formation = match index {
                        0 => FormationType::Circle,
                        1 => FormationType::Grid,
                        2 => FormationType::VFormation,
                        3 => FormationType::Line,
                        _ => FormationType::Random,
                    };
                    let count = self.formation_params.drone_count;
                    self.spawn_drones(count);
                }
                DemoAction::StartPSO => {
                    self.active_algorithm = AlgorithmType::PSO;
                    self.pso_state = Some(PSOVisualState::new(40));
                }
                DemoAction::StartACO => {
                    self.active_algorithm = AlgorithmType::ACO;
                    self.aco_state = Some(ACOVisualState::new());
                }
                DemoAction::StartGWO => {
                    self.active_algorithm = AlgorithmType::GWO;
                    self.gwo_state = Some(GWOVisualState::new(25));
                }
                DemoAction::StartScaleTest => {
                    self.formation_params.drone_count = 50;
                    self.spawn_drones(50);
                }
                DemoAction::IncreaseDrones => {
                    self.formation_params.drone_count += 10;
                    let count = self.formation_params.drone_count;
                    self.spawn_drones(count);
                }
                DemoAction::RestartDemo => {
                    self.formation_params.drone_count = 15;
                    self.spawn_drones(15);
                }
            }
        }
    }

    pub fn reset(&mut self) {
        self.time_step = 0;
        self.spawn_drones(self.formation_params.drone_count);
        if let Some(ref mut pso) = self.pso_state {
            *pso = PSOVisualState::new(pso.particles.len());
        }
        if let Some(ref mut aco) = self.aco_state {
            *aco = ACOVisualState::new();
        }
        if let Some(ref mut gwo) = self.gwo_state {
            *gwo = GWOVisualState::new(gwo.wolves.len());
        }
    }

    pub fn spawn_drones(&mut self, count: usize) {
        self.drones.clear();
        self.network = NetworkTopology::new();

        let radius = self.formation_params.circle_radius as f32;

        for i in 0..count {
            let angle = (i as f32 / count as f32) * std::f32::consts::TAU;
            let pos = match self.formation {
                FormationType::Circle => Pos2::new(radius * angle.cos(), radius * angle.sin()),
                FormationType::Grid => {
                    let cols = (count as f32).sqrt().ceil() as usize;
                    let row = i / cols;
                    let col = i % cols;
                    let spacing = self.formation_params.grid_spacing as f32;
                    Pos2::new(
                        col as f32 * spacing - (cols as f32 * spacing / 2.0),
                        row as f32 * spacing - ((count / cols) as f32 * spacing / 2.0),
                    )
                }
                FormationType::Line => {
                    let spacing = self.formation_params.line_spacing as f32;
                    Pos2::new(i as f32 * spacing - (count as f32 * spacing / 2.0), 0.0)
                }
                FormationType::VFormation => {
                    let spacing = self.formation_params.v_spacing as f32;
                    let half = count / 2;
                    if i <= half {
                        Pos2::new(-(i as f32) * spacing, -(i as f32) * spacing)
                    } else {
                        let j = i - half;
                        Pos2::new(j as f32 * spacing, -(j as f32) * spacing)
                    }
                }
                FormationType::Random => Pos2::new(
                    (rand::random::<f32>() - 0.5) * 200.0,
                    (rand::random::<f32>() - 0.5) * 200.0,
                ),
            };

            let drone = DroneVisual {
                id: i as u64,
                position: pos,
                target_position: pos,
                altitude: 10.0 + rand::random::<f32>() * 5.0,
                velocity: Vec2::ZERO,
                battery: 80 + (rand::random::<f32>() * 20.0) as u8,
                status: DroneStatus::Active,
                trail: Vec::new(),
            };

            self.drones.push(drone);

            // Add network node
            self.network.nodes.push(NetworkNode {
                id: i as u64,
                position: pos,
                neighbor_count: 0,
            });
        }

        // Create network edges (connect nearby drones)
        self.update_network_topology();
    }

    fn update_network_topology(&mut self) {
        self.network.edges.clear();

        let comm_range = 80.0; // Communication range

        // Update node positions first
        for (i, drone) in self.drones.iter().enumerate() {
            if let Some(node) = self.network.nodes.get_mut(i) {
                node.position = drone.position;
            }
        }

        for i in 0..self.drones.len() {
            let mut neighbor_count = 0;
            for j in (i + 1)..self.drones.len() {
                let dist = self.drones[i].position.distance(self.drones[j].position);
                if dist < comm_range {
                    let link_quality = 1.0 - (dist / comm_range);
                    self.network.edges.push(NetworkEdge {
                        from: i as u64,
                        to: j as u64,
                        link_quality,
                        rtt_ms: (dist * 0.5) as u32,
                    });
                    neighbor_count += 1;
                }
            }
            if let Some(node) = self.network.nodes.get_mut(i) {
                node.neighbor_count = neighbor_count;
            }
        }
    }

    pub fn step(&mut self) {
        self.time_step += 1;

        // Update demo mode if active
        self.update_demo();

        // Update drone positions (simple formation seeking)
        let formation_center = self.calculate_formation_center();

        // Pre-calculate all target positions to avoid borrow issues
        let targets: Vec<Pos2> = (0..self.drones.len())
            .map(|i| self.calculate_target_position_static(i, formation_center, self.drones.len()))
            .collect();

        let show_trails = self.viewport.show_trails;
        let simulation_speed = self.simulation_speed;
        let time_step = self.time_step;

        for (i, drone) in self.drones.iter_mut().enumerate() {
            // Save trail
            if show_trails {
                drone.trail.push(drone.position);
                if drone.trail.len() > 50 {
                    drone.trail.remove(0);
                }
            }

            // Get pre-calculated target position
            let target = targets[i];
            drone.target_position = target;

            // Move towards target
            let direction = target - drone.position;
            let distance = direction.length();

            if distance > 1.0 {
                let speed = (simulation_speed * 2.0).min(distance);
                drone.velocity = direction.normalized() * speed;
                drone.position += drone.velocity * 0.1;
            } else {
                drone.velocity = Vec2::ZERO;
            }

            // Drain battery slowly
            if time_step % 100 == 0 && drone.battery > 0 {
                drone.battery = drone.battery.saturating_sub(1);
            }

            // Update status based on battery (check lowest threshold first)
            drone.status = if drone.battery < 10 {
                DroneStatus::Emergency
            } else if drone.battery < 20 {
                DroneStatus::Returning
            } else {
                DroneStatus::Active
            };
        }

        // Update network topology
        self.update_network_topology();

        // Update algorithm states
        self.step_algorithms();
    }

    fn calculate_formation_center(&self) -> Pos2 {
        if self.drones.is_empty() {
            return Pos2::ZERO;
        }
        let sum: Pos2 = self
            .drones
            .iter()
            .map(|d| d.position)
            .fold(Pos2::ZERO, |a, b| a + b.to_vec2());
        Pos2::new(
            sum.x / self.drones.len() as f32,
            sum.y / self.drones.len() as f32,
        )
    }

    fn calculate_target_position_static(&self, index: usize, center: Pos2, count: usize) -> Pos2 {
        if count == 0 {
            return center;
        }

        match self.formation {
            FormationType::Circle => {
                let radius = self.formation_params.circle_radius as f32;
                let angle = (index as f32 / count as f32) * std::f32::consts::TAU;
                center + Vec2::new(radius * angle.cos(), radius * angle.sin())
            }
            FormationType::Grid => {
                let cols = (count as f32).sqrt().ceil() as usize;
                let row = index / cols;
                let col = index % cols;
                let spacing = self.formation_params.grid_spacing as f32;
                Pos2::new(
                    col as f32 * spacing - (cols as f32 * spacing / 2.0),
                    row as f32 * spacing - ((count / cols) as f32 * spacing / 2.0),
                )
            }
            FormationType::Line => {
                let spacing = self.formation_params.line_spacing as f32;
                Pos2::new(index as f32 * spacing - (count as f32 * spacing / 2.0), 0.0)
            }
            FormationType::VFormation => {
                let spacing = self.formation_params.v_spacing as f32;
                let half = count / 2;
                if index <= half {
                    Pos2::new(-(index as f32) * spacing, -(index as f32) * spacing)
                } else {
                    let j = index - half;
                    Pos2::new(j as f32 * spacing, -(j as f32) * spacing)
                }
            }
            FormationType::Random => center, // Keep current position for random
        }
    }

    fn step_algorithms(&mut self) {
        // Step PSO
        if let Some(ref mut pso) = self.pso_state {
            pso.step();
        }

        // Step ACO
        if let Some(ref mut aco) = self.aco_state {
            aco.step();
        }

        // Step GWO
        if let Some(ref mut gwo) = self.gwo_state {
            gwo.step();
        }

        // Step Federated Learning
        if let Some(ref mut fed) = self.federated_state {
            fed.step();
        }

        // Step Consensus
        if let Some(ref mut cons) = self.consensus_state {
            cons.step();
        }

        // Step DE
        if let Some(ref mut de) = self.de_state {
            de.step();
        }
    }
}

// ============ Drone Types ============

#[derive(Clone, Debug)]
pub struct DroneVisual {
    pub id: u64,
    pub position: Pos2,
    pub target_position: Pos2,
    pub altitude: f32,
    pub velocity: Vec2,
    pub battery: u8,
    pub status: DroneStatus,
    pub trail: Vec<Pos2>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DroneStatus {
    Idle,
    Active,
    Returning,
    Emergency,
    Failed,
}

// ============ Formation Types ============

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FormationType {
    Random,
    Grid,
    Line,
    Circle,
    VFormation,
}

#[derive(Clone, Debug)]
pub struct FormationParams {
    pub drone_count: usize,
    pub grid_spacing: u32,
    pub circle_radius: u32,
    pub line_spacing: u32,
    pub v_spacing: u32,
}

impl Default for FormationParams {
    fn default() -> Self {
        Self {
            drone_count: 10,
            grid_spacing: 30,
            circle_radius: 60,
            line_spacing: 25,
            v_spacing: 20,
        }
    }
}

// ============ Algorithm Types ============

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AlgorithmType {
    PSO,
    ACO,
    GWO,
    Federated,
    Consensus,
    DE,
}

// ============ PSO State ============

#[derive(Clone, Debug)]
pub struct PSOVisualState {
    pub particles: Vec<ParticleVisual>,
    pub gbest_position: Pos2,
    pub gbest_cost: f32,
    pub cost_history: Vec<f32>,
    pub iteration: u32,
    // Parameters
    pub cognitive: f32,
    pub social: f32,
    pub inertia: f32,
}

#[derive(Clone, Debug)]
pub struct ParticleVisual {
    pub position: Pos2,
    pub velocity: Vec2,
    pub pbest_position: Pos2,
    pub pbest_cost: f32,
}

impl PSOVisualState {
    pub fn new(particle_count: usize) -> Self {
        let mut particles = Vec::new();
        let bounds = 100.0;

        for _ in 0..particle_count {
            let pos = Pos2::new(
                (rand::random::<f32>() - 0.5) * bounds * 2.0,
                (rand::random::<f32>() - 0.5) * bounds * 2.0,
            );
            particles.push(ParticleVisual {
                position: pos,
                velocity: Vec2::new(
                    (rand::random::<f32>() - 0.5) * 10.0,
                    (rand::random::<f32>() - 0.5) * 10.0,
                ),
                pbest_position: pos,
                pbest_cost: f32::MAX,
            });
        }

        Self {
            particles,
            gbest_position: Pos2::ZERO,
            gbest_cost: f32::MAX,
            cost_history: Vec::new(),
            iteration: 0,
            cognitive: 2.0,
            social: 2.0,
            inertia: 0.7,
        }
    }

    pub fn step(&mut self) {
        self.iteration += 1;

        // Simple sphere function optimization (minimize distance to origin)
        for particle in &mut self.particles {
            // Evaluate fitness (sphere function)
            let cost = particle.position.x.powi(2) + particle.position.y.powi(2);

            // Update personal best
            if cost < particle.pbest_cost {
                particle.pbest_cost = cost;
                particle.pbest_position = particle.position;
            }

            // Update global best
            if cost < self.gbest_cost {
                self.gbest_cost = cost;
                self.gbest_position = particle.position;
            }
        }

        // Update velocities and positions
        for particle in &mut self.particles {
            let r1 = rand::random::<f32>();
            let r2 = rand::random::<f32>();

            // Velocity update
            particle.velocity = particle.velocity * self.inertia
                + (particle.pbest_position - particle.position) * self.cognitive * r1
                + (self.gbest_position - particle.position) * self.social * r2;

            // Clamp velocity
            let max_vel = 5.0;
            if particle.velocity.length() > max_vel {
                particle.velocity = particle.velocity.normalized() * max_vel;
            }

            // Position update
            particle.position += particle.velocity;

            // Boundary handling (bounce)
            let bounds = 100.0;
            if particle.position.x.abs() > bounds {
                particle.position.x = particle.position.x.signum() * bounds;
                particle.velocity.x *= -0.5;
            }
            if particle.position.y.abs() > bounds {
                particle.position.y = particle.position.y.signum() * bounds;
                particle.velocity.y *= -0.5;
            }
        }

        // Record cost history
        self.cost_history.push(self.gbest_cost);
        if self.cost_history.len() > 200 {
            self.cost_history.remove(0);
        }
    }
}

// ============ ACO State ============

#[derive(Clone, Debug)]
pub struct ACOVisualState {
    pub ants: Vec<AntVisual>,
    pub pheromones: Vec<PheromoneTrail>,
    pub best_path: Vec<Pos2>,
    pub obstacles: Vec<ObstacleVisual>,
    pub start: Pos2,
    pub goal: Pos2,
    pub iteration: usize,
    // Parameters
    pub evaporation_rate: f32,
    pub alpha: f32,
    pub beta: f32,
}

#[derive(Clone, Debug)]
pub struct AntVisual {
    pub position: Pos2,
    pub path: Vec<Pos2>,
}

#[derive(Clone, Debug)]
pub struct PheromoneTrail {
    pub from: Pos2,
    pub to: Pos2,
    pub strength: f32,
}

#[derive(Clone, Debug)]
pub struct ObstacleVisual {
    pub center: Pos2,
    pub radius: f32,
}

impl ACOVisualState {
    pub fn new() -> Self {
        let start = Pos2::new(-80.0, -60.0);
        let goal = Pos2::new(80.0, 60.0);

        let obstacles = vec![
            ObstacleVisual {
                center: Pos2::new(0.0, 0.0),
                radius: 25.0,
            },
            ObstacleVisual {
                center: Pos2::new(-40.0, 30.0),
                radius: 15.0,
            },
            ObstacleVisual {
                center: Pos2::new(40.0, -20.0),
                radius: 20.0,
            },
        ];

        let mut ants = Vec::new();
        for _ in 0..20 {
            ants.push(AntVisual {
                position: start,
                path: vec![start],
            });
        }

        Self {
            ants,
            pheromones: Vec::new(),
            best_path: vec![start, goal],
            obstacles,
            start,
            goal,
            iteration: 0,
            evaporation_rate: 0.1,
            alpha: 1.0,
            beta: 2.0,
        }
    }

    pub fn step(&mut self) {
        self.iteration += 1;

        // Move ants towards goal
        for ant in &mut self.ants {
            let direction = self.goal - ant.position;
            let dist = direction.length();

            if dist > 5.0 {
                // Add some randomness
                let noise = Vec2::new(
                    (rand::random::<f32>() - 0.5) * 20.0,
                    (rand::random::<f32>() - 0.5) * 20.0,
                );
                let move_dir = (direction.normalized() * 3.0 + noise * 0.3).normalized();

                let new_pos = ant.position + move_dir * 5.0;

                // Check obstacle collision
                let collides = self
                    .obstacles
                    .iter()
                    .any(|obs| (new_pos - obs.center).length() < obs.radius + 5.0);

                if !collides {
                    // Add pheromone trail
                    self.pheromones.push(PheromoneTrail {
                        from: ant.position,
                        to: new_pos,
                        strength: 1.0,
                    });
                    ant.position = new_pos;
                    ant.path.push(new_pos);
                }
            } else {
                // Reached goal, check if best path
                if ant.path.len() < self.best_path.len() || self.best_path.len() <= 2 {
                    self.best_path = ant.path.clone();
                }
                // Reset ant
                ant.position = self.start;
                ant.path = vec![self.start];
            }
        }

        // Evaporate pheromones
        self.pheromones.retain_mut(|p| {
            p.strength *= 1.0 - self.evaporation_rate;
            p.strength > 0.05
        });

        // Limit pheromone count
        if self.pheromones.len() > 500 {
            self.pheromones.drain(0..100);
        }
    }
}

// ============ GWO State ============

#[derive(Clone, Debug)]
pub struct GWOVisualState {
    pub wolves: Vec<WolfVisual>,
    pub alpha: Option<WolfVisual>,
    pub beta: Option<WolfVisual>,
    pub delta: Option<WolfVisual>,
    pub convergence_param: f32,
    pub fitness_history: Vec<f32>,
    pub iteration: usize,
}

#[derive(Clone, Debug)]
pub struct WolfVisual {
    pub position: Pos2,
    pub fitness: f32,
    pub rank: WolfRank,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WolfRank {
    Alpha,
    Beta,
    Delta,
    Omega,
}

impl GWOVisualState {
    pub fn new(wolf_count: usize) -> Self {
        let bounds = 100.0;
        let mut wolves = Vec::new();

        for _ in 0..wolf_count {
            wolves.push(WolfVisual {
                position: Pos2::new(
                    (rand::random::<f32>() - 0.5) * bounds * 2.0,
                    (rand::random::<f32>() - 0.5) * bounds * 2.0,
                ),
                fitness: f32::MAX,
                rank: WolfRank::Omega,
            });
        }

        Self {
            wolves,
            alpha: None,
            beta: None,
            delta: None,
            convergence_param: 2.0,
            fitness_history: Vec::new(),
            iteration: 0,
        }
    }

    pub fn step(&mut self) {
        self.iteration += 1;

        // Update convergence parameter (decreases from 2 to 0)
        let max_iter = 500.0;
        self.convergence_param = 2.0 * (1.0 - self.iteration as f32 / max_iter);

        // Evaluate fitness (minimize rastrigin-like function)
        for wolf in &mut self.wolves {
            let x = wolf.position.x / 20.0;
            let y = wolf.position.y / 20.0;
            wolf.fitness = x.powi(2) + y.powi(2)
                - 10.0 * (2.0 * std::f32::consts::PI * x).cos()
                - 10.0 * (2.0 * std::f32::consts::PI * y).cos()
                + 20.0;
        }

        // Sort by fitness and assign ranks
        self.wolves
            .sort_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap());

        for (i, wolf) in self.wolves.iter_mut().enumerate() {
            wolf.rank = match i {
                0 => WolfRank::Alpha,
                1 => WolfRank::Beta,
                2 => WolfRank::Delta,
                _ => WolfRank::Omega,
            };
        }

        // Get leaders
        self.alpha = self.wolves.get(0).cloned();
        self.beta = self.wolves.get(1).cloned();
        self.delta = self.wolves.get(2).cloned();

        // Record fitness
        if let Some(ref alpha) = self.alpha {
            self.fitness_history.push(alpha.fitness);
            if self.fitness_history.len() > 200 {
                self.fitness_history.remove(0);
            }
        }

        // Update positions (skip leaders)
        let alpha_pos = self
            .alpha
            .as_ref()
            .map(|w| w.position)
            .unwrap_or(Pos2::ZERO);
        let beta_pos = self.beta.as_ref().map(|w| w.position).unwrap_or(Pos2::ZERO);
        let delta_pos = self
            .delta
            .as_ref()
            .map(|w| w.position)
            .unwrap_or(Pos2::ZERO);

        for i in 3..self.wolves.len() {
            let a = self.convergence_param;

            // Random coefficients
            let r1 = rand::random::<f32>();
            let r2 = rand::random::<f32>();
            let a1 = 2.0 * a * r1 - a;
            let c1 = 2.0 * r2;

            let r1 = rand::random::<f32>();
            let r2 = rand::random::<f32>();
            let a2 = 2.0 * a * r1 - a;
            let c2 = 2.0 * r2;

            let r1 = rand::random::<f32>();
            let r2 = rand::random::<f32>();
            let a3 = 2.0 * a * r1 - a;
            let c3 = 2.0 * r2;

            let wolf = &mut self.wolves[i];

            // Calculate X1, X2, X3
            let d_alpha = (c1 * alpha_pos.x - wolf.position.x).abs();
            let x1_x = alpha_pos.x - a1 * d_alpha;
            let d_alpha_y = (c1 * alpha_pos.y - wolf.position.y).abs();
            let x1_y = alpha_pos.y - a1 * d_alpha_y;

            let d_beta = (c2 * beta_pos.x - wolf.position.x).abs();
            let x2_x = beta_pos.x - a2 * d_beta;
            let d_beta_y = (c2 * beta_pos.y - wolf.position.y).abs();
            let x2_y = beta_pos.y - a2 * d_beta_y;

            let d_delta = (c3 * delta_pos.x - wolf.position.x).abs();
            let x3_x = delta_pos.x - a3 * d_delta;
            let d_delta_y = (c3 * delta_pos.y - wolf.position.y).abs();
            let x3_y = delta_pos.y - a3 * d_delta_y;

            // Average position
            wolf.position = Pos2::new((x1_x + x2_x + x3_x) / 3.0, (x1_y + x2_y + x3_y) / 3.0);

            // Boundary handling
            let bounds = 100.0;
            wolf.position.x = wolf.position.x.clamp(-bounds, bounds);
            wolf.position.y = wolf.position.y.clamp(-bounds, bounds);
        }
    }
}

// ============ Federated Learning State ============

#[derive(Clone, Debug)]
pub struct FederatedVisualState {
    pub nodes: Vec<FederatedNode>,
    pub aggregator: Option<usize>,
    pub global_model: Vec<f32>,
    pub round: u32,
    pub accuracy_history: Vec<f32>,
    pub current_accuracy: f32,
}

#[derive(Clone, Debug)]
pub struct FederatedNode {
    pub id: usize,
    pub position: Pos2,
    pub local_weights: Vec<f32>,
    pub contribution: f32,
    pub is_selected: bool,
    pub training_progress: f32,
}

impl FederatedVisualState {
    pub fn new(node_count: usize) -> Self {
        let mut nodes = Vec::new();
        let bounds = 80.0;

        for i in 0..node_count {
            let angle = (i as f32 / node_count as f32) * std::f32::consts::TAU;
            let radius = 60.0;
            nodes.push(FederatedNode {
                id: i,
                position: Pos2::new(radius * angle.cos(), radius * angle.sin()),
                local_weights: vec![rand::random::<f32>(); 5],
                contribution: rand::random::<f32>(),
                is_selected: false,
                training_progress: 0.0,
            });
        }

        Self {
            nodes,
            aggregator: Some(0),
            global_model: vec![0.5; 5],
            round: 0,
            accuracy_history: Vec::new(),
            current_accuracy: 0.5,
        }
    }

    pub fn step(&mut self) {
        // Simulate training round
        self.round += 1;

        // Select random nodes for this round
        for node in &mut self.nodes {
            node.is_selected = rand::random::<f32>() > 0.3;
            if node.is_selected {
                node.training_progress = (node.training_progress + 0.1).min(1.0);
                // Update local weights
                for w in &mut node.local_weights {
                    *w += (rand::random::<f32>() - 0.5) * 0.1;
                    *w = w.clamp(0.0, 1.0);
                }
            }
        }

        // Aggregate models
        if self.round % 10 == 0 {
            let selected: Vec<&FederatedNode> =
                self.nodes.iter().filter(|n| n.is_selected).collect();
            if !selected.is_empty() {
                for i in 0..self.global_model.len() {
                    let sum: f32 = selected
                        .iter()
                        .map(|n| n.local_weights.get(i).unwrap_or(&0.5))
                        .sum();
                    self.global_model[i] = sum / selected.len() as f32;
                }
            }

            // Improve accuracy over time
            self.current_accuracy = (self.current_accuracy + 0.01).min(0.98);
            self.accuracy_history.push(self.current_accuracy);
            if self.accuracy_history.len() > 100 {
                self.accuracy_history.remove(0);
            }

            // Rotate aggregator
            self.aggregator = Some((self.aggregator.unwrap_or(0) + 1) % self.nodes.len());
        }
    }
}

// ============ Consensus State ============

#[derive(Clone, Debug)]
pub struct ConsensusVisualState {
    pub nodes: Vec<ConsensusNode>,
    pub leader_id: Option<usize>,
    pub term: u32,
    pub log_entries: Vec<LogEntry>,
    pub committed_index: usize,
    pub messages: Vec<ConsensusMessage>,
}

#[derive(Clone, Debug)]
pub struct ConsensusNode {
    pub id: usize,
    pub position: Pos2,
    pub state: NodeState,
    pub vote_granted: bool,
    pub log_index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NodeState {
    Follower,
    Candidate,
    Leader,
}

#[derive(Clone, Debug)]
pub struct LogEntry {
    pub term: u32,
    pub command: String,
    pub committed: bool,
}

#[derive(Clone, Debug)]
pub struct ConsensusMessage {
    pub from: usize,
    pub to: usize,
    pub msg_type: MessageType,
    pub progress: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MessageType {
    RequestVote,
    VoteGranted,
    AppendEntries,
    Heartbeat,
}

impl ConsensusVisualState {
    pub fn new(node_count: usize) -> Self {
        let mut nodes = Vec::new();
        let radius = 70.0;

        for i in 0..node_count {
            let angle = (i as f32 / node_count as f32) * std::f32::consts::TAU;
            nodes.push(ConsensusNode {
                id: i,
                position: Pos2::new(radius * angle.cos(), radius * angle.sin()),
                state: if i == 0 {
                    NodeState::Leader
                } else {
                    NodeState::Follower
                },
                vote_granted: false,
                log_index: 0,
            });
        }

        Self {
            nodes,
            leader_id: Some(0),
            term: 1,
            log_entries: vec![LogEntry {
                term: 1,
                command: "init".to_string(),
                committed: true,
            }],
            committed_index: 0,
            messages: Vec::new(),
        }
    }

    pub fn step(&mut self) {
        // Update message progress
        self.messages.retain_mut(|msg| {
            msg.progress += 0.1;
            msg.progress < 1.0
        });

        // Simulate leader sending heartbeats
        if let Some(leader) = self.leader_id {
            if rand::random::<f32>() > 0.9 {
                for i in 0..self.nodes.len() {
                    if i != leader {
                        self.messages.push(ConsensusMessage {
                            from: leader,
                            to: i,
                            msg_type: MessageType::Heartbeat,
                            progress: 0.0,
                        });
                    }
                }
            }

            // Occasionally add log entry
            if rand::random::<f32>() > 0.95 {
                self.log_entries.push(LogEntry {
                    term: self.term,
                    command: format!("cmd_{}", self.log_entries.len()),
                    committed: false,
                });

                // Send AppendEntries
                for i in 0..self.nodes.len() {
                    if i != leader {
                        self.messages.push(ConsensusMessage {
                            from: leader,
                            to: i,
                            msg_type: MessageType::AppendEntries,
                            progress: 0.0,
                        });
                    }
                }
            }

            // Commit entries
            if self.log_entries.len() > self.committed_index + 1 && rand::random::<f32>() > 0.9 {
                self.committed_index += 1;
                if let Some(entry) = self.log_entries.get_mut(self.committed_index) {
                    entry.committed = true;
                }
            }
        }

        // Simulate leader election (rarely)
        if rand::random::<f32>() > 0.995 {
            self.term += 1;
            let new_leader = rand::random::<usize>() % self.nodes.len();

            // Make old leader follower
            if let Some(old_leader) = self.leader_id {
                if let Some(node) = self.nodes.get_mut(old_leader) {
                    node.state = NodeState::Follower;
                }
            }

            // New leader
            if let Some(node) = self.nodes.get_mut(new_leader) {
                node.state = NodeState::Leader;
            }
            self.leader_id = Some(new_leader);

            // Request votes
            for i in 0..self.nodes.len() {
                if i != new_leader {
                    self.messages.push(ConsensusMessage {
                        from: new_leader,
                        to: i,
                        msg_type: MessageType::RequestVote,
                        progress: 0.0,
                    });
                }
            }
        }

        // Update node log indices
        for node in &mut self.nodes {
            node.log_index = self.committed_index;
        }
    }
}

// ============ Differential Evolution State ============

#[derive(Clone, Debug)]
pub struct DEVisualState {
    pub population: Vec<DEIndividual>,
    pub best_individual: Option<DEIndividual>,
    pub best_fitness: f32,
    pub fitness_history: Vec<f32>,
    pub iteration: u32,
    pub mutation_factor: f32,
    pub crossover_rate: f32,
}

#[derive(Clone, Debug)]
pub struct DEIndividual {
    pub position: Pos2,
    pub fitness: f32,
    pub is_trial: bool,
}

impl DEVisualState {
    pub fn new(pop_size: usize) -> Self {
        let bounds = 100.0;
        let mut population = Vec::new();

        for _ in 0..pop_size {
            population.push(DEIndividual {
                position: Pos2::new(
                    (rand::random::<f32>() - 0.5) * bounds * 2.0,
                    (rand::random::<f32>() - 0.5) * bounds * 2.0,
                ),
                fitness: f32::MAX,
                is_trial: false,
            });
        }

        Self {
            population,
            best_individual: None,
            best_fitness: f32::MAX,
            fitness_history: Vec::new(),
            iteration: 0,
            mutation_factor: 0.8,
            crossover_rate: 0.9,
        }
    }

    pub fn step(&mut self) {
        self.iteration += 1;
        let pop_size = self.population.len();
        let bounds = 100.0;

        // Evaluate fitness
        for ind in &mut self.population {
            ind.fitness = ind.position.x.powi(2) + ind.position.y.powi(2);
            ind.is_trial = false;
        }

        // Find best
        if let Some(best) = self
            .population
            .iter()
            .min_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap())
        {
            if best.fitness < self.best_fitness {
                self.best_fitness = best.fitness;
                self.best_individual = Some(best.clone());
            }
        }

        self.fitness_history.push(self.best_fitness);
        if self.fitness_history.len() > 200 {
            self.fitness_history.remove(0);
        }

        // DE/rand/1/bin mutation and crossover
        let mut new_population = Vec::new();
        for i in 0..pop_size {
            // Select three random distinct individuals
            let mut indices: Vec<usize> = (0..pop_size).filter(|&j| j != i).collect();
            let r1 = indices.remove(rand::random::<usize>() % indices.len());
            let r2 = indices.remove(rand::random::<usize>() % indices.len());
            let r3 = indices.remove(rand::random::<usize>() % indices.len());

            // Mutation
            let mutant_x = self.population[r1].position.x
                + self.mutation_factor
                    * (self.population[r2].position.x - self.population[r3].position.x);
            let mutant_y = self.population[r1].position.y
                + self.mutation_factor
                    * (self.population[r2].position.y - self.population[r3].position.y);

            // Crossover
            let trial_x = if rand::random::<f32>() < self.crossover_rate {
                mutant_x.clamp(-bounds, bounds)
            } else {
                self.population[i].position.x
            };
            let trial_y = if rand::random::<f32>() < self.crossover_rate {
                mutant_y.clamp(-bounds, bounds)
            } else {
                self.population[i].position.y
            };

            let trial = DEIndividual {
                position: Pos2::new(trial_x, trial_y),
                fitness: trial_x.powi(2) + trial_y.powi(2),
                is_trial: true,
            };

            // Selection
            if trial.fitness < self.population[i].fitness {
                new_population.push(trial);
            } else {
                new_population.push(self.population[i].clone());
            }
        }

        self.population = new_population;
    }
}

// ============ Network Types ============

#[derive(Clone, Debug)]
pub struct NetworkTopology {
    pub nodes: Vec<NetworkNode>,
    pub edges: Vec<NetworkEdge>,
}

#[derive(Clone, Debug)]
pub struct NetworkNode {
    pub id: u64,
    pub position: Pos2,
    pub neighbor_count: usize,
}

#[derive(Clone, Debug)]
pub struct NetworkEdge {
    pub from: u64,
    pub to: u64,
    pub link_quality: f32,
    pub rtt_ms: u32,
}

impl NetworkTopology {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
        }
    }
}

// ============ Viewport State ============

#[derive(Clone, Debug)]
pub struct ViewportState {
    pub center: Pos2,
    pub zoom: f32,
    pub show_grid: bool,
    pub show_trails: bool,
    pub show_velocities: bool,
}

impl Default for ViewportState {
    fn default() -> Self {
        Self {
            center: Pos2::ZERO,
            zoom: 2.0,
            show_grid: true,
            show_trails: true,
            show_velocities: true,
        }
    }
}

// ============ Demo Mode ============

#[derive(Clone, Debug)]
pub struct DemoMode {
    pub current_scenario: DemoScenario,
    pub step: u64,
    pub formation_index: usize,
}

impl DemoMode {
    pub fn new() -> Self {
        Self {
            current_scenario: DemoScenario::FormationShowcase,
            step: 0,
            formation_index: 0,
        }
    }

    pub fn scenario_name(&self) -> &'static str {
        match self.current_scenario {
            DemoScenario::FormationShowcase => "Formation Showcase",
            DemoScenario::PSOConvergence => "PSO Optimization",
            DemoScenario::ACOPathfinding => "ACO Pathfinding",
            DemoScenario::GWOHunting => "GWO Wolf Pack",
            DemoScenario::ScaleTest => "Scale Test",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DemoScenario {
    FormationShowcase,
    PSOConvergence,
    ACOPathfinding,
    GWOHunting,
    ScaleTest,
}

// ============ Safety State ============

#[derive(Clone, Debug)]
pub struct SafetyState {
    pub collision_avoidance_enabled: bool,
    pub min_separation: f32,
    pub avoidance_radius: f32,
    pub show_avoidance_zones: bool,
}

impl Default for SafetyState {
    fn default() -> Self {
        Self {
            collision_avoidance_enabled: true,
            min_separation: 5.0,
            avoidance_radius: 15.0,
            show_avoidance_zones: true,
        }
    }
}

// ============ Mission State ============

#[derive(Clone, Debug)]
pub struct MissionState {
    pub waypoints: Vec<Pos2>,
    pub current_waypoint: usize,
    pub is_active: bool,
    pub show_waypoints: bool,
    pub show_path: bool,
}

impl Default for MissionState {
    fn default() -> Self {
        Self {
            waypoints: Vec::new(),
            current_waypoint: 0,
            is_active: false,
            show_waypoints: true,
            show_path: true,
        }
    }
}

// ============ Telemetry State ============

#[derive(Clone, Debug)]
pub struct TelemetryState {
    pub show_health_overlay: bool,
}

impl Default for TelemetryState {
    fn default() -> Self {
        Self {
            show_health_overlay: true,
        }
    }
}

// ============ Failsafe State ============

#[derive(Clone, Debug)]
pub struct FailsafeState {
    pub active_failsafe: Option<String>,
    pub geofence_enabled: bool,
    pub geofence_radius: f32,
    pub show_geofence: bool,
}

impl Default for FailsafeState {
    fn default() -> Self {
        Self {
            active_failsafe: None,
            geofence_enabled: true,
            geofence_radius: 150.0,
            show_geofence: true,
        }
    }
}

// ============ Alert Types ============

#[derive(Clone, Debug)]
pub struct Alert {
    pub message: String,
    pub severity: String,
}

// ============ Swarm Telemetry Stats ============

#[derive(Clone, Debug)]
pub struct SwarmTelemetryStats {
    pub health_status: String,
    pub avg_battery: f32,
    pub min_battery: u8,
    pub avg_speed: f32,
    pub max_speed: f32,
    pub connected_drones: usize,
    pub total_drones: usize,
}

// ============ Safety Helper Methods ============

impl SimulationState {
    /// Get collision warnings for drones that are too close
    pub fn get_collision_warnings(&self) -> Vec<(u64, u64, f32)> {
        let mut warnings = Vec::new();
        for i in 0..self.drones.len() {
            for j in (i + 1)..self.drones.len() {
                let dist = self.drones[i].position.distance(self.drones[j].position);
                if dist < self.safety_state.min_separation * 2.0 {
                    warnings.push((self.drones[i].id, self.drones[j].id, dist));
                }
            }
        }
        warnings
    }

    /// Get swarm telemetry statistics
    pub fn get_swarm_telemetry(&self) -> SwarmTelemetryStats {
        if self.drones.is_empty() {
            return SwarmTelemetryStats {
                health_status: "No Drones".to_string(),
                avg_battery: 0.0,
                min_battery: 0,
                avg_speed: 0.0,
                max_speed: 0.0,
                connected_drones: 0,
                total_drones: 0,
            };
        }

        let total = self.drones.len();
        let avg_battery: f32 =
            self.drones.iter().map(|d| d.battery as f32).sum::<f32>() / total as f32;
        let min_battery = self.drones.iter().map(|d| d.battery).min().unwrap_or(0);
        let avg_speed: f32 =
            self.drones.iter().map(|d| d.velocity.length()).sum::<f32>() / total as f32;
        let max_speed = self
            .drones
            .iter()
            .map(|d| d.velocity.length())
            .fold(0.0f32, |a, b| a.max(b));

        let health_status = if min_battery < 10 {
            "Critical"
        } else if min_battery < 20 {
            "Warning"
        } else {
            "Healthy"
        }
        .to_string();

        SwarmTelemetryStats {
            health_status,
            avg_battery,
            min_battery,
            avg_speed,
            max_speed,
            connected_drones: total,
            total_drones: total,
        }
    }

    /// Add a random waypoint for mission planning
    pub fn add_random_waypoint(&mut self) {
        let x = (rand::random::<f32>() - 0.5) * 200.0;
        let y = (rand::random::<f32>() - 0.5) * 200.0;
        self.mission_state.waypoints.push(Pos2::new(x, y));
    }

    /// Generate survey pattern waypoints
    pub fn generate_survey_pattern(&mut self, pattern: &str) {
        self.mission_state.waypoints.clear();
        self.mission_state.current_waypoint = 0;

        match pattern {
            "lawnmower" => {
                // Lawnmower pattern
                let rows = 5;
                let cols = 4;
                let spacing = 30.0;
                for row in 0..rows {
                    if row % 2 == 0 {
                        for col in 0..cols {
                            let x = col as f32 * spacing - (cols as f32 * spacing / 2.0);
                            let y = row as f32 * spacing - (rows as f32 * spacing / 2.0);
                            self.mission_state.waypoints.push(Pos2::new(x, y));
                        }
                    } else {
                        for col in (0..cols).rev() {
                            let x = col as f32 * spacing - (cols as f32 * spacing / 2.0);
                            let y = row as f32 * spacing - (rows as f32 * spacing / 2.0);
                            self.mission_state.waypoints.push(Pos2::new(x, y));
                        }
                    }
                }
            }
            "spiral" => {
                // Spiral pattern
                let turns = 3.0;
                let points = 30;
                for i in 0..points {
                    let t = i as f32 / points as f32;
                    let angle = t * turns * std::f32::consts::TAU;
                    let radius = t * 80.0;
                    self.mission_state
                        .waypoints
                        .push(Pos2::new(radius * angle.cos(), radius * angle.sin()));
                }
            }
            "square" => {
                // Square pattern
                let size = 60.0;
                self.mission_state.waypoints.push(Pos2::new(-size, -size));
                self.mission_state.waypoints.push(Pos2::new(size, -size));
                self.mission_state.waypoints.push(Pos2::new(size, size));
                self.mission_state.waypoints.push(Pos2::new(-size, size));
                self.mission_state.waypoints.push(Pos2::new(-size, -size));
            }
            _ => {}
        }
    }

    /// Trigger a failsafe condition
    pub fn trigger_failsafe(&mut self, reason: &str) {
        self.failsafe_state.active_failsafe = Some(reason.to_string());
        self.alerts.push(Alert {
            message: format!("FAILSAFE: {}", reason),
            severity: "Critical".to_string(),
        });
    }
}
