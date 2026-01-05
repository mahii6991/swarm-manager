//! Mission Planning and Waypoint Management
//!
//! This module provides comprehensive mission planning capabilities:
//! - Waypoint missions with configurable actions
//! - Survey patterns (lawnmower, spiral, expanding square)
//! - Search and rescue patterns
//! - Loiter and hold patterns
//! - Dynamic mission modification
//!
//! # Features
//! - `no_std` compatible for embedded systems
//! - Fixed-size buffers using heapless
//! - Support for conditional waypoints
//! - Mission state persistence
//!
//! # Example
//! ```ignore
//! use drone_swarm_system::mission_planning::{Mission, Waypoint, WaypointAction};
//!
//! let mut mission = Mission::new();
//! mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
//! mission.add_waypoint(Waypoint::loiter([10.0, 10.0, -10.0], 30.0));
//! mission.add_waypoint(Waypoint::land([0.0, 0.0, 0.0]));
//! ```

use heapless::Vec;

/// Maximum waypoints in a mission
pub const MAX_WAYPOINTS: usize = 64;

/// Maximum survey points
pub const MAX_SURVEY_POINTS: usize = 256;

/// Waypoint types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaypointType {
    /// Navigate to position
    Goto,
    /// Takeoff to altitude
    Takeoff,
    /// Land at position
    Land,
    /// Loiter at position for duration
    Loiter,
    /// Return to launch
    ReturnToLaunch,
    /// Region of Interest (point camera)
    ROI,
    /// Speed change
    SetSpeed,
    /// Delay/wait
    Delay,
    /// Trigger camera
    CameraTrigger,
    /// Start/stop video
    VideoControl,
    /// Custom command
    Custom,
}

/// Waypoint action to perform at waypoint
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaypointAction {
    /// No action, continue to next
    None,
    /// Hover for specified seconds
    Hover { duration_secs: f32 },
    /// Take photo
    TakePhoto,
    /// Start video recording
    StartVideo,
    /// Stop video recording
    StopVideo,
    /// Execute custom command ID
    CustomCommand { command_id: u16 },
}

/// Waypoint acceptance radius mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AcceptanceMode {
    /// Pass through waypoint (larger radius)
    PassThrough,
    /// Stop at waypoint (smaller radius)
    StopAt,
    /// Custom radius
    Custom { radius: f32 },
}

/// Single waypoint definition
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    /// Waypoint type
    pub wp_type: WaypointType,
    /// Position [x, y, z] in NED frame (z negative = altitude)
    pub position: [f32; 3],
    /// Heading/yaw in radians (NaN = don't care)
    pub heading: f32,
    /// Speed to this waypoint (m/s, 0 = default)
    pub speed: f32,
    /// Acceptance radius mode
    pub acceptance: AcceptanceMode,
    /// Action to perform at waypoint
    pub action: WaypointAction,
    /// Loiter time in seconds (for Loiter type)
    pub loiter_time: f32,
    /// Is waypoint enabled
    pub enabled: bool,
}

impl Waypoint {
    /// Create a simple goto waypoint
    pub fn goto(position: [f32; 3]) -> Self {
        Self {
            wp_type: WaypointType::Goto,
            position,
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::PassThrough,
            action: WaypointAction::None,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Create a goto waypoint with heading
    pub fn goto_with_heading(position: [f32; 3], heading_deg: f32) -> Self {
        Self {
            wp_type: WaypointType::Goto,
            position,
            heading: heading_deg.to_radians(),
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::None,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Create a takeoff waypoint
    pub fn takeoff(altitude: f32) -> Self {
        Self {
            wp_type: WaypointType::Takeoff,
            position: [0.0, 0.0, -altitude.abs()],
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::None,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Create a land waypoint
    pub fn land(position: [f32; 3]) -> Self {
        Self {
            wp_type: WaypointType::Land,
            position: [position[0], position[1], 0.0],
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::None,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Create a loiter waypoint
    pub fn loiter(position: [f32; 3], duration_secs: f32) -> Self {
        Self {
            wp_type: WaypointType::Loiter,
            position,
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::Hover {
                duration_secs,
            },
            loiter_time: duration_secs,
            enabled: true,
        }
    }

    /// Create RTL waypoint
    pub fn rtl() -> Self {
        Self {
            wp_type: WaypointType::ReturnToLaunch,
            position: [0.0, 0.0, 0.0],
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::None,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Create a photo waypoint
    pub fn photo(position: [f32; 3]) -> Self {
        Self {
            wp_type: WaypointType::Goto,
            position,
            heading: f32::NAN,
            speed: 0.0,
            acceptance: AcceptanceMode::StopAt,
            action: WaypointAction::TakePhoto,
            loiter_time: 0.0,
            enabled: true,
        }
    }

    /// Set speed for this waypoint
    pub fn with_speed(mut self, speed: f32) -> Self {
        self.speed = speed;
        self
    }

    /// Set acceptance radius
    pub fn with_acceptance(mut self, radius: f32) -> Self {
        self.acceptance = AcceptanceMode::Custom { radius };
        self
    }

    /// Get acceptance radius in meters
    pub fn acceptance_radius(&self) -> f32 {
        match self.acceptance {
            AcceptanceMode::PassThrough => 5.0,
            AcceptanceMode::StopAt => 1.0,
            AcceptanceMode::Custom { radius } => radius,
        }
    }
}

/// Mission state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    /// Mission not started
    Idle,
    /// Mission running
    Active,
    /// Mission paused
    Paused,
    /// Mission completed
    Completed,
    /// Mission aborted
    Aborted,
}

/// Mission definition
#[derive(Debug, Clone)]
pub struct Mission {
    /// Mission name/ID
    pub id: u16,
    /// Waypoints
    waypoints: Vec<Waypoint, MAX_WAYPOINTS>,
    /// Current waypoint index
    current_index: usize,
    /// Mission state
    state: MissionState,
    /// Home position (for RTL)
    home_position: [f32; 3],
    /// Default cruise speed (m/s)
    cruise_speed: f32,
    /// Repeat mission when complete
    repeat: bool,
    /// Repeat count (0 = infinite)
    repeat_count: u8,
    /// Current repeat iteration
    current_repeat: u8,
}

impl Mission {
    /// Create a new empty mission
    pub fn new() -> Self {
        Self {
            id: 0,
            waypoints: Vec::new(),
            current_index: 0,
            state: MissionState::Idle,
            home_position: [0.0, 0.0, 0.0],
            cruise_speed: 5.0,
            repeat: false,
            repeat_count: 0,
            current_repeat: 0,
        }
    }

    /// Create mission with ID
    pub fn with_id(id: u16) -> Self {
        let mut m = Self::new();
        m.id = id;
        m
    }

    /// Set home position
    pub fn set_home(&mut self, position: [f32; 3]) {
        self.home_position = position;
    }

    /// Set cruise speed
    pub fn set_cruise_speed(&mut self, speed: f32) {
        self.cruise_speed = speed;
    }

    /// Enable mission repeat
    pub fn set_repeat(&mut self, repeat: bool, count: u8) {
        self.repeat = repeat;
        self.repeat_count = count;
    }

    /// Add waypoint to mission
    pub fn add_waypoint(&mut self, waypoint: Waypoint) -> bool {
        self.waypoints.push(waypoint).is_ok()
    }

    /// Insert waypoint at index
    pub fn insert_waypoint(&mut self, index: usize, waypoint: Waypoint) -> bool {
        if index > self.waypoints.len() || self.waypoints.len() >= MAX_WAYPOINTS {
            return false;
        }

        // Shift waypoints
        if self.waypoints.push(waypoint).is_err() {
            return false;
        }

        // Rotate to correct position
        let len = self.waypoints.len();
        for i in (index + 1..len).rev() {
            self.waypoints.swap(i, i - 1);
        }

        true
    }

    /// Remove waypoint at index
    pub fn remove_waypoint(&mut self, index: usize) -> Option<Waypoint> {
        if index >= self.waypoints.len() {
            return None;
        }

        Some(self.waypoints.swap_remove(index))
    }

    /// Get waypoint at index
    pub fn get_waypoint(&self, index: usize) -> Option<&Waypoint> {
        self.waypoints.get(index)
    }

    /// Get mutable waypoint at index
    pub fn get_waypoint_mut(&mut self, index: usize) -> Option<&mut Waypoint> {
        self.waypoints.get_mut(index)
    }

    /// Get current waypoint
    pub fn current_waypoint(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.current_index)
    }

    /// Get current waypoint index
    pub fn current_index(&self) -> usize {
        self.current_index
    }

    /// Get total waypoint count
    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len()
    }

    /// Get mission state
    pub fn state(&self) -> MissionState {
        self.state
    }

    /// Start mission
    pub fn start(&mut self) {
        if !self.waypoints.is_empty() {
            self.state = MissionState::Active;
            self.current_index = 0;
        }
    }

    /// Pause mission
    pub fn pause(&mut self) {
        if self.state == MissionState::Active {
            self.state = MissionState::Paused;
        }
    }

    /// Resume mission
    pub fn resume(&mut self) {
        if self.state == MissionState::Paused {
            self.state = MissionState::Active;
        }
    }

    /// Abort mission
    pub fn abort(&mut self) {
        self.state = MissionState::Aborted;
    }

    /// Reset mission to beginning
    pub fn reset(&mut self) {
        self.current_index = 0;
        self.current_repeat = 0;
        self.state = MissionState::Idle;
    }

    /// Advance to next waypoint
    pub fn advance(&mut self) -> bool {
        if self.state != MissionState::Active {
            return false;
        }

        self.current_index += 1;

        if self.current_index >= self.waypoints.len() {
            if self.repeat && (self.repeat_count == 0 || self.current_repeat < self.repeat_count) {
                // Restart mission
                self.current_index = 0;
                self.current_repeat += 1;
                true
            } else {
                self.state = MissionState::Completed;
                false
            }
        } else {
            true
        }
    }

    /// Jump to specific waypoint
    pub fn jump_to(&mut self, index: usize) -> bool {
        if index < self.waypoints.len() {
            self.current_index = index;
            true
        } else {
            false
        }
    }

    /// Check if position has reached current waypoint
    pub fn check_waypoint_reached(&self, position: [f32; 3]) -> bool {
        if let Some(wp) = self.current_waypoint() {
            let dist = distance_3d(&position, &wp.position);
            dist < wp.acceptance_radius()
        } else {
            false
        }
    }

    /// Get distance to current waypoint
    pub fn distance_to_current(&self, position: [f32; 3]) -> f32 {
        if let Some(wp) = self.current_waypoint() {
            distance_3d(&position, &wp.position)
        } else {
            0.0
        }
    }

    /// Get total mission distance
    pub fn total_distance(&self) -> f32 {
        if self.waypoints.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 1..self.waypoints.len() {
            total += distance_3d(&self.waypoints[i - 1].position, &self.waypoints[i].position);
        }
        total
    }

    /// Estimate mission time at given speed
    pub fn estimated_time(&self, speed: f32) -> f32 {
        let speed = if speed > 0.0 { speed } else { self.cruise_speed };
        let mut time = self.total_distance() / speed;

        // Add loiter times
        for wp in &self.waypoints {
            time += wp.loiter_time;
        }

        time
    }

    /// Get home position
    pub fn home(&self) -> [f32; 3] {
        self.home_position
    }

    /// Clear all waypoints
    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.current_index = 0;
        self.state = MissionState::Idle;
    }
}

impl Default for Mission {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Survey Pattern Generators
// ============================================================================

/// Survey pattern types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurveyPattern {
    /// Lawnmower/boustrophedon pattern
    Lawnmower,
    /// Expanding square search
    ExpandingSquare,
    /// Spiral pattern (inward or outward)
    Spiral { inward: bool },
    /// Sector search
    Sector { angle_deg: f32 },
    /// Parallel track
    ParallelTrack,
}

/// Survey area definition
#[derive(Debug, Clone)]
pub struct SurveyArea {
    /// Area vertices (polygon) [x, y]
    pub vertices: Vec<[f32; 2], 16>,
    /// Survey altitude (positive, meters AGL)
    pub altitude: f32,
    /// Track spacing (meters)
    pub spacing: f32,
    /// Overshoot distance at turns (meters)
    pub overshoot: f32,
    /// Entry angle (degrees from north)
    pub entry_angle: f32,
}

impl Default for SurveyArea {
    fn default() -> Self {
        // Default 100x100m square
        let mut vertices = Vec::new();
        let _ = vertices.push([0.0, 0.0]);
        let _ = vertices.push([100.0, 0.0]);
        let _ = vertices.push([100.0, 100.0]);
        let _ = vertices.push([0.0, 100.0]);

        Self {
            vertices,
            altitude: 30.0,
            spacing: 20.0,
            overshoot: 10.0,
            entry_angle: 0.0,
        }
    }
}

/// Generate survey mission from pattern
pub fn generate_survey_mission(
    area: &SurveyArea,
    pattern: SurveyPattern,
) -> Vec<[f32; 3], MAX_SURVEY_POINTS> {
    match pattern {
        SurveyPattern::Lawnmower => generate_lawnmower(area),
        SurveyPattern::ExpandingSquare => generate_expanding_square(area),
        SurveyPattern::Spiral { inward } => generate_spiral(area, inward),
        SurveyPattern::Sector { angle_deg } => generate_sector(area, angle_deg),
        SurveyPattern::ParallelTrack => generate_lawnmower(area), // Same as lawnmower
    }
}

/// Generate lawnmower/boustrophedon pattern
fn generate_lawnmower(area: &SurveyArea) -> Vec<[f32; 3], MAX_SURVEY_POINTS> {
    let mut points: Vec<[f32; 3], MAX_SURVEY_POINTS> = Vec::new();

    if area.vertices.len() < 3 {
        return points;
    }

    // Find bounding box
    let (min_x, max_x, min_y, max_y) = bounding_box(&area.vertices);
    let width = max_x - min_x;
    let height = max_y - min_y;

    // Calculate rotation based on entry angle
    let angle_rad = area.entry_angle.to_radians();
    let cos_a = libm::cosf(angle_rad);
    let sin_a = libm::sinf(angle_rad);

    // Center of area
    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;

    // Generate tracks
    let num_tracks = ((width.max(height)) / area.spacing).ceil() as i32 + 1;
    let alt = -area.altitude; // NED frame

    let mut forward = true;
    let mut y = min_y - area.overshoot;

    for _ in 0..num_tracks {
        if y > max_y + area.overshoot {
            break;
        }

        let (x1, x2) = if forward {
            (min_x - area.overshoot, max_x + area.overshoot)
        } else {
            (max_x + area.overshoot, min_x - area.overshoot)
        };

        // Rotate points around center
        let p1 = rotate_point(x1, y, cx, cy, cos_a, sin_a);
        let p2 = rotate_point(x2, y, cx, cy, cos_a, sin_a);

        let _ = points.push([p1.0, p1.1, alt]);
        let _ = points.push([p2.0, p2.1, alt]);

        y += area.spacing;
        forward = !forward;
    }

    points
}

/// Generate expanding square search pattern
fn generate_expanding_square(area: &SurveyArea) -> Vec<[f32; 3], MAX_SURVEY_POINTS> {
    let mut points: Vec<[f32; 3], MAX_SURVEY_POINTS> = Vec::new();

    if area.vertices.is_empty() {
        return points;
    }

    // Start from center
    let (min_x, max_x, min_y, max_y) = bounding_box(&area.vertices);
    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;
    let alt = -area.altitude;

    let max_radius = ((max_x - min_x).max(max_y - min_y)) / 2.0 + area.overshoot;

    // Start at center
    let _ = points.push([cx, cy, alt]);

    let mut leg = area.spacing;
    let mut x = cx;
    let mut y = cy;
    let directions = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]; // E, N, W, S
    let mut dir_idx = 0;
    let legs_in_direction = 1;
    let mut leg_count = 0;

    while leg < max_radius * 2.0 {
        // Move in current direction
        x += directions[dir_idx].0 * leg;
        y += directions[dir_idx].1 * leg;

        let _ = points.push([x, y, alt]);

        leg_count += 1;
        if leg_count >= legs_in_direction {
            leg_count = 0;
            dir_idx = (dir_idx + 1) % 4;

            // Increase leg length every 2 direction changes
            if dir_idx % 2 == 0 {
                leg += area.spacing;
            }
        }

        if points.len() >= MAX_SURVEY_POINTS - 1 {
            break;
        }
    }

    points
}

/// Generate spiral pattern
fn generate_spiral(area: &SurveyArea, inward: bool) -> Vec<[f32; 3], MAX_SURVEY_POINTS> {
    let mut points: Vec<[f32; 3], MAX_SURVEY_POINTS> = Vec::new();

    if area.vertices.is_empty() {
        return points;
    }

    let (min_x, max_x, min_y, max_y) = bounding_box(&area.vertices);
    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;
    let alt = -area.altitude;

    let max_radius = ((max_x - min_x).max(max_y - min_y)) / 2.0;
    let num_points = (max_radius * 2.0 * core::f32::consts::PI / area.spacing) as usize;
    let num_points = num_points.min(MAX_SURVEY_POINTS);

    for i in 0..num_points {
        let t = i as f32 / num_points as f32;
        let angle = t * 4.0 * core::f32::consts::PI; // 2 full rotations

        let radius = if inward {
            max_radius * (1.0 - t)
        } else {
            max_radius * t
        };

        let x = cx + radius * libm::cosf(angle);
        let y = cy + radius * libm::sinf(angle);

        let _ = points.push([x, y, alt]);
    }

    points
}

/// Generate sector search pattern
fn generate_sector(area: &SurveyArea, sector_angle: f32) -> Vec<[f32; 3], MAX_SURVEY_POINTS> {
    let mut points: Vec<[f32; 3], MAX_SURVEY_POINTS> = Vec::new();

    if area.vertices.is_empty() {
        return points;
    }

    let (min_x, max_x, min_y, max_y) = bounding_box(&area.vertices);
    let cx = (min_x + max_x) / 2.0;
    let cy = (min_y + max_y) / 2.0;
    let alt = -area.altitude;

    let max_radius = ((max_x - min_x).max(max_y - min_y)) / 2.0 + area.overshoot;
    let sector_rad = sector_angle.to_radians();
    let num_legs = (max_radius / area.spacing).ceil() as usize;

    // Start at center
    let _ = points.push([cx, cy, alt]);

    let base_angle = area.entry_angle.to_radians();

    for i in 0..num_legs {
        let r = (i + 1) as f32 * area.spacing;

        // Leg out at +angle
        let angle1 = base_angle + sector_rad / 2.0;
        let x1 = cx + r * libm::cosf(angle1);
        let y1 = cy + r * libm::sinf(angle1);
        let _ = points.push([x1, y1, alt]);

        // Back to center
        let _ = points.push([cx, cy, alt]);

        // Leg out at -angle
        let angle2 = base_angle - sector_rad / 2.0;
        let x2 = cx + r * libm::cosf(angle2);
        let y2 = cy + r * libm::sinf(angle2);
        let _ = points.push([x2, y2, alt]);

        // Back to center
        let _ = points.push([cx, cy, alt]);

        if points.len() >= MAX_SURVEY_POINTS - 4 {
            break;
        }
    }

    points
}

/// Convert survey points to mission
pub fn survey_to_mission(
    points: &Vec<[f32; 3], MAX_SURVEY_POINTS>,
    take_photos: bool,
) -> Mission {
    let mut mission = Mission::new();

    for point in points {
        let wp = if take_photos {
            Waypoint::photo(*point)
        } else {
            Waypoint::goto(*point)
        };
        mission.add_waypoint(wp);
    }

    mission
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Calculate 3D distance
fn distance_3d(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    libm::sqrtf(dx * dx + dy * dy + dz * dz)
}

/// Get bounding box of polygon
fn bounding_box(vertices: &Vec<[f32; 2], 16>) -> (f32, f32, f32, f32) {
    if vertices.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;

    for v in vertices {
        min_x = min_x.min(v[0]);
        max_x = max_x.max(v[0]);
        min_y = min_y.min(v[1]);
        max_y = max_y.max(v[1]);
    }

    (min_x, max_x, min_y, max_y)
}

/// Rotate point around center
fn rotate_point(x: f32, y: f32, cx: f32, cy: f32, cos_a: f32, sin_a: f32) -> (f32, f32) {
    let dx = x - cx;
    let dy = y - cy;
    (
        cx + dx * cos_a - dy * sin_a,
        cy + dx * sin_a + dy * cos_a,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_waypoint_creation() {
        let wp = Waypoint::goto([10.0, 20.0, -30.0]);
        assert_eq!(wp.wp_type, WaypointType::Goto);
        assert_eq!(wp.position, [10.0, 20.0, -30.0]);
    }

    #[test]
    fn test_mission_basic() {
        let mut mission = Mission::new();
        assert!(mission.add_waypoint(Waypoint::takeoff(10.0)));
        assert!(mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0])));
        assert!(mission.add_waypoint(Waypoint::land([0.0, 0.0, 0.0])));

        assert_eq!(mission.waypoint_count(), 3);
        assert_eq!(mission.state(), MissionState::Idle);

        mission.start();
        assert_eq!(mission.state(), MissionState::Active);
        assert_eq!(mission.current_index(), 0);

        assert!(mission.advance());
        assert_eq!(mission.current_index(), 1);

        assert!(mission.advance());
        assert_eq!(mission.current_index(), 2);

        assert!(!mission.advance()); // No more waypoints
        assert_eq!(mission.state(), MissionState::Completed);
    }

    #[test]
    fn test_mission_pause_resume() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([20.0, 0.0, -10.0]));

        mission.start();
        mission.pause();
        assert_eq!(mission.state(), MissionState::Paused);

        mission.resume();
        assert_eq!(mission.state(), MissionState::Active);
    }

    #[test]
    fn test_waypoint_reached() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]).with_acceptance(2.0));
        mission.start();

        assert!(!mission.check_waypoint_reached([0.0, 0.0, -10.0]));
        assert!(mission.check_waypoint_reached([10.0, 0.5, -10.0]));
    }

    #[test]
    fn test_lawnmower_generation() {
        let area = SurveyArea::default();
        let points = generate_lawnmower(&area);

        assert!(!points.is_empty());
        // Points should be at survey altitude
        for p in &points {
            assert!((p[2] + area.altitude).abs() < 0.01);
        }
    }

    #[test]
    fn test_expanding_square() {
        let area = SurveyArea::default();
        let points = generate_expanding_square(&area);

        assert!(!points.is_empty());
        // First point should be at center
        let (min_x, max_x, min_y, max_y) = bounding_box(&area.vertices);
        let cx = (min_x + max_x) / 2.0;
        let cy = (min_y + max_y) / 2.0;

        assert!((points[0][0] - cx).abs() < 0.01);
        assert!((points[0][1] - cy).abs() < 0.01);
    }

    #[test]
    fn test_total_distance() {
        let mut mission = Mission::new();
        mission.add_waypoint(Waypoint::goto([0.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([10.0, 0.0, -10.0]));
        mission.add_waypoint(Waypoint::goto([10.0, 10.0, -10.0]));

        let dist = mission.total_distance();
        assert!((dist - 20.0).abs() < 0.01);
    }
}
