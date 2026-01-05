---
layout: default
title: API Reference
nav_order: 4
---

# API Reference

Complete reference for all public APIs in the Drone Swarm Communication System.

## Table of Contents

- [Core Types](#core-types)
- [Crypto Module](#crypto-module)
- [Network Module](#network-module)
- [Consensus Module](#consensus-module)
- [Federated Learning Module](#federated-learning-module)
- [Swarm Module](#swarm-module)
- [Configuration](#configuration)

## Core Types

### DroneId

Unique identifier for each drone in the swarm.

```rust
pub struct DroneId(u32);

impl DroneId {
    pub fn new(id: u32) -> Self
    pub fn as_u32(&self) -> u32
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
assert_eq!(drone_id.as_u32(), 1);
```

### Position

3D position in space (meters).

```rust
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Position {
    pub fn distance_to(&self, other: &Position) -> f32
    pub fn midpoint(&self, other: &Position) -> Position
}
```

**Example**:
```rust
let pos1 = Position { x: 0.0, y: 0.0, z: 10.0 };
let pos2 = Position { x: 3.0, y: 4.0, z: 10.0 };
let distance = pos1.distance_to(&pos2); // 5.0 meters
```

### Velocity

3D velocity vector (m/s).

```rust
pub struct Velocity {
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
}
```

## Crypto Module

### CryptoContext

Main cryptographic context for encryption, signing, and verification.

```rust
pub struct CryptoContext { /* private fields */ }

impl CryptoContext {
    /// Creates a new crypto context from a seed
    pub fn new(seed: [u8; 32]) -> Self

    /// Encrypts data with ChaCha20-Poly1305
    pub fn encrypt(&self, plaintext: &[u8], nonce: &[u8; 12]) -> Vec<u8>

    /// Decrypts data
    pub fn decrypt(&self, ciphertext: &[u8], nonce: &[u8; 12]) -> Result<Vec<u8>, CryptoError>

    /// Signs data with Ed25519
    pub fn sign(&self, message: &[u8]) -> Signature

    /// Verifies signature
    pub fn verify(&self, message: &[u8], signature: &Signature, public_key: &PublicKey) -> bool

    /// Computes BLAKE3 hash
    pub fn hash_blake3(&self, data: &[u8]) -> [u8; 32]

    /// Computes SHA3-256 hash
    pub fn hash_sha3(&self, data: &[u8]) -> [u8; 32]
}
```

**Example**:
```rust
let seed = [42u8; 32]; // Use hardware RNG in production
let crypto = CryptoContext::new(seed);

// Encryption
let message = b"Secret message";
let nonce = [0u8; 12];
let ciphertext = crypto.encrypt(message, &nonce);
let decrypted = crypto.decrypt(&ciphertext, &nonce).unwrap();
assert_eq!(message, decrypted.as_slice());

// Signing
let signature = crypto.sign(message);
let is_valid = crypto.verify(message, &signature, &crypto.public_key());
assert!(is_valid);
```

### Signature

Ed25519 signature (64 bytes).

```rust
pub struct Signature([u8; 64]);
```

### PublicKey

Ed25519 public key (32 bytes).

```rust
pub struct PublicKey([u8; 32]);
```

## Network Module

### MeshNetwork

Manages mesh networking and routing.

```rust
pub struct MeshNetwork { /* private fields */ }

impl MeshNetwork {
    /// Creates a new mesh network
    pub fn new(drone_id: DroneId) -> Self

    /// Adds a neighbor
    pub fn add_neighbor(&mut self, neighbor_id: DroneId, distance: f32)

    /// Removes a neighbor
    pub fn remove_neighbor(&mut self, neighbor_id: DroneId)

    /// Sends a message to a destination
    pub fn send_message(&mut self, dest: DroneId, payload: &[u8]) -> Result<(), NetworkError>

    /// Receives a message (non-blocking)
    pub fn receive(&mut self) -> Option<Message>

    /// Updates routing table
    pub fn update_routes(&mut self)

    /// Gets neighbor count
    pub fn neighbor_count(&self) -> usize

    /// Gets routing table size
    pub fn route_count(&self) -> usize
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
let mut network = MeshNetwork::new(drone_id);

// Add neighbors
network.add_neighbor(DroneId::new(2), 50.0);
network.add_neighbor(DroneId::new(3), 75.0);

// Send message
let dest = DroneId::new(2);
network.send_message(dest, b"Hello").unwrap();

// Receive messages
while let Some(msg) = network.receive() {
    println!("Received from {}: {:?}", msg.source.as_u32(), msg.payload);
}
```

### Message

Network message structure.

```rust
pub struct Message {
    pub source: DroneId,
    pub dest: DroneId,
    pub payload: Vec<u8>,
    pub nonce: [u8; 12],
    pub timestamp: u64,
}
```

## Consensus Module

### ConsensusEngine

Raft-based consensus engine.

```rust
pub struct ConsensusEngine { /* private fields */ }

impl ConsensusEngine {
    /// Creates a new consensus engine
    pub fn new(drone_id: DroneId, heartbeat_interval_ms: u64) -> Self

    /// Processes a tick (call periodically)
    pub fn tick(&mut self)

    /// Proposes a value to be agreed upon
    pub fn propose(&mut self, value: Vec<u8>) -> Result<(), ConsensusError>

    /// Gets the current state (Leader, Follower, Candidate)
    pub fn state(&self) -> RaftState

    /// Gets the current term
    pub fn current_term(&self) -> u64

    /// Checks if this node is the leader
    pub fn is_leader(&self) -> bool

    /// Gets committed entries
    pub fn get_committed(&self) -> Vec<LogEntry>
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
let mut consensus = ConsensusEngine::new(drone_id, 150);

// Main loop
loop {
    consensus.tick();

    if consensus.is_leader() {
        // Propose a value
        consensus.propose(b"New command".to_vec()).unwrap();
    }

    // Check for committed entries
    for entry in consensus.get_committed() {
        println!("Committed: {:?}", entry.data);
    }

    std::thread::sleep(std::time::Duration::from_millis(10));
}
```

### RaftState

Raft node state.

```rust
pub enum RaftState {
    Follower,
    Candidate,
    Leader,
}
```

### LogEntry

Replicated log entry.

```rust
pub struct LogEntry {
    pub term: u64,
    pub index: u64,
    pub data: Vec<u8>,
}
```

## Federated Learning Module

### FederatedLearner

Manages federated learning across the swarm.

```rust
pub struct FederatedLearner { /* private fields */ }

impl FederatedLearner {
    /// Creates a new federated learner
    pub fn new(drone_id: DroneId) -> Self

    /// Trains on local data
    pub fn train_local(&mut self, data: &[TrainingSample]) -> ModelUpdate

    /// Aggregates model updates (leader only)
    pub fn aggregate(&mut self, updates: &[ModelUpdate]) -> GlobalModel

    /// Updates local model from global model
    pub fn update_model(&mut self, global_model: &GlobalModel)

    /// Gets current model accuracy
    pub fn accuracy(&self) -> f32
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
let mut learner = FederatedLearner::new(drone_id);

// Train on local data
let samples = vec![/* training data */];
let update = learner.train_local(&samples);

// Leader aggregates
let updates = vec![update];
let global_model = learner.aggregate(&updates);

// Update local model
learner.update_model(&global_model);

println!("Accuracy: {:.2}%", learner.accuracy() * 100.0);
```

### ModelUpdate

Local model update (gradients).

```rust
pub struct ModelUpdate {
    pub drone_id: DroneId,
    pub gradients: Vec<f32>,
    pub sample_count: usize,
}
```

### GlobalModel

Global model parameters.

```rust
pub struct GlobalModel {
    pub weights: Vec<f32>,
    pub version: u64,
}
```

## Swarm Module

### SwarmController

Controls swarm behavior and formations.

```rust
pub struct SwarmController { /* private fields */ }

impl SwarmController {
    /// Creates a new swarm controller
    pub fn new(drone_id: DroneId, position: Position) -> Self

    /// Sets the formation
    pub fn set_formation(&mut self, formation: Formation)

    /// Updates position based on formation
    pub fn update_position(&mut self)

    /// Gets current position
    pub fn get_position(&self) -> Position

    /// Sets target position
    pub fn set_destination(&mut self, destination: Position)

    /// Checks for collisions with other drones
    pub fn check_collision(&self, other_positions: &[Position]) -> bool

    /// Plans path using specified algorithm
    pub fn plan_path(&mut self, algorithm: PathAlgorithm, waypoints: &[Position])
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
let position = Position { x: 0.0, y: 0.0, z: 10.0 };
let mut swarm = SwarmController::new(drone_id, position);

// Set formation
swarm.set_formation(Formation::Circle { radius: 50.0 });

// Main loop
loop {
    swarm.update_position();
    let pos = swarm.get_position();
    println!("Position: ({:.2}, {:.2}, {:.2})", pos.x, pos.y, pos.z);

    std::thread::sleep(std::time::Duration::from_millis(100));
}
```

### Formation

Swarm formation types.

```rust
pub enum Formation {
    Grid { spacing: f32, rows: usize, cols: usize },
    Line { spacing: f32, direction: f32 },
    Circle { radius: f32 },
    VFormation { spacing: f32, angle: f32 },
}
```

### PathAlgorithm

Path planning algorithms.

```rust
pub enum PathAlgorithm {
    PSO,  // Particle Swarm Optimization
    ACO,  // Ant Colony Optimization
    GWO,  // Grey Wolf Optimizer
}
```

## Configuration

### SwarmConfig

Global configuration for the swarm system.

```rust
pub struct SwarmConfig {
    pub drone_id: DroneId,
    pub encryption_enabled: bool,
    pub signature_verification: bool,
    pub consensus_enabled: bool,
    pub federated_learning_enabled: bool,
    pub max_neighbors: usize,
    pub comm_range: f32,
    pub heartbeat_interval: u64,
    pub learning_rate: f32,
}

impl SwarmConfig {
    pub fn new(drone_id: DroneId) -> Self
    pub fn default() -> Self
}
```

**Example**:
```rust
let drone_id = DroneId::new(1);
let mut config = SwarmConfig::new(drone_id);

// Customize configuration
config.encryption_enabled = true;
config.max_neighbors = 10;
config.comm_range = 1000.0; // 1km
config.heartbeat_interval = 150; // ms
```

## Error Types

### CryptoError

Cryptographic operation errors.

```rust
pub enum CryptoError {
    DecryptionFailed,
    InvalidSignature,
    InvalidKeyLength,
    InvalidNonceLength,
}
```

### NetworkError

Network operation errors.

```rust
pub enum NetworkError {
    RouteNotFound,
    MessageTooLarge,
    QueueFull,
    InvalidDestination,
}
```

### ConsensusError

Consensus operation errors.

```rust
pub enum ConsensusError {
    NotLeader,
    LogFull,
    InvalidTerm,
    CommitFailed,
}
```

## Constants

```rust
// Cryptographic
pub const KEY_SIZE: usize = 32;
pub const NONCE_SIZE: usize = 12;
pub const SIGNATURE_SIZE: usize = 64;

// Network
pub const MAX_NEIGHBORS: usize = 32;
pub const MAX_MESSAGE_SIZE: usize = 1024;
pub const MAX_HOPS: u8 = 10;

// Consensus
pub const MAX_LOG_ENTRIES: usize = 1000;
pub const ELECTION_TIMEOUT_MIN: u64 = 150; // ms
pub const ELECTION_TIMEOUT_MAX: u64 = 300; // ms

// Swarm
pub const MIN_SAFE_DISTANCE: f32 = 5.0; // meters
pub const MAX_VELOCITY: f32 = 20.0; // m/s
```

## Full API Documentation

For complete API documentation with all internal details, build and view the Rust docs:

```bash
cargo doc --open
```

This will generate and open the full documentation in your browser, including:
- All public and private items
- Source code links
- Trait implementations
- Example code
- Type aliases
- Re-exports

---

[Back to Documentation Home](./index.html)
