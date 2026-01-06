# System Architecture

This document provides a comprehensive overview of the Drone Swarm Communication System architecture.

## Table of Contents

- [Overview](#overview)
- [Layered Architecture](#layered-architecture)
- [Core Modules](#core-modules)
- [Data Flow](#data-flow)
- [Design Principles](#design-principles)
- [Technology Decisions](#technology-decisions)

## Overview

The system follows a layered architecture pattern, separating concerns across six distinct layers from hardware abstraction to application logic.

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│              (Swarm Coordination & Tasks)                │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│              Federated Learning Layer                    │
│         (Distributed Model Training & AI)                │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                Consensus Layer                           │
│           (SwarmRaft Distributed Agreement)              │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│              Security & Crypto Layer                     │
│    (Encryption, Signatures, Access Control, IDS)         │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│              Network Layer                               │
│         (Mesh Routing, Multi-hop, Discovery)             │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│            Hardware Abstraction Layer                    │
│         (Embedded HAL, Microcontroller Support)          │
└─────────────────────────────────────────────────────────┘
```

## Layered Architecture

### Layer 1: Hardware Abstraction Layer (HAL)

**Purpose**: Provides a unified interface to hardware components across different platforms.

**Components**:
- Sensor interfaces (GPS, IMU, compass)
- Radio module drivers (LoRa, Wi-Fi, Bluetooth)
- Motor controllers
- Power management
- Hardware RNG (Random Number Generator)

**Key Features**:
- Platform-independent API
- No-std compatible for embedded systems
- Zero-cost abstractions
- Compile-time hardware configuration

**Example**:
```rust
pub trait RadioDriver {
    fn send(&mut self, data: &[u8]) -> Result<(), Error>;
    fn receive(&mut self) -> Result<Option<Vec<u8>>, Error>;
    fn set_channel(&mut self, channel: u8);
}
```

### Layer 2: Network Layer

**Purpose**: Manages mesh networking, routing, and message delivery.

**Components**:
- **Mesh Network Manager**: Maintains network topology
- **Routing Engine**: Implements AODV (Ad-hoc On-Demand Distance Vector) routing
- **Link Quality Monitor**: Tracks connection health
- **Message Queue**: Buffers messages for reliable delivery

**Key Algorithms**:
1. **Route Discovery**:
   ```
   1. Source broadcasts RREQ (Route Request)
   2. Intermediate nodes forward RREQ
   3. Destination sends RREP (Route Reply)
   4. Source receives RREP and updates routing table
   ```

2. **Multi-hop Forwarding**:
   ```
   [Drone A] --hop1--> [Drone B] --hop2--> [Drone C]
   ```

3. **Link Quality Calculation**:
   ```
   LQ = (0.4 × RSSI) + (0.3 × PDR) + (0.3 × Latency)

   where:
   - RSSI: Received Signal Strength Indicator
   - PDR: Packet Delivery Ratio
   - Latency: Round-trip time
   ```

**Data Structures**:
```rust
pub struct MeshNetwork {
    drone_id: DroneId,
    neighbors: NeighborTable,
    routing_table: RoutingTable,
    message_queue: MessageQueue,
}

pub struct RoutingEntry {
    dest: DroneId,
    next_hop: DroneId,
    hop_count: u8,
    link_quality: f32,
    last_updated: Timestamp,
}
```

### Layer 3: Security & Crypto Layer

**Purpose**: Provides end-to-end security for all communications.

**Components**:
- **Encryption Engine**: ChaCha20-Poly1305 AEAD
- **Signature Manager**: Ed25519 digital signatures
- **Key Exchange**: X25519 Diffie-Hellman
- **Hash Functions**: BLAKE3 + SHA3-256
- **Access Control**: Role-based permissions
- **Intrusion Detection System (IDS)**: Anomaly detection

**Security Pipeline**:
```
[Plaintext]
    ↓
[Sign with Ed25519]
    ↓
[Encrypt with ChaCha20-Poly1305]
    ↓
[Add nonce + timestamp]
    ↓
[Ciphertext + MAC]

Decryption reverses this process
```

**Nonce Generation**:
```rust
// Prevents replay attacks
nonce = counter || timestamp || random_bytes
```

**Key Features**:
- Perfect forward secrecy
- Replay attack protection (nonce tracking)
- Byzantine fault tolerance
- Rate limiting
- Audit logging

### Layer 4: Consensus Layer (SwarmRaft)

**Purpose**: Enables distributed decision-making and state synchronization.

**Based on**: Raft consensus algorithm

**Components**:
- **Leader Election**: Selects a leader drone
- **Log Replication**: Ensures all drones have same state
- **State Machine**: Applies committed log entries
- **Heartbeat Manager**: Maintains leader liveness

**Raft States**:
```
┌──────────┐     timeout     ┌──────────┐
│ Follower │ ──────────────> │Candidate │
└──────────┘                 └──────────┘
     ↑                            │
     │                            │ wins election
     │                            ↓
     │                       ┌────────┐
     └───────────────────────│ Leader │
         receives heartbeat  └────────┘
```

**Consensus Process**:
1. **Normal Operation**:
   - Leader sends heartbeats every 150ms
   - Followers acknowledge
   - Client requests go to leader

2. **Log Replication**:
   - Leader appends entry to local log
   - Leader sends AppendEntries RPC to followers
   - Followers append entry and acknowledge
   - Leader commits entry when majority acknowledges
   - Leader notifies followers to commit

3. **Leader Election**:
   - Follower times out (no heartbeat received)
   - Follower becomes candidate, increments term
   - Candidate votes for itself, requests votes
   - Other nodes vote (at most one vote per term)
   - Candidate with majority becomes leader

**Optimization for Drones**:
- Reduced heartbeat interval (50ms for low-latency)
- Compact log storage (bounded memory)
- Priority-based leader selection (battery, position)

### Layer 5: Federated Learning Layer

**Purpose**: Enables collaborative AI model training without sharing raw data.

**Components**:
- **Model Manager**: Maintains local neural network
- **Gradient Calculator**: Computes model updates
- **Aggregator**: Combines updates from multiple drones (FedAvg)
- **Byzantine Detector**: Filters malicious updates

**Federated Averaging (FedAvg)**:
```
Global Model Update:
w(t+1) = Σ (n_i / n) × w_i(t)

where:
- w(t+1): New global model
- n_i: Number of samples on drone i
- n: Total samples across all drones
- w_i(t): Local model update from drone i
```

**Training Process**:
```
1. Leader broadcasts current global model
2. Each drone trains on local data
3. Drones send gradients (not raw data!) to leader
4. Leader aggregates gradients using FedAvg
5. Leader updates global model
6. Repeat
```

**Byzantine-Resistant Aggregation**:
```rust
// Krum algorithm: Select most trustworthy updates
fn krum_aggregate(updates: &[ModelUpdate]) -> ModelUpdate {
    let scores = compute_krum_scores(updates);
    let trusted_updates = select_top_k(updates, scores, k);
    average(trusted_updates)
}
```

### Layer 6: Application Layer

**Purpose**: Implements swarm coordination and mission-specific logic.

**Components**:
- **Swarm Controller**: Manages swarm behavior
- **Formation Manager**: Maintains geometric formations
- **Task Allocator**: Distributes tasks among drones
- **Path Planner**: Uses ACO, PSO, GWO for navigation
- **Collision Avoidance**: Prevents inter-drone collisions

**Swarm Intelligence Algorithms**:

1. **Particle Swarm Optimization (PSO)**:
   ```
   Velocity Update:
   v_i = w×v_i + c1×r1×(pbest_i - x_i) + c2×r2×(gbest - x_i)

   Position Update:
   x_i = x_i + v_i

   where:
   - w: inertia weight (0.7)
   - c1, c2: learning factors (1.5, 1.5)
   - pbest_i: personal best position
   - gbest: global best position
   ```

2. **Ant Colony Optimization (ACO)**:
   ```
   Pheromone Update:
   τ_ij = (1-ρ)×τ_ij + Σ Δτ_ij^k

   Probability of selecting edge (i,j):
   p_ij = (τ_ij^α × η_ij^β) / Σ (τ_ik^α × η_ik^β)

   where:
   - τ_ij: pheromone on edge (i,j)
   - η_ij: heuristic (1/distance)
   - ρ: evaporation rate (0.1)
   - α, β: pheromone/heuristic importance (1.0, 2.0)
   ```

3. **Grey Wolf Optimizer (GWO)**:
   ```
   Position Update:
   X(t+1) = (X1 + X2 + X3) / 3

   where:
   X1 = X_α - A1 × |C1 × X_α - X|
   X2 = X_β - A2 × |C2 × X_β - X|
   X3 = X_δ - A3 × |C3 × X_δ - X|

   - X_α, X_β, X_δ: Top 3 solutions
   - A, C: coefficient vectors
   ```

## Core Modules

The codebase is organized into logical module groups under `src/`:

### Module Group: `safety/`

**Location**: `src/safety/`

**Submodules**:
- `crypto.rs` - Encryption & signatures
- `security.rs` - Intrusion detection
- `failsafe.rs` - Safety behaviors
- `fault_tolerance.rs` - Self-healing

**Key Types**:
```rust
pub struct CryptoContext {
    signing_key: SigningKey,
    verify_key: VerifyKey,
    encryption_key: [u8; 32],
}
```

### Module Group: `network/`

**Location**: `src/network/`

**Submodules**:
- `core.rs` - Mesh network management
- `mesh.rs` - ESP32 mesh protocol
- `mavlink.rs` - MAVLink interface
- `esp32.rs` - ESP32-specific networking
- `routing/` - Proactive routing & link prediction

**Key Types**:
```rust
pub struct MeshNetwork {
    drone_id: DroneId,
    neighbors: NeighborTable,
    routing_table: RoutingTable,
}
```

### Module Group: `consensus/`

**Location**: `src/consensus/`

**Submodules**:
- `raft.rs` - SwarmRaft protocol
- `pbft.rs` - Byzantine fault tolerance
- `hierarchical.rs` - Hierarchical consensus
- `merkle.rs` - Merkle tree logging

**Key Types**:
```rust
pub struct ConsensusEngine {
    state: RaftState,
    log: ReplicatedLog,
    current_term: u64,
    voted_for: Option<DroneId>,
}
```

### Module Group: `ml/`

**Location**: `src/ml/`

**Submodules**:
- `federated.rs` - Federated learning

**Key Types**:
```rust
pub struct FederatedLearner {
    model: NeuralNetwork,
    aggregator: Aggregator,
    byzantine_detector: ByzantineDetector,
}
```

### Module Group: `control/`

**Location**: `src/control/`

**Submodules**:
- `swarm.rs` - Formation control
- `collision.rs` - Collision avoidance (VO, RVO, ORCA, APF)
- `mission.rs` - Mission planning & waypoints
- `coordinator.rs` - Multi-drone coordination
- `task.rs` - Task allocation

**Key Types**:
```rust
pub struct SwarmController {
    drone_id: DroneId,
    position: Position,
    formation: Formation,
    path_planner: PathPlanner,
}
```

### Module Group: `algorithms/`

**Location**: `src/algorithms/`

**Submodules**:
- `pso/` - Particle Swarm Optimization (basic & advanced)
- `aco.rs` - Ant Colony Optimization
- `gwo.rs` - Grey Wolf Optimizer
- `woa.rs` - Whale Optimization Algorithm
- `hybrid.rs` - Hybrid optimizer
- `selector.rs` - Deep RL algorithm selection

### Module Group: `system/`

**Location**: `src/system/`

**Submodules**:
- `config.rs` - Configuration management
- `telemetry.rs` - Health monitoring
- `time.rs` - Time abstraction
- `clustering.rs` - Cluster management

## Data Flow

### Message Transmission Flow

```
Application
    │
    ↓ (plaintext message)
Security Layer
    │ (encrypt + sign)
    ↓ (ciphertext + signature)
Network Layer
    │ (add routing headers)
    ↓ (packet)
HAL Layer
    │ (serialize)
    ↓ (bytes)
Radio Hardware
    │
    ↓ (RF transmission)
  [AIR]
```

### Message Reception Flow

```
Radio Hardware
    ↓ (bytes)
HAL Layer
    ↓ (deserialize)
Network Layer
    │ (check routing, forward if needed)
    ↓ (ciphertext + signature)
Security Layer
    │ (verify + decrypt)
    ↓ (plaintext message)
Application
```

### Consensus State Replication

```
Leader:
  1. Receives client request
  2. Appends to local log
  3. Sends AppendEntries to followers
  4. Waits for majority ACK
  5. Commits entry
  6. Notifies followers
  7. Applies to state machine

Follower:
  1. Receives AppendEntries RPC
  2. Validates term and log consistency
  3. Appends entry to local log
  4. Sends ACK to leader
  5. Waits for commit notification
  6. Applies to state machine
```

## Design Principles

### 1. Safety First

- **100% Safe Rust**: No unsafe blocks
- **Compile-time Guarantees**: Ownership prevents data races
- **No Heap Allocation**: Predictable memory usage
- **Bounded Collections**: Prevents unbounded growth

### 2. Defense in Depth

Multiple security layers:
- Cryptographic protection (encryption + signatures)
- Byzantine fault tolerance
- Intrusion detection
- Rate limiting
- Audit logging

### 3. Resource Efficiency

- **No-std Compatible**: Runs on embedded systems
- **Zero-copy**: Minimal memory allocations
- **Efficient Serialization**: Postcard format
- **Compact Binary**: < 200KB release build

### 4. Modularity

- Clear separation of concerns
- Loosely coupled modules
- Well-defined interfaces
- Dependency injection

### 5. Testability

- Unit tests for each module
- Integration tests for interactions
- Property-based testing for crypto
- Simulation environment

## Technology Decisions

### Why Rust?

- **Memory Safety**: No buffer overflows, use-after-free, data races
- **Performance**: Zero-cost abstractions, no GC pauses
- **Embedded Support**: Excellent no-std ecosystem
- **Tooling**: Cargo, Clippy, rustfmt, rust-analyzer

### Why ChaCha20-Poly1305?

- **Speed**: Faster than AES on non-AES-NI hardware
- **Security**: Authenticated encryption (AEAD)
- **Simplicity**: Single algorithm for confidentiality + integrity
- **Side-channel Resistance**: Constant-time implementation

### Why Raft Consensus?

- **Understandability**: Easier to reason about than Paxos
- **Proven**: Used in production (etcd, Consul, etc.)
- **Crash Fault Tolerance**: Tolerates f failures with 2f+1 nodes
- **Strong Consistency**: Linearizable reads/writes

### Why Federated Learning?

- **Privacy**: Raw data never leaves drone
- **Bandwidth**: Share gradients (small) not datasets (large)
- **Robustness**: Byzantine-resistant aggregation
- **Scalability**: Trains across distributed drones

## Performance Characteristics

### Time Complexity

| Operation | Complexity |
|-----------|-----------|
| Message Routing | O(log n) |
| Neighbor Discovery | O(n) |
| Consensus Agreement | O(n) |
| Formation Update | O(1) |
| Path Planning (ACO) | O(m × n) |

### Space Complexity

| Component | Memory Usage |
|-----------|-------------|
| Routing Table | O(n) neighbors |
| Consensus Log | O(k) entries (bounded) |
| Message Queue | O(m) messages (bounded) |
| Crypto Context | O(1) constant |

### Scalability

- **Network**: Tested with 100+ drones
- **Consensus**: Optimal with 3-7 nodes (Raft limitation)
- **Federated Learning**: Linear scaling
- **Formation Control**: Handles 1000+ drones

---

For implementation details, see the [API Reference](api-reference.md).
