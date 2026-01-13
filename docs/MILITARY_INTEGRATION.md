# Military Extensions Integration Guide

This document describes the architecture for separating commercial (open-source) and military (private) code in the Swarm Manager project.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    PUBLIC REPOSITORY (Open Source)                       │
│                    github.com/YOUR_ORG/swarm-manager                     │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                        Core Modules                                │  │
│  │  • algorithms/    - PSO, ACO, GWO, WOA, Hybrid                    │  │
│  │  • consensus/     - Raft, PBFT, Hierarchical                      │  │
│  │  • network/       - Mesh, MAVLink, ESP32, Routing                 │  │
│  │  • control/       - Swarm, Collision, Mission, Task               │  │
│  │  • safety/        - Crypto (ChaCha20/Ed25519), Security, Failsafe │  │
│  │  • ml/            - Federated Learning                            │  │
│  │  • system/        - Config, Telemetry, Clustering                 │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                     Extension Traits (src/extensions/)             │  │
│  │  • CryptoProvider       - Pluggable cryptography                  │  │
│  │  • TacticalProvider     - Pluggable tactical algorithms           │  │
│  │  • SecureCommProvider   - Pluggable secure communications         │  │
│  │  • ExtensionRegistry    - Runtime provider management             │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Git Submodule + Cargo Dependency
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                   PRIVATE REPOSITORY (Restricted Access)                 │
│                github.com/YOUR_ORG/swarm-manager-military                │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    Military Extensions                             │  │
│  │  • Post-quantum cryptography (Kyber, Dilithium, SPHINCS+)         │  │
│  │  • Advanced tactical algorithms                                    │  │
│  │  • Military-grade communication protocols                         │  │
│  │  • Classified mission planning strategies                         │  │
│  │  • Hardware-specific military integrations                        │  │
│  └───────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

## For Commercial Users

Commercial users clone the public repository and use the default implementations:

```bash
# Clone public repository
git clone https://github.com/YOUR_ORG/swarm-manager.git
cd swarm-manager

# Build with default features
cargo build --release

# Use in your project
```

```rust
use drone_swarm_system::{
    DefaultCryptoProvider,
    DefaultTacticalProvider,
    ExtensionRegistry,
};

fn main() {
    // Use default (commercial) providers
    let registry = ExtensionRegistry::new();

    // Access providers
    let crypto = registry.crypto();
    println!("Using: {}", crypto.provider_name());
    // Output: "DefaultCrypto (ChaCha20-Poly1305 + Ed25519)"
}
```

## For Military Users

Authorized users with access to the private repository:

### Initial Setup

```bash
# Clone public repository
git clone https://github.com/YOUR_ORG/swarm-manager.git
cd swarm-manager

# Run military setup script (requires SSH access to private repo)
./scripts/setup-military.sh

# Or manually add submodule
git submodule add git@github.com:YOUR_ORG/swarm-manager-military.git military
```

### Building with Military Extensions

```bash
# Build with military features
cargo build --release --features military

# Run tests with military features
cargo test --features military
```

### Using Military Extensions

```rust
use drone_swarm_system::{
    ExtensionRegistry,
    CryptoProvider,
    TacticalProvider,
};

// Import military implementations (only available with 'military' feature)
#[cfg(feature = "military")]
use swarm_manager_military::{
    PostQuantumCrypto,
    AdvancedTactical,
};

fn main() {
    let mut registry = ExtensionRegistry::new();

    #[cfg(feature = "military")]
    {
        // Replace default providers with military-grade implementations
        registry.set_crypto_provider(Box::new(PostQuantumCrypto::new()));
        registry.set_tactical_provider(Box::new(AdvancedTactical::new()));
    }

    let crypto = registry.crypto();
    println!("Using: {}", crypto.provider_name());
    // Output: "PostQuantumCrypto (Kyber + Dilithium)"
}
```

## Extension Traits

### CryptoProvider

Defines cryptographic operations:

```rust
pub trait CryptoProvider: Send + Sync {
    fn encrypt(&self, plaintext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError>;
    fn decrypt(&self, ciphertext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError>;
    fn sign(&self, data: &[u8], private_key: &[u8]) -> Result<Vec<u8>, CryptoError>;
    fn verify(&self, data: &[u8], signature: &[u8], public_key: &[u8]) -> Result<bool, CryptoError>;
    fn generate_signing_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError>;
    fn generate_exchange_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError>;
    fn key_exchange(&self, our_private: &[u8], their_public: &[u8]) -> Result<Vec<u8>, CryptoError>;
    fn provider_name(&self) -> &'static str;
    fn security_level(&self) -> u32;
}
```

| Implementation | Algorithm | Security Level | Repository |
|----------------|-----------|----------------|------------|
| `DefaultCryptoProvider` | ChaCha20-Poly1305 + Ed25519 | 256-bit | Public |
| `PostQuantumCrypto` | Kyber + Dilithium | Post-quantum | Private |

### TacticalProvider

Defines tactical operations:

```rust
pub trait TacticalProvider: Send + Sync {
    fn compute_formation(&self, positions: &[(DroneId, Position)], objective: &TacticalObjective) -> Result<FormationPlan, TacticalError>;
    fn compute_threat_response(&self, threat: &ThreatAssessment, swarm_state: &SwarmSnapshot) -> Result<ThreatResponse, TacticalError>;
    fn compute_evasion(&self, drone_id: DroneId, current_pos: Position, current_vel: Velocity, threat_pos: Position) -> Result<EvasionManeuver, TacticalError>;
    fn provider_name(&self) -> &'static str;
}
```

| Implementation | Capabilities | Repository |
|----------------|--------------|------------|
| `DefaultTacticalProvider` | Grid formations, basic threat response | Public |
| `AdvancedTactical` | Classified formations, advanced threat response | Private |

### SecureCommProvider

Defines secure communication operations:

```rust
pub trait SecureCommProvider: Send + Sync {
    fn send_secure(&self, to: DroneId, message: &[u8], priority: MessagePriority) -> Result<(), CommError>;
    fn broadcast_secure(&self, message: &[u8], priority: MessagePriority) -> Result<(), CommError>;
    fn receive(&self) -> Result<Option<SecureMessage>, CommError>;
    fn is_channel_secure(&self) -> bool;
    fn rotate_keys(&mut self) -> Result<(), CommError>;
    fn provider_name(&self) -> &'static str;
}
```

## Creating the Private Repository

To set up the private military repository:

### 1. Create Repository Structure

```
swarm-manager-military/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── crypto/
│   │   ├── mod.rs
│   │   ├── kyber.rs
│   │   └── dilithium.rs
│   ├── tactical/
│   │   ├── mod.rs
│   │   └── advanced.rs
│   └── comm/
│       ├── mod.rs
│       └── secure.rs
└── README.md
```

### 2. Cargo.toml for Private Crate

```toml
[package]
name = "swarm-manager-military"
version = "0.1.0"
edition = "2021"
publish = false  # Never publish to crates.io

[dependencies]
drone-swarm-system = { git = "https://github.com/YOUR_ORG/swarm-manager.git" }

# Post-quantum cryptography
pqcrypto-kyber = "0.8"
pqcrypto-dilithium = "0.5"
pqcrypto-sphincsplus = "0.7"
```

### 3. Implement Extension Traits

```rust
// src/lib.rs
use drone_swarm_system::extensions::{
    CryptoProvider, CryptoError,
    TacticalProvider, TacticalError,
    // ... other traits
};

mod crypto;
mod tactical;
mod comm;

pub use crypto::PostQuantumCrypto;
pub use tactical::AdvancedTactical;
pub use comm::MilitaryComm;
```

```rust
// src/crypto/mod.rs
use drone_swarm_system::extensions::{CryptoProvider, CryptoError};

pub struct PostQuantumCrypto {
    // Kyber for key exchange
    // Dilithium for signatures
}

impl CryptoProvider for PostQuantumCrypto {
    fn encrypt(&self, plaintext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        // Post-quantum encryption implementation
    }

    // ... implement other methods

    fn provider_name(&self) -> &'static str {
        "PostQuantumCrypto (Kyber + Dilithium)"
    }

    fn security_level(&self) -> u32 {
        384 // Post-quantum security level
    }
}
```

## Security Considerations

### Export Control Compliance

- **ITAR**: International Traffic in Arms Regulations
- **EAR**: Export Administration Regulations

Ensure all military extensions comply with applicable export control regulations before deployment.

### Access Control

- Private repository requires SSH key authentication
- Use GitHub Teams for access management
- Implement audit logging for access events

### Code Separation

- Military code NEVER enters public git history
- All military-specific logic stays in private repository
- Public repository contains only trait definitions and default implementations

## CI/CD Configuration

### Public Repository (.github/workflows/ci.yml)

```yaml
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Build (Commercial)
        run: cargo build --release
      - name: Test (Commercial)
        run: cargo test
```

### Private Repository (.github/workflows/ci.yml)

```yaml
name: Military CI

on: [push, pull_request]

jobs:
  build:
    runs-on: self-hosted  # Use private runners
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive
      - name: Build (Military)
        run: cargo build --release --features military
      - name: Test (Military)
        run: cargo test --features military
```

## Troubleshooting

### "Repository not found" when setting up military extensions

Ensure you have SSH access to the private repository:

```bash
ssh -T git@github.com
```

### Build fails with "feature military requires swarm-manager-military"

The military feature requires the private repository. Either:
1. Set up the military submodule (authorized users only)
2. Build without the military feature: `cargo build`

### Submodule out of sync

Update the submodule to the latest version:

```bash
git submodule update --remote military
```
