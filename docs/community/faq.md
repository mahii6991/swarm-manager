# Frequently Asked Questions

Common questions and troubleshooting guide for the drone swarm system.

## General Questions

### What is this project?

A production-ready Rust library for autonomous drone swarm coordination, featuring:
- Multi-drone formation control and path planning
- Enterprise-grade end-to-end encryption
- Swarm intelligence algorithms (PSO, ACO, GWO)
- Federated learning for collaborative AI
- Embedded systems support (no_std compatible)

### Who is this for?

- **Researchers**: Academic swarm robotics and multi-agent systems research
- **Developers**: Building commercial drone applications (agriculture, delivery, monitoring)
- **Defense contractors**: Secure autonomous swarm systems
- **Hobbyists**: DIY drone swarm projects and educational purposes
- **Embedded engineers**: Real-time systems on STM32/ESP32/nRF52

### What drones are supported?

The library is **hardware-agnostic** and works with any drone platform via integration:
- **Flight controllers**: PX4, ArduPilot (via MAVLink)
- **Microcontrollers**: STM32F4/F7/H7, ESP32, nRF52, RISC-V
- **Companion computers**: Raspberry Pi, Jetson Nano, ODROID
- **Simulators**: Gazebo, AirSim, JMAVSim

See [Hardware Integration Guide](hardware-stm32.md) for details.

### Is this production-ready?

**Current status: Beta (v0.1.0)**

✅ **Production-ready components**:
- Core swarm algorithms (100% test coverage)
- Cryptography (audited primitives from RustCrypto)
- Formation control (tested with 100+ drone simulations)
- Network stack (stress-tested with million-message runs)

⚠️ **Needs more testing**:
- Real-world hardware deployment (limited field testing)
- Long-duration missions (>24 hours)
- Adversarial scenarios (security hardening in progress)

**Recommendation**: Use for research and prototypes now; production after v1.0 (Q3 2025)

---

## Technical Questions

### What's the minimum hardware requirement?

**Embedded (no_std)**:
- CPU: ARM Cortex-M4+ @ 80 MHz
- RAM: 64 KB minimum, 128 KB recommended
- Flash: 256 KB
- Network: WiFi or LoRa radio

**Desktop/Server**:
- Any x86_64 or ARM64 CPU
- RAM: 512 MB
- OS: Linux, macOS, Windows

### How many drones can one swarm support?

**Theoretical limits**:
- **WiFi mesh (2.4 GHz)**: 200 drones per swarm
- **LoRa long-range**: 500+ drones with hierarchical clustering
- **5 GHz WiFi**: 300 drones

**Practical recommendations**:
- **10-50 drones**: Excellent performance, <10 ms latency
- **50-100 drones**: Good performance, <20 ms latency
- **100-200 drones**: Acceptable performance, <50 ms latency
- **200+ drones**: Requires hierarchical sub-swarms

Memory usage: ~12-24 KB per drone

### Is GPS required?

**No**, but it helps. The system supports multiple positioning methods:

1. **GPS/GNSS**: Outdoor absolute positioning
2. **Visual odometry**: Camera-based SLAM for indoor/GPS-denied
3. **UWB (Ultra-Wideband)**: High-precision indoor positioning
4. **IMU dead reckoning**: Backup for short-term GPS loss
5. **Relative positioning**: Formation control without absolute coordinates

See `Position` struct documentation for coordinate system details.

### What's the communication range?

**WiFi (IEEE 802.11n/ac)**:
- Line-of-sight: 500-1000m
- Urban/obstacles: 100-300m
- Data rate: 10-100 Mbps

**LoRa (868/915 MHz)**:
- Line-of-sight: 10-15 km
- Urban: 2-5 km
- Data rate: 0.3-50 kbps (low bandwidth, long range)

**Mesh routing** extends range through multi-hop forwarding.

### How secure is the encryption?

**Enterprise-grade security**:
- **Encryption**: ChaCha20-Poly1305 AEAD (256-bit keys)
- **Signatures**: Ed25519 (128-bit security)
- **Key exchange**: ECDH with Curve25519
- **Forward secrecy**: Session keys rotated every 24 hours

**Threat model**:
- ✅ Protects against: Eavesdropping, message tampering, replay attacks
- ✅ Resistant to: Brute force, side-channel attacks (constant-time crypto)
- ⚠️ Vulnerable to: Quantum computers (post-quantum upgrade planned Q3 2025)

**Audits**: Uses RustCrypto primitives (community-audited, FIPS-approved algorithms)

### Can I use this commercially?

**Yes!** Licensed under Apache 2.0:
- ✅ Use in commercial products
- ✅ Modify and distribute
- ✅ Patent grant included
- ✅ No attribution in binary required (but appreciated!)

**Requirements**:
- Include LICENSE and NOTICE files in source distributions
- State changes made to the code

No royalties, no fees, no restrictions.

---

## Build and Compilation

### Build fails with "cannot find crate `alloc`"

**Cause**: You're trying to build a no_std example without a target specified.

**Solution**:
```bash
# For embedded (e.g., STM32):
cargo build --example stm32_deployment --target thumbv7em-none-eabihf --release

# For desktop (std):
cargo build --example simple_swarm --features std
```

### "error: requires `std` feature" when building for embedded

**Cause**: Some dependencies require `std` but you're building no_std.

**Solution**: Disable default features and enable `no_std` feature:
```toml
[dependencies]
swarm-manager = { version = "0.1", default-features = false, features = ["no_std"] }
```

### Linker error: "undefined reference to `_Unwind_Resume`"

**Cause**: Missing panic handler or runtime for no_std.

**Solution**: Add panic handler:
```rust
#![no_std]
use panic_halt as _;  // Simple panic handler

// Or for production:
use panic_probe as _;  // Logging panic handler
```

### Slow compile times

**Solutions**:

1. **Use sccache** (shared compilation cache):
```bash
cargo install sccache
export RUSTC_WRAPPER=sccache
```

2. **Parallel codegen**:
```toml
# .cargo/config.toml
[build]
jobs = 8  # Use 8 parallel jobs
```

3. **Link with LLD** (faster linker):
```toml
# .cargo/config.toml
[target.x86_64-unknown-linux-gnu]
linker = "clang"
rustflags = ["-C", "link-arg=-fuse-ld=lld"]
```

---

## Runtime Errors

### `EncryptionError::NonceExhaustion`

**Cause**: Encrypted >2^32 messages with same key (nonce wraparound).

**Solution**: Automatically handled by key rotation. If you see this:
```rust
// Manual key rotation (normally automatic):
crypto.rotate_keys()?;
```

**Prevention**: Enable `auto_key_rotation` feature (enabled by default).

### `SwarmError::ConsensusTimeout`

**Cause**: Raft consensus couldn't achieve quorum (network partition or >50% nodes offline).

**Solution**:
```rust
// Check cluster health:
if consensus.get_cluster_health() < 0.5 {
    // Less than half nodes available
    consensus.trigger_election()?;
}

// Increase timeout for high-latency networks:
let config = RaftConfig {
    election_timeout: Duration::from_secs(5),  // Default: 3s
    ..Default::default()
};
```

### `PathPlanningError::NoValidPath`

**Cause**: No collision-free path found (too many obstacles or impossible constraints).

**Solution**:
```rust
// 1. Increase PSO iterations:
optimizer.set_max_iterations(200)?;  // Default: 100

// 2. Relax constraints:
optimizer.set_min_clearance(5.0)?;  // Reduce from 10.0m

// 3. Use ACO instead (better for complex obstacles):
let aco_planner = ACOPathPlanner::new(start, goal, aco_config)?;
let path = aco_planner.optimize()?;
```

### Memory allocation failure on embedded

**Cause**: Heap exhaustion (too many simultaneous neighbors or large messages).

**Solution**:
```rust
// Use heapless collections with bounded capacity:
use heapless::Vec;
const MAX_NEIGHBORS: usize = 10;  // Limit neighbors

let mut network = MeshNetwork::new_with_capacity::<MAX_NEIGHBORS>(drone_id);
```

**Memory tuning**:
- Reduce `MAX_NEIGHBORS` (default: 20)
- Limit `MAX_MESSAGE_SIZE` (default: 4096 bytes)
- Disable unused features (saves ~20-50 KB)

---

## Network Issues

### Drones can't discover each other

**Checklist**:

1. **Same network?**
   ```rust
   // All drones must use same network ID:
   let config = NetworkConfig {
       network_id: 0x1234,  // Must match
       ..Default::default()
   };
   ```

2. **Firewall blocking?**
   ```bash
   # Allow UDP port 8080 (default):
   sudo ufw allow 8080/udp
   ```

3. **Multicast working?**
   ```bash
   # Test multicast (send on one device, receive on another):
   # Sender:
   echo "test" | socat - UDP-DATAGRAM:224.0.0.251:8080,broadcast

   # Receiver:
   socat UDP-RECV:8080,ip-add-membership=224.0.0.251:0.0.0.0 -
   ```

4. **Hello messages enabled?**
   ```rust
   network.start_discovery()?;  // Sends periodic Hello
   ```

### High packet loss (>5%)

**Causes and solutions**:

1. **WiFi interference**:
   ```rust
   // Switch to less-congested channel:
   let config = NetworkConfig {
       wifi_channel: 11,  // Try 1, 6, or 11
       ..Default::default()
   };
   ```

2. **Too many drones on one channel**:
   - Use 5 GHz if available (more channels)
   - Split swarm into sub-swarms

3. **Distance too far**:
   ```rust
   // Enable multi-hop routing:
   let config = NetworkConfig {
       max_hops: 5,  // Default: 3
       ..Default::default()
   };
   ```

4. **Message rate too high**:
   ```rust
   // Reduce broadcast frequency:
   let config = NetworkConfig {
       hello_interval: Duration::from_secs(2),  // Default: 1s
       ..Default::default()
   };
   ```

### "Connection refused" errors

**Cause**: Peer not listening or incorrect address.

**Debug**:
```rust
// Check if peer is reachable:
if let Some(neighbor) = network.get_neighbor(peer_id) {
    println!("Peer address: {:?}", neighbor.address);
    println!("Last seen: {:?}", neighbor.last_hello);
} else {
    println!("Peer not in neighbor table!");
}
```

---

## Algorithm Issues

### PSO not converging

**Tuning parameters**:

```rust
let config = PSOConfig {
    num_particles: 50,     // Increase from 30
    max_iterations: 200,   // Increase from 100
    inertia_weight: 0.7,   // Decrease from 0.9 (more exploration)
    cognitive: 1.5,        // Personal best weight
    social: 1.5,           // Swarm best weight
    ..Default::default()
};
```

**Common issues**:
- **Premature convergence**: Increase `num_particles` or `inertia_weight`
- **Too slow**: Decrease `max_iterations` or use `topology: Ring` (faster than `Global`)
- **Stuck in local minimum**: Add `mutation_rate: 0.1` for random exploration

### ACO finds suboptimal paths

**Parameter tuning**:

```rust
let config = ACOConfig {
    num_ants: 100,         // Increase from 50
    alpha: 1.5,            // Increase pheromone importance
    beta: 2.5,             // Increase heuristic importance
    evaporation: 0.05,     // Decrease (slower pheromone decay)
    variant: ACOVariant::MaxMinAntSystem,  // Better convergence
    ..Default::default()
};
```

**Debugging**:
```rust
// Log best path cost per iteration:
for iteration in 0..100 {
    aco.step()?;
    println!("Iteration {}: Best cost = {:.2}", iteration, aco.best_cost());
}
```

### GWO stuck in local optimum

**Solution**: Use hybrid mode

```rust
let config = GWOConfig {
    variant: GWOVariant::Hybrid,  // GWO + PSO
    adaptive: true,                // Adaptive parameters
    ..Default::default()
};
```

---

## Performance Issues

### High latency (>100 ms)

**Diagnosis**:

```rust
// Add timing measurements:
let start = Instant::now();
let velocity = swarm.compute_control_velocity(dt);
let elapsed = start.elapsed();
println!("Control loop: {:?}", elapsed);
```

**Common bottlenecks**:
1. **Cryptography**: Batch encrypt multiple messages
2. **Network**: Use UDP instead of TCP for real-time data
3. **Path planning**: Run optimization less frequently (1 Hz instead of 10 Hz)

**Optimization**:
```rust
// Cache formation positions:
let target = swarm.get_target_position_cached()?;  // No recomputation
```

### High CPU usage

**Solutions**:

1. **Reduce control loop rate**:
   ```rust
   loop {
       swarm.update(dt);
       thread::sleep(Duration::from_millis(10));  // 100 Hz instead of 1000 Hz
   }
   ```

2. **Disable unused features**:
   ```toml
   [dependencies]
   swarm-manager = { version = "0.1", default-features = false, features = ["swarm", "pso"] }
   ```

3. **Use release mode**:
   ```bash
   cargo build --release  # 10-100x faster than debug
   ```

### Battery drains too fast

**Power optimization**:

```rust
// 1. Reduce network traffic:
let config = NetworkConfig {
    hello_interval: Duration::from_secs(5),  // Less frequent
    ..Default::default()
};

// 2. Sleep when idle:
if !swarm.has_active_mission() {
    swarm.enter_low_power_mode()?;
}

// 3. Optimize path for energy:
let fitness = |path: &[Position]| {
    let distance = calculate_path_length(path);
    let energy = calculate_energy_cost(path);
    distance * 0.3 + energy * 0.7  // Weight energy more
};
```

---

## Testing and Debugging

### How do I test without real drones?

**Simulation options**:

1. **Unit tests** (mock Position):
   ```rust
   #[test]
   fn test_formation_circle() {
       let drone_id = DroneId::new(1);
       let pos = Position { x: 0.0, y: 0.0, z: 10.0 };
       let swarm = SwarmController::new(drone_id, pos);
       // ... assertions
   }
   ```

2. **Gazebo simulator**:
   ```bash
   roslaunch drone_swarm_gazebo swarm_50.launch
   cargo run --example gazebo_interface
   ```

3. **SITL (Software In The Loop)**:
   ```bash
   # Start PX4 SITL instances:
   ./start_sitl_swarm.sh 10  # 10 drones
   cargo run --example mavlink_swarm
   ```

### Enable debug logging

```rust
// Cargo.toml:
[dependencies]
env_logger = "0.10"
log = "0.4"

// main.rs:
use log::{info, debug, warn, error};

fn main() {
    env_logger::init();
    info!("Starting swarm...");
    debug!("Drone ID: {:?}", drone_id);
}
```

```bash
# Run with logging:
RUST_LOG=debug cargo run
RUST_LOG=drone_swarm_system=trace cargo run  # Very verbose
```

### Capture network traffic

```bash
# Wireshark/tcpdump:
sudo tcpdump -i wlan0 -w swarm_capture.pcap port 8080

# Analyze in Wireshark:
wireshark swarm_capture.pcap
```

---

## Hardware-Specific Issues

### STM32: "panic: out of memory"

**Cause**: Heap too small in linker script.

**Solution**: Edit `memory.x`:
```
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K  /* Increase from 128K */
}
```

### ESP32: WiFi connection drops

**Cause**: Power supply instability or WiFi power saving.

**Solution**:
```c
// Disable WiFi power saving:
esp_wifi_set_ps(WIFI_PS_NONE);

// Increase TX power:
esp_wifi_set_max_tx_power(84);  // Max 84 (21 dBm)
```

### Raspberry Pi: I2C errors

**Cause**: I2C clock stretching issues.

**Solution**:
```bash
# /boot/config.txt:
dtparam=i2c_arm=on,i2c_arm_baudrate=100000

# Increase timeout:
echo 1000 > /sys/module/i2c_bcm2835/parameters/debug
```

---

## Contributing and Support

### How do I report a bug?

1. **Check existing issues**: [GitHub Issues](https://github.com/mahii6991/swarm-manager/issues)
2. **Provide details**:
   - Rust version: `rustc --version`
   - OS and architecture
   - Minimal reproducible example
   - Error messages and stack traces
3. **Submit issue**: Use bug report template

### How can I contribute?

- Code contributions (submit PRs)
- Documentation improvements
- Bug reports and feature requests
- Hardware testing and benchmarks

### Where can I get help?

- **GitHub Discussions**: [Community forum](https://github.com/mahii6991/swarm-manager/discussions)
- **GitHub Issues**: [Report bugs](https://github.com/mahii6991/swarm-manager/issues)
- **Email**: m.s.rajpoot20@gmail.com (critical security issues only)

---

## Licensing and Legal

### Can I use this in a closed-source product?

**Yes!** Apache 2.0 allows proprietary use. You must:
- Include LICENSE and NOTICE files in source distributions
- Provide attribution if distributing source code

You do **NOT** need to:
- Open-source your modifications
- Pay royalties or fees
- Attribute in binary/commercial products

### Are there export restrictions?

**Potentially**, depending on your country and use case:

- **Encryption**: ChaCha20 is generally export-controlled (check local laws)
- **Drone technology**: Some countries restrict autonomous systems
- **Dual-use**: Swarm tech may be subject to ITAR/EAR in USA

**Recommendation**: Consult legal counsel for commercial/defense applications.

### What if I find a security vulnerability?

**DO NOT** open a public issue. Email: security@example.com

We follow **responsible disclosure**:
1. Report sent to security team
2. We acknowledge within 48 hours
3. Fix developed and tested
4. Coordinated disclosure (CVE assigned if applicable)
5. Public announcement after fix released

**Bug bounty**: Not available yet (planned for v1.0).

---

## Still Have Questions?

- [Join GitHub Discussions](https://github.com/mahii6991/swarm-manager/discussions)
- [Read the Full Documentation](index.md)
- [Check Examples](examples.md)
- [Review API Reference](api-reference.md)

---

**This FAQ is continuously updated based on community questions. Last update: 2025-11-30**
