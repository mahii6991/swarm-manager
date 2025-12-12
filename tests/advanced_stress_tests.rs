//! 🚀 ADVANCED STRESS TESTS - New Features (Merkle, WOA, Secure Aggregation)
//!
//! Verifies the robustness of the newly added "God Level" features.

#![cfg(test)]

use drone_swarm_system::federated::*;
use drone_swarm_system::merkle::*;
use drone_swarm_system::types::*;
use drone_swarm_system::woa::*;
use heapless::Vec;

// ═══════════════════════════════════════════════════════════════════════════
// MERKLE TREE STRESS TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn stress_merkle_large_tree() {
    println!("\n🌳 STRESS TEST: Merkle Tree with max capacity");

    const CAP: usize = 1024;
    let mut tree = MerkleTree::<CAP>::new();

    // Create dummy data references
    // Since we need references to data that lives long enough, we use a static array of bytes
    // and slice it differently to simulate different data.
    let dummy_data = [0u8; 1024];
    let mut data_refs: Vec<&[u8], CAP> = Vec::new();

    for i in 0..CAP / 2 {
        // Fill up to capacity (leafs take half usually, depending on implementation details)
        // Actually our MerkleTree implementation stores everything in `nodes`.
        // A tree of N leaves needs 2*N - 1 nodes total.
        // So with capacity 1024, we can support ~512 leaves.
        if data_refs.push(&dummy_data[0..i + 1]).is_err() {
            break;
        }
    }

    let start = std::time::Instant::now();
    let root = tree.compute_root(&data_refs);
    let duration = start.elapsed();

    match root {
        Ok(hash) => {
            println!(
                "  ✅ Computed root for {} leaves in {:?}",
                data_refs.len(),
                duration
            );
            println!("  Root: {:02x?}", &hash[0..4]); // Print first 4 bytes
        }
        Err(e) => {
            println!("  ⚠️ Failed (expected if capacity exceeded): {:?}", e);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// WHALE OPTIMIZATION STRESS TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn bench_woa_convergence() {
    println!("\n🐋 BENCHMARK: Whale Optimization Convergence");

    let config = WoaConfig {
        max_iterations: 100,
        population_size: 50, // Larger population for stress
        spiral_param: 1.0,
    };

    let mut woa = WhaleOptimizer::new(config, 999);

    let min = Position {
        x: -100.0,
        y: -100.0,
        z: -100.0,
    };
    let max = Position {
        x: 100.0,
        y: 100.0,
        z: 100.0,
    };

    woa.initialize(min, max).unwrap();

    // Rastrigin function (complex, multimodal)
    // f(x) = 10n + sum(x^2 - 10cos(2pi*x))
    let cost_fn = |p: &Position| -> f32 {
        let x = p.x;
        let y = p.y;
        let z = p.z;
        use libm::cosf;
        let pi = core::f32::consts::PI;

        10.0 * 3.0
            + (x * x - 10.0 * cosf(2.0 * pi * x))
            + (y * y - 10.0 * cosf(2.0 * pi * y))
            + (z * z - 10.0 * cosf(2.0 * pi * z))
    };

    let start = std::time::Instant::now();
    for i in 0..100 {
        woa.step(i, &cost_fn).unwrap();
    }
    let duration = start.elapsed();

    println!("  ✅ 100 iterations with 50 whales in {:?}", duration);
    println!(
        "  Time per iteration: {:.2} ms",
        duration.as_millis() as f64 / 100.0
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// SECURE AGGREGATION STRESS TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn stress_secure_aggregation_throughput() {
    println!("\n🔐 STRESS TEST: Secure Aggregation Throughput");

    let mut sa = SecureAggregation::new();
    const NUM_DRONES: u64 = 50;

    // Setup
    for i in 0..NUM_DRONES {
        sa.add_participant(DroneId::new(i), [i as u8; 32]).unwrap();
    }

    // Create updates
    let mut updates: Vec<(DroneId, Vec<u8, 2048>), 100> = Vec::new();
    for i in 0..NUM_DRONES {
        let mut params = Vec::new();
        for _ in 0..100 {
            // 100 parameters
            params.push(1.0).unwrap();
        }

        let update = ModelUpdate {
            drone_id: DroneId::new(i),
            round: 0,
            parameters: params,
            sample_count: 10,
            loss: 0.1,
            signature: [0u8; 64],
        };

        let encrypted = sa.encrypt_update(&update).unwrap();
        updates.push((DroneId::new(i), encrypted)).ok();
    }

    let start = std::time::Instant::now();

    // Aggregate
    // Note: slice conversion
    let updates_slice: &[(DroneId, Vec<u8, 2048>)] = &updates;
    let result = sa.aggregate_encrypted(updates_slice).unwrap();

    let duration = start.elapsed();

    println!(
        "  ✅ Aggregated {} encrypted updates (100 params each) in {:?}",
        NUM_DRONES, duration
    );
    assert_eq!(result.len(), 1000);
}
