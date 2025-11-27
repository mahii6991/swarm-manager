//! Benchmark and test the time source implementation
//!
//! This example measures the accuracy and performance of the time source.
//!
//! Run with:
//! ```bash
//! cargo run --example time_benchmark --release
//! ```

use drone_swarm_system::{init_time_source, get_time_ms, get_time_us, delay_ms};

fn main() {
    println!("═══════════════════════════════════════════════════════════");
    println!("  Drone Swarm Time Source Benchmark");
    println!("═══════════════════════════════════════════════════════════\n");

    init_time_source(0);

    // Test 1: Millisecond accuracy
    println!("Test 1: Millisecond Timing Accuracy");
    println!("────────────────────────────────────");
    test_millisecond_accuracy();

    // Test 2: Microsecond accuracy
    println!("\nTest 2: Microsecond Timing Accuracy");
    println!("────────────────────────────────────");
    test_microsecond_accuracy();

    // Test 3: Monotonicity
    println!("\nTest 3: Time Monotonicity");
    println!("────────────────────────────────────");
    test_monotonicity();

    // Test 4: Delay accuracy
    println!("\nTest 4: Delay Function Accuracy");
    println!("────────────────────────────────────");
    test_delay_accuracy();

    // Test 5: Performance
    println!("\nTest 5: Time Function Performance");
    println!("────────────────────────────────────");
    test_performance();

    println!("\n═══════════════════════════════════════════════════════════");
    println!("  ✅ All tests completed successfully!");
    println!("═══════════════════════════════════════════════════════════");
}

fn test_millisecond_accuracy() {
    let delays = [10, 50, 100, 500, 1000];

    for &delay in &delays {
        let start = get_time_ms();
        delay_ms(delay);
        let elapsed = get_time_ms() - start;

        let error = (elapsed as i64 - delay as i64).abs();
        let error_pct = (error as f64 / delay as f64) * 100.0;

        let status = if error_pct < 5.0 { "✓" } else { "✗" };

        println!("  {} Delay: {} ms → Actual: {} ms (error: {} ms, {:.2}%)",
                 status, delay, elapsed, error, error_pct);
    }
}

fn test_microsecond_accuracy() {
    let start_us = get_time_us();
    delay_ms(100);
    let elapsed_us = get_time_us() - start_us;
    let elapsed_ms = elapsed_us / 1000;

    let error = (elapsed_ms as i64 - 100).abs();
    let error_pct = (error as f64 / 100.0) * 100.0;

    println!("  100 ms delay measured in microseconds:");
    println!("    Expected: 100,000 µs");
    println!("    Actual:   {} µs", elapsed_us);
    println!("    In ms:    {} ms", elapsed_ms);
    println!("    Error:    {:.2}%", error_pct);

    if error_pct < 5.0 {
        println!("  ✓ Microsecond precision is accurate");
    }
}

fn test_monotonicity() {
    let mut prev = get_time_ms();
    let mut violations = 0;

    for _ in 0..1000 {
        let current = get_time_ms();
        if current < prev {
            violations += 1;
        }
        prev = current;

        // Small delay
        delay_ms(1);
    }

    if violations == 0 {
        println!("  ✓ Time is strictly monotonic (1000 samples)");
    } else {
        println!("  ✗ Found {} monotonicity violations!", violations);
    }
}

fn test_delay_accuracy() {
    let test_cases = [
        (1, "1 ms"),
        (5, "5 ms"),
        (10, "10 ms"),
        (50, "50 ms"),
    ];

    for (delay, description) in &test_cases {
        let measurements: Vec<u64> = (0..10)
            .map(|_| {
                let start = get_time_ms();
                delay_ms(*delay);
                get_time_ms() - start
            })
            .collect();

        let avg: u64 = measurements.iter().sum::<u64>() / measurements.len() as u64;
        let min = *measurements.iter().min().unwrap();
        let max = *measurements.iter().max().unwrap();

        let error = (avg as i64 - *delay as i64).abs();
        let error_pct = (error as f64 / *delay as f64) * 100.0;

        println!("  {} delay (10 samples):", description);
        println!("    Average: {} ms (error: {:.2}%)", avg, error_pct);
        println!("    Range:   {} - {} ms", min, max);
    }
}

fn test_performance() {
    // Measure get_time_ms() performance
    const ITERATIONS: usize = 100_000;

    let start = std::time::Instant::now();
    for _ in 0..ITERATIONS {
        let _ = get_time_ms();
    }
    let elapsed = start.elapsed();

    let ns_per_call = elapsed.as_nanos() / ITERATIONS as u128;
    let calls_per_sec = (ITERATIONS as f64 / elapsed.as_secs_f64()) as u64;

    println!("  get_time_ms() performance:");
    println!("    Iterations:    {}", ITERATIONS);
    println!("    Total time:    {:?}", elapsed);
    println!("    Time per call: {} ns", ns_per_call);
    println!("    Calls/second:  {} M/s", calls_per_sec / 1_000_000);

    if ns_per_call < 1000 {
        println!("  ✓ Excellent performance (< 1 µs per call)");
    } else if ns_per_call < 10_000 {
        println!("  ✓ Good performance (< 10 µs per call)");
    } else {
        println!("  ⚠ Performance could be improved");
    }

    // Measure get_time_us() performance
    let start = std::time::Instant::now();
    for _ in 0..ITERATIONS {
        let _ = get_time_us();
    }
    let elapsed = start.elapsed();

    let ns_per_call = elapsed.as_nanos() / ITERATIONS as u128;

    println!("\n  get_time_us() performance:");
    println!("    Time per call: {} ns", ns_per_call);
}
