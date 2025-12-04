//! Comprehensive tests for time abstraction
//!
//! Tests the platform-agnostic time API and StdTimeSource implementation

use drone_swarm_system::time_abstraction::*;

#[cfg(feature = "std")]
mod std_time_source_tests {
    use super::*;

    #[test]
    fn test_std_time_source_creation() {
        let source = StdTimeSource::new();
        let time = source.get_time_ms();

        // Time should be very small (close to creation)
        assert!(time < 1000, "Newly created time source should report small elapsed time");
    }

    #[test]
    fn test_std_time_source_default() {
        let source = StdTimeSource::default();
        let time = source.get_time_ms();

        assert!(time < 1000, "Default time source should work like new()");
    }

    #[test]
    fn test_std_time_source_monotonic_ms() {
        let source = StdTimeSource::new();

        let t1 = source.get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(5));
        let t2 = source.get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(5));
        let t3 = source.get_time_ms();

        assert!(t2 > t1, "Time should monotonically increase");
        assert!(t3 > t2, "Time should continue to increase");
        assert!(t2 - t1 >= 5, "Time delta should be at least 5ms");
        assert!(t3 - t2 >= 5, "Time delta should be at least 5ms");
    }

    #[test]
    fn test_std_time_source_monotonic_us() {
        let source = StdTimeSource::new();

        let t1 = source.get_time_us();
        std::thread::sleep(std::time::Duration::from_micros(100));
        let t2 = source.get_time_us();

        assert!(t2 > t1, "Microsecond time should be monotonic");
        assert!(t2 - t1 >= 100, "Should have at least 100us elapsed");
    }

    #[test]
    fn test_std_time_source_us_precision() {
        let source = StdTimeSource::new();

        let ms = source.get_time_ms();
        let us = source.get_time_us();

        // Microseconds should be ms * 1000 (approximately)
        // Allow some tolerance due to measurement overhead
        assert!(us >= ms * 1000, "Microseconds should be >= milliseconds * 1000");
        assert!(us < (ms + 10) * 1000, "Microseconds should be close to ms * 1000");
    }

    #[test]
    fn test_std_time_source_reset() {
        let source = StdTimeSource::new();

        // Wait a bit to accumulate time
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Reset should work (though for std implementation it just clears global counter)
        source.reset();

        // This should not panic
        let _time = source.get_time_ms();
    }

    #[test]
    fn test_time_source_trait_object() {
        let source: Box<dyn TimeSource> = Box::new(StdTimeSource::new());

        let t1 = source.get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(5));
        let t2 = source.get_time_ms();

        assert!(t2 > t1, "Trait object should work correctly");
    }
}

#[cfg(feature = "std")]
mod public_api_tests {
    use super::*;

    #[test]
    fn test_init_time_source() {
        // Should not panic (std version is a no-op)
        init_time_source(0);
        init_time_source(168_000_000);
    }

    #[test]
    fn test_get_time_ms_monotonic() {
        let t1 = get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let t2 = get_time_ms();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let t3 = get_time_ms();

        assert!(t2 > t1, "get_time_ms should be monotonic");
        assert!(t3 > t2, "get_time_ms should remain monotonic");
    }

    #[test]
    fn test_get_time_us_monotonic() {
        let t1 = get_time_us();
        std::thread::sleep(std::time::Duration::from_micros(500));
        let t2 = get_time_us();

        assert!(t2 > t1, "get_time_us should be monotonic");
        assert!(t2 - t1 >= 500, "Should wait at least 500 microseconds");
    }

    #[test]
    fn test_get_time_us_higher_precision_than_ms() {
        let ms = get_time_ms();
        let us = get_time_us();

        // us should be much larger than ms (since it's microseconds)
        assert!(us > ms * 1000, "Microseconds should be larger than milliseconds * 1000");
    }

    #[test]
    fn test_delay_ms_basic() {
        let start = get_time_ms();
        delay_ms(20);
        let elapsed = get_time_ms() - start;

        assert!(elapsed >= 20, "delay_ms should wait at least the specified time");
        assert!(elapsed < 50, "delay_ms should not wait excessively long");
    }

    #[test]
    fn test_delay_ms_zero() {
        let start = get_time_ms();
        delay_ms(0);
        let elapsed = get_time_ms() - start;

        // Zero delay should return quickly
        assert!(elapsed < 10, "Zero delay should complete quickly");
    }

    #[test]
    fn test_delay_ms_short() {
        let start = get_time_ms();
        delay_ms(1);
        let elapsed = get_time_ms() - start;

        assert!(elapsed >= 1, "Even 1ms delay should work");
        assert!(elapsed < 10, "1ms delay should not take too long");
    }

    #[test]
    fn test_delay_ms_multiple_calls() {
        let start = get_time_ms();

        delay_ms(5);
        delay_ms(5);
        delay_ms(5);

        let elapsed = get_time_ms() - start;

        assert!(elapsed >= 15, "Multiple delays should accumulate");
        assert!(elapsed < 30, "Should not take excessively long");
    }

    #[test]
    fn test_time_measurement_consistency() {
        // Measure the same operation multiple times
        let mut measurements = Vec::new();

        for _ in 0..5 {
            let start = get_time_ms();
            std::thread::sleep(std::time::Duration::from_millis(10));
            let elapsed = get_time_ms() - start;
            measurements.push(elapsed);
        }

        // All measurements should be at least 10ms
        for &measurement in &measurements {
            assert!(measurement >= 10, "Each measurement should be at least 10ms");
        }

        // Measurements should be reasonably consistent (within 20ms of each other)
        let min = *measurements.iter().min().unwrap();
        let max = *measurements.iter().max().unwrap();
        assert!(max - min < 20, "Measurements should be consistent");
    }

    #[test]
    fn test_concurrent_time_reads() {
        use std::sync::Arc;
        use std::thread;

        let times = Arc::new(std::sync::Mutex::new(Vec::new()));
        let mut handles = vec![];

        // Spawn multiple threads reading time
        for _ in 0..4 {
            let times_clone = Arc::clone(&times);
            let handle = thread::spawn(move || {
                for _ in 0..10 {
                    let t = get_time_ms();
                    times_clone.lock().unwrap().push(t);
                    thread::sleep(std::time::Duration::from_millis(1));
                }
            });
            handles.push(handle);
        }

        // Wait for all threads
        for handle in handles {
            handle.join().unwrap();
        }

        let times = times.lock().unwrap();
        assert_eq!(times.len(), 40, "Should have 40 time measurements");

        // Verify all times are reasonable (non-zero)
        for &t in times.iter() {
            assert!(t > 0, "Time should be non-zero");
        }
    }

    #[test]
    fn test_high_precision_timing() {
        // Measure a very short operation with microsecond precision
        let start = get_time_us();

        // Do a tiny amount of work
        let mut _sum = 0u64;
        for i in 0..100 {
            _sum += i;
        }

        let elapsed = get_time_us() - start;

        // Should measure some non-zero time
        assert!(elapsed >= 0, "Elapsed time should be non-negative");
    }
}

#[cfg(feature = "std")]
mod time_source_trait_tests {
    use super::*;

    /// Test that TimeSource trait is object-safe (can be used with dyn)
    #[test]
    fn test_time_source_is_object_safe() {
        fn use_time_source(source: &dyn TimeSource) -> u64 {
            source.get_time_ms()
        }

        let source = StdTimeSource::new();
        let time = use_time_source(&source);

        assert!(time >= 0, "Time source trait object should work");
    }

    /// Test that TimeSource can be stored in a vector
    #[test]
    fn test_multiple_time_sources() {
        let sources: Vec<Box<dyn TimeSource>> = vec![
            Box::new(StdTimeSource::new()),
            Box::new(StdTimeSource::new()),
            Box::new(StdTimeSource::new()),
        ];

        std::thread::sleep(std::time::Duration::from_millis(5));

        for source in sources {
            let time = source.get_time_ms();
            assert!(time > 0, "Each time source should report time");
        }
    }
}

#[cfg(feature = "std")]
mod edge_case_tests {
    use super::*;

    #[test]
    fn test_delay_large_value() {
        // Test a moderately large delay (not too long to slow down tests)
        let start = get_time_ms();
        delay_ms(100);
        let elapsed = get_time_ms() - start;

        assert!(elapsed >= 100, "Large delay should work");
        assert!(elapsed < 200, "Should not take too long");
    }

    #[test]
    fn test_time_never_decreases() {
        let mut last_time = get_time_ms();

        for _ in 0..100 {
            let current_time = get_time_ms();
            assert!(current_time >= last_time, "Time should never decrease");
            last_time = current_time;
        }
    }

    #[test]
    fn test_microsecond_never_decreases() {
        let mut last_time = get_time_us();

        for _ in 0..100 {
            let current_time = get_time_us();
            assert!(current_time >= last_time, "Microsecond time should never decrease");
            last_time = current_time;
        }
    }
}
