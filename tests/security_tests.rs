//! Comprehensive tests for the security module
//!
//! Tests SecurityMonitor, RateLimiter, IntrusionDetectionSystem, AccessControl, and AuditLog

use drone_swarm_system::security::*;
use drone_swarm_system::types::*;

// ═══════════════════════════════════════════════════════════════════════════
// SecurityMonitor Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod security_monitor_tests {
    use super::*;

    #[test]
    fn test_new() {
        let monitor = SecurityMonitor::new();
        assert!(!monitor.is_banned(DroneId::new(1)));
    }

    #[test]
    fn test_default() {
        let monitor = SecurityMonitor::default();
        assert!(!monitor.is_banned(DroneId::new(1)));
    }

    #[test]
    fn test_record_auth_failure_single() {
        let mut monitor = SecurityMonitor::new();
        let drone = DroneId::new(1);

        let result = monitor.record_auth_failure(drone);
        assert!(result.is_ok());
        assert!(!monitor.is_banned(drone));
    }

    #[test]
    fn test_record_auth_failure_threshold() {
        let mut monitor = SecurityMonitor::new();
        let drone = DroneId::new(1);

        // Should allow up to threshold failures
        for _ in 0..5 {
            assert!(monitor.record_auth_failure(drone).is_ok());
        }
        assert!(!monitor.is_banned(drone));

        // 6th failure should trigger ban
        let result = monitor.record_auth_failure(drone);
        assert!(result.is_err());
        assert!(monitor.is_banned(drone));
    }

    #[test]
    fn test_clear_failures() {
        let mut monitor = SecurityMonitor::new();
        let drone = DroneId::new(1);

        // Record some failures
        for _ in 0..3 {
            monitor.record_auth_failure(drone).ok();
        }

        // Clear failures
        monitor.clear_failures(drone);

        // Should be able to fail again without immediate ban
        for _ in 0..5 {
            assert!(monitor.record_auth_failure(drone).is_ok());
        }
    }

    #[test]
    fn test_multiple_drones() {
        let mut monitor = SecurityMonitor::new();
        let drone1 = DroneId::new(1);
        let drone2 = DroneId::new(2);

        // Ban drone1
        for _ in 0..6 {
            monitor.record_auth_failure(drone1).ok();
        }

        assert!(monitor.is_banned(drone1));
        assert!(!monitor.is_banned(drone2));
    }

    #[test]
    fn test_analyze_message_normal() {
        let mut monitor = SecurityMonitor::new();
        let drone = DroneId::new(1);
        let message = b"Normal message content";

        let result = monitor.analyze_message(drone, message);
        assert!(result.is_ok());
    }

    #[test]
    fn test_analyze_message_large() {
        let mut monitor = SecurityMonitor::new();
        let drone = DroneId::new(1);
        let large_message = vec![0u8; 3000]; // > 2048 bytes

        // Large messages are flagged as suspicious
        let result = monitor.analyze_message(drone, &large_message);
        assert!(result.is_ok()); // First offense is allowed
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// RateLimiter Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod rate_limiter_tests {
    use super::*;

    #[test]
    fn test_new() {
        let _limiter = RateLimiter::new(100, 1000);
        // Construction succeeded without panic
    }

    #[test]
    fn test_within_limit() {
        let mut limiter = RateLimiter::new(10, 1000);
        let drone = DroneId::new(1);

        for _ in 0..10 {
            assert!(limiter.check(drone).is_ok());
        }
    }

    #[test]
    fn test_exceeds_limit() {
        let mut limiter = RateLimiter::new(5, 1000);
        let drone = DroneId::new(1);

        // Use up the limit
        for _ in 0..5 {
            assert!(limiter.check(drone).is_ok());
        }

        // Next should fail
        assert!(limiter.check(drone).is_err());
    }

    #[test]
    fn test_multiple_drones_independent() {
        let mut limiter = RateLimiter::new(5, 1000);
        let drone1 = DroneId::new(1);
        let drone2 = DroneId::new(2);

        // Use up drone1's limit
        for _ in 0..5 {
            limiter.check(drone1).ok();
        }
        assert!(limiter.check(drone1).is_err());

        // drone2 should still have limit
        for _ in 0..5 {
            assert!(limiter.check(drone2).is_ok());
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// IntrusionDetectionSystem Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod ids_tests {
    use super::*;

    #[test]
    fn test_new() {
        let ids = IntrusionDetectionSystem::new();
        assert!(!ids.is_banned(DroneId::new(1)));
    }

    #[test]
    fn test_default() {
        let ids = IntrusionDetectionSystem::default();
        assert!(!ids.is_banned(DroneId::new(1)));
    }

    #[test]
    fn test_ban_drone() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        assert!(!ids.is_banned(drone));
        ids.ban_drone(drone).unwrap();
        assert!(ids.is_banned(drone));
    }

    #[test]
    fn test_ban_drone_idempotent() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        // Ban multiple times should not error
        ids.ban_drone(drone).unwrap();
        ids.ban_drone(drone).unwrap();
        assert!(ids.is_banned(drone));
    }

    #[test]
    fn test_unban_drone() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        ids.ban_drone(drone).unwrap();
        assert!(ids.is_banned(drone));

        ids.unban_drone(drone).unwrap();
        assert!(!ids.is_banned(drone));
    }

    #[test]
    fn test_unban_not_banned() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        // Should not error
        assert!(ids.unban_drone(drone).is_ok());
    }

    #[test]
    fn test_analyze_normal_message() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);
        let message = b"This is a normal message";

        assert!(ids.analyze(drone, message).is_ok());
    }

    #[test]
    fn test_analyze_repetitive_pattern() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        // Create highly repetitive message
        let pattern = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11];
        let mut message = Vec::new();
        for _ in 0..50 {
            message.extend_from_slice(&pattern);
        }

        // Multiple suspicious activities should eventually lead to ban
        for _ in 0..15 {
            ids.analyze(drone, &message).ok();
        }
        // After many suspicious activities, should be banned
        assert!(ids.is_banned(drone));
    }

    #[test]
    fn test_analyze_small_message() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);
        let message = b"small";

        // Small messages should pass
        assert!(ids.analyze(drone, message).is_ok());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// AccessControl Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod access_control_tests {
    use super::*;

    #[test]
    fn test_new() {
        let ac = AccessControl::new();
        let drone = DroneId::new(1);
        assert!(ac.check_permission(drone, SecurityLevel::Public).is_err());
    }

    #[test]
    fn test_default() {
        let ac = AccessControl::default();
        let drone = DroneId::new(1);
        assert!(ac.check_permission(drone, SecurityLevel::Public).is_err());
    }

    #[test]
    fn test_authorize() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        ac.authorize(drone, SecurityLevel::Internal).unwrap();
        assert!(ac.check_permission(drone, SecurityLevel::Internal).is_ok());
    }

    #[test]
    fn test_permission_hierarchy() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        ac.authorize(drone, SecurityLevel::Critical).unwrap();

        // Should have access to all levels
        assert!(ac.check_permission(drone, SecurityLevel::Public).is_ok());
        assert!(ac.check_permission(drone, SecurityLevel::Internal).is_ok());
        assert!(ac.check_permission(drone, SecurityLevel::Sensitive).is_ok());
        assert!(ac.check_permission(drone, SecurityLevel::Critical).is_ok());
    }

    #[test]
    fn test_permission_denied() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        ac.authorize(drone, SecurityLevel::Public).unwrap();

        // Should not have access to higher levels
        assert!(ac.check_permission(drone, SecurityLevel::Internal).is_err());
        assert!(ac
            .check_permission(drone, SecurityLevel::Sensitive)
            .is_err());
        assert!(ac.check_permission(drone, SecurityLevel::Critical).is_err());
    }

    #[test]
    fn test_revoke() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        ac.authorize(drone, SecurityLevel::Critical).unwrap();
        assert!(ac.check_permission(drone, SecurityLevel::Critical).is_ok());

        ac.revoke(drone).unwrap();
        assert!(ac.check_permission(drone, SecurityLevel::Public).is_err());
    }

    #[test]
    fn test_revoke_not_authorized() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        assert!(ac.revoke(drone).is_err());
    }

    #[test]
    fn test_multiple_drones() {
        let mut ac = AccessControl::new();
        let drone1 = DroneId::new(1);
        let drone2 = DroneId::new(2);

        ac.authorize(drone1, SecurityLevel::Critical).unwrap();
        ac.authorize(drone2, SecurityLevel::Public).unwrap();

        assert!(ac.check_permission(drone1, SecurityLevel::Critical).is_ok());
        assert!(ac
            .check_permission(drone2, SecurityLevel::Critical)
            .is_err());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// AuditLog Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod audit_log_tests {
    use super::*;

    #[test]
    fn test_new() {
        let log = AuditLog::new();
        assert_eq!(log.get_recent(10).len(), 0);
    }

    #[test]
    fn test_default() {
        let log = AuditLog::default();
        assert_eq!(log.get_recent(10).len(), 0);
    }

    #[test]
    fn test_log_event() {
        let mut log = AuditLog::new();
        let drone = DroneId::new(1);

        log.log(drone, AuditEvent::AuthSuccess).unwrap();
        assert_eq!(log.get_recent(10).len(), 1);
    }

    #[test]
    fn test_log_multiple_events() {
        let mut log = AuditLog::new();
        let drone = DroneId::new(1);

        log.log(drone, AuditEvent::AuthSuccess).unwrap();
        log.log(drone, AuditEvent::MessageSent).unwrap();
        log.log(drone, AuditEvent::MessageReceived).unwrap();

        assert_eq!(log.get_recent(10).len(), 3);
    }

    #[test]
    fn test_get_recent_limited() {
        let mut log = AuditLog::new();
        let drone = DroneId::new(1);

        for _ in 0..10 {
            log.log(drone, AuditEvent::AuthSuccess).unwrap();
        }

        assert_eq!(log.get_recent(5).len(), 5);
    }

    #[test]
    fn test_log_all_event_types() {
        let mut log = AuditLog::new();
        let drone = DroneId::new(1);

        log.log(drone, AuditEvent::AuthSuccess).unwrap();
        log.log(drone, AuditEvent::AuthFailure).unwrap();
        log.log(drone, AuditEvent::MessageSent).unwrap();
        log.log(drone, AuditEvent::MessageReceived).unwrap();
        log.log(drone, AuditEvent::ConsensusReached).unwrap();
        log.log(drone, AuditEvent::DroneJoined).unwrap();
        log.log(drone, AuditEvent::DroneLeft).unwrap();
        log.log(drone, AuditEvent::SecurityViolation).unwrap();
        log.log(drone, AuditEvent::SystemError).unwrap();

        assert_eq!(log.get_recent(20).len(), 9);
    }

    #[test]
    fn test_audit_entry_fields() {
        let mut log = AuditLog::new();
        let drone = DroneId::new(42);

        log.log(drone, AuditEvent::AuthSuccess).unwrap();

        let entries = log.get_recent(1);
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].source, drone);
        // Timestamp should be set (just check it's not 0 in test env)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_security_workflow() {
        let mut monitor = SecurityMonitor::new();
        let mut ac = AccessControl::new();
        let mut log = AuditLog::new();

        let drone = DroneId::new(1);

        // Authorize drone
        ac.authorize(drone, SecurityLevel::Internal).unwrap();
        log.log(drone, AuditEvent::DroneJoined).unwrap();

        // Check rate limit (should pass)
        assert!(monitor.check_rate_limit(drone).is_ok());

        // Analyze message (should pass)
        assert!(monitor.analyze_message(drone, b"Hello swarm").is_ok());
        log.log(drone, AuditEvent::MessageReceived).unwrap();

        // Check permission (should pass)
        assert!(ac.check_permission(drone, SecurityLevel::Internal).is_ok());

        // Log success
        log.log(drone, AuditEvent::AuthSuccess).unwrap();

        assert_eq!(log.get_recent(10).len(), 3);
    }

    #[test]
    fn test_attack_detection_workflow() {
        let mut monitor = SecurityMonitor::new();
        let mut log = AuditLog::new();

        let attacker = DroneId::new(666);

        // Attacker sends many failed auths
        for _ in 0..6 {
            if monitor.record_auth_failure(attacker).is_err() {
                log.log(attacker, AuditEvent::SecurityViolation).unwrap();
            }
        }

        // Attacker should be banned
        assert!(monitor.is_banned(attacker));
    }
}
