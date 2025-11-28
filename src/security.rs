//! Advanced security features and intrusion detection

use crate::types::*;
use crate::crypto::{CryptoContext, KeyStore, NonceTracker};
use heapless::{FnvIndexMap, Vec};

/// Security monitor for detecting and preventing attacks
pub struct SecurityMonitor {
    /// Anomaly detection threshold
    anomaly_threshold: u32,
    /// Failed authentication attempts per drone
    failed_auth_attempts: FnvIndexMap<u64, u32, 128>,
    /// Message rate limiter
    rate_limiter: RateLimiter,
    /// Intrusion detection system
    ids: IntrusionDetectionSystem,
}

impl SecurityMonitor {
    /// Create a new security monitor
    pub fn new() -> Self {
        Self {
            anomaly_threshold: 5,
            failed_auth_attempts: FnvIndexMap::new(),
            rate_limiter: RateLimiter::new(100, 1000), // 100 msg/sec
            ids: IntrusionDetectionSystem::new(),
        }
    }

    /// Record failed authentication attempt
    pub fn record_auth_failure(&mut self, drone_id: DroneId) -> Result<()> {
        use heapless::Entry;
        let count = match self.failed_auth_attempts.entry(drone_id.as_u64()) {
            Entry::Occupied(mut o) => {
                *o.get_mut() = o.get().saturating_add(1);
                *o.get()
            }
            Entry::Vacant(v) => {
                v.insert(1).ok();
                1
            }
        };

        // Ban if too many failures
        if count > self.anomaly_threshold {
            self.ids.ban_drone(drone_id)?;
            return Err(SwarmError::PermissionDenied);
        }

        Ok(())
    }

    /// Check if message is allowed (rate limiting)
    pub fn check_rate_limit(&mut self, drone_id: DroneId) -> Result<()> {
        self.rate_limiter.check(drone_id)
    }

    /// Analyze message for anomalies
    pub fn analyze_message(&mut self, drone_id: DroneId, message: &[u8]) -> Result<()> {
        self.ids.analyze(drone_id, message)
    }

    /// Check if drone is banned
    pub fn is_banned(&self, drone_id: DroneId) -> bool {
        self.ids.is_banned(drone_id)
    }

    /// Clear authentication failures (on successful auth)
    pub fn clear_failures(&mut self, drone_id: DroneId) {
        self.failed_auth_attempts.remove(&drone_id.as_u64());
    }
}

impl Default for SecurityMonitor {
    fn default() -> Self {
        Self::new()
    }
}

/// Rate limiter to prevent DoS attacks
pub struct RateLimiter {
    /// Maximum messages per window
    max_messages: u32,
    /// Time window in milliseconds
    window_ms: u32,
    /// Message counts per drone
    counts: FnvIndexMap<u64, RateLimitEntry, 128>,
}

#[derive(Debug, Clone, Copy)]
struct RateLimitEntry {
    count: u32,
    window_start: u64,
}

impl RateLimiter {
    /// Create a new rate limiter
    pub fn new(max_messages: u32, window_ms: u32) -> Self {
        Self {
            max_messages,
            window_ms,
            counts: FnvIndexMap::new(),
        }
    }

    /// Check if message is within rate limit
    pub fn check(&mut self, drone_id: DroneId) -> Result<()> {
        use heapless::Entry;
        let now = Self::get_time_ms();

        let entry = match self.counts.entry(drone_id.as_u64()) {
            Entry::Occupied(o) => o.into_mut(),
            Entry::Vacant(v) => v.insert(RateLimitEntry {
                count: 0,
                window_start: now,
            }).map_err(|_| SwarmError::ResourceExhausted)?,
        };

        // Reset window if expired
        if now - entry.window_start > self.window_ms as u64 {
            entry.count = 0;
            entry.window_start = now;
        }

        // Check limit
        if entry.count >= self.max_messages {
            return Err(SwarmError::ResourceExhausted);
        }

        entry.count += 1;
        Ok(())
    }

    /// Get current time in milliseconds
    /// Note: In production, use hardware timer
    fn get_time_ms() -> u64 {
        crate::get_time_ms()
    }
}

/// Intrusion Detection System
pub struct IntrusionDetectionSystem {
    /// Banned drones
    banned: Vec<DroneId, 100>,
    /// Suspicious activity counter
    suspicious_activity: FnvIndexMap<u64, u32, 128>,
}

impl IntrusionDetectionSystem {
    /// Create a new IDS
    pub fn new() -> Self {
        Self {
            banned: Vec::new(),
            suspicious_activity: FnvIndexMap::new(),
        }
    }

    /// Ban a drone from the swarm
    pub fn ban_drone(&mut self, drone_id: DroneId) -> Result<()> {
        if !self.banned.contains(&drone_id) {
            self.banned
                .push(drone_id)
                .map_err(|_| SwarmError::ResourceExhausted)?;
        }
        Ok(())
    }

    /// Check if drone is banned
    pub fn is_banned(&self, drone_id: DroneId) -> bool {
        self.banned.contains(&drone_id)
    }

    /// Analyze message for suspicious patterns
    pub fn analyze(&mut self, drone_id: DroneId, message: &[u8]) -> Result<()> {
        let mut suspicious = false;

        // Check for abnormal message size
        if message.len() > 2048 {
            suspicious = true;
        }

        // Check for repeated patterns (possible attack)
        if Self::has_repetitive_pattern(message) {
            suspicious = true;
        }

        if suspicious {
            use heapless::Entry;
            let count = match self.suspicious_activity.entry(drone_id.as_u64()) {
                Entry::Occupied(mut o) => {
                    *o.get_mut() += 1;
                    *o.get()
                }
                Entry::Vacant(v) => {
                    v.insert(1).ok();
                    1
                }
            };

            // Ban after multiple suspicious activities
            if count > 10 {
                self.ban_drone(drone_id)?;
                return Err(SwarmError::PermissionDenied);
            }
        }

        Ok(())
    }

    /// Detect repetitive patterns (simple heuristic)
    fn has_repetitive_pattern(data: &[u8]) -> bool {
        if data.len() < 16 {
            return false;
        }

        // Check if first 8 bytes repeat
        let pattern = &data[..8];
        let mut matches = 0;

        for chunk in data.chunks(8) {
            if chunk == pattern {
                matches += 1;
            }
        }

        matches > data.len() / 16
    }

    /// Unban a drone (for testing or after verification)
    pub fn unban_drone(&mut self, drone_id: DroneId) -> Result<()> {
        if let Some(pos) = self.banned.iter().position(|&id| id == drone_id) {
            self.banned.swap_remove(pos);
        }
        Ok(())
    }
}

impl Default for IntrusionDetectionSystem {
    fn default() -> Self {
        Self::new()
    }
}

/// Access control manager
pub struct AccessControl {
    /// Authorized drones
    authorized: FnvIndexMap<u64, SecurityLevel, 128>,
}

impl AccessControl {
    /// Create a new access control manager
    pub fn new() -> Self {
        Self {
            authorized: FnvIndexMap::new(),
        }
    }

    /// Authorize a drone with specific security level
    pub fn authorize(&mut self, drone_id: DroneId, level: SecurityLevel) -> Result<()> {
        self.authorized
            .insert(drone_id.as_u64(), level)
            .map_err(|_| SwarmError::ResourceExhausted)?;
        Ok(())
    }

    /// Check if drone has required security level
    pub fn check_permission(&self, drone_id: DroneId, required: SecurityLevel) -> Result<()> {
        let level = self
            .authorized
            .get(&drone_id.as_u64())
            .ok_or(SwarmError::PermissionDenied)?;

        if *level >= required {
            Ok(())
        } else {
            Err(SwarmError::PermissionDenied)
        }
    }

    /// Revoke authorization
    pub fn revoke(&mut self, drone_id: DroneId) -> Result<()> {
        self.authorized
            .remove(&drone_id.as_u64())
            .ok_or(SwarmError::InvalidDroneId)?;
        Ok(())
    }
}

impl Default for AccessControl {
    fn default() -> Self {
        Self::new()
    }
}

/// Secure audit log for forensics
pub struct AuditLog {
    /// Log entries
    entries: Vec<AuditEntry, 1000>,
}

#[derive(Debug, Clone, Copy)]
pub struct AuditEntry {
    /// Timestamp
    pub timestamp: u64,
    /// Source drone
    pub source: DroneId,
    /// Event type
    pub event: AuditEvent,
}

#[derive(Debug, Clone, Copy)]
pub enum AuditEvent {
    /// Authentication success
    AuthSuccess,
    /// Authentication failure
    AuthFailure,
    /// Message sent
    MessageSent,
    /// Message received
    MessageReceived,
    /// Consensus reached
    ConsensusReached,
    /// Drone joined swarm
    DroneJoined,
    /// Drone left swarm
    DroneLeft,
    /// Security violation detected
    SecurityViolation,
    /// System error
    SystemError,
}

impl AuditLog {
    /// Create a new audit log
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Log an event
    pub fn log(&mut self, source: DroneId, event: AuditEvent) -> Result<()> {
        let entry = AuditEntry {
            timestamp: Self::get_time(),
            source,
            event,
        };

        if self.entries.len() >= self.entries.capacity() {
            // Rotate log (remove oldest entry)
            self.entries.remove(0);
        }

        self.entries
            .push(entry)
            .map_err(|_| SwarmError::ResourceExhausted)?;

        Ok(())
    }

    /// Get recent entries
    pub fn get_recent(&self, count: usize) -> &[AuditEntry] {
        let start = self.entries.len().saturating_sub(count);
        &self.entries[start..]
    }

    /// Get timestamp (uses centralized time abstraction)
    fn get_time() -> u64 {
        crate::get_time_ms()
    }
}

impl Default for AuditLog {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rate_limiter() {
        let mut limiter = RateLimiter::new(5, 1000);
        let drone = DroneId::new(1);

        // Should allow 5 messages
        for _ in 0..5 {
            assert!(limiter.check(drone).is_ok());
        }

        // 6th message should fail
        assert!(limiter.check(drone).is_err());
    }

    #[test]
    fn test_access_control() {
        let mut ac = AccessControl::new();
        let drone = DroneId::new(1);

        ac.authorize(drone, SecurityLevel::Internal).unwrap();

        // Should allow Internal level
        assert!(ac.check_permission(drone, SecurityLevel::Internal).is_ok());

        // Should deny Critical level
        assert!(ac.check_permission(drone, SecurityLevel::Critical).is_err());
    }

    #[test]
    fn test_ids_ban() {
        let mut ids = IntrusionDetectionSystem::new();
        let drone = DroneId::new(1);

        assert!(!ids.is_banned(drone));

        ids.ban_drone(drone).unwrap();
        assert!(ids.is_banned(drone));

        ids.unban_drone(drone).unwrap();
        assert!(!ids.is_banned(drone));
    }
}
