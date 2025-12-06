//! Comprehensive tests for cryptographic security module
//!
//! Tests encryption, signatures, key management, and replay protection

use drone_swarm_system::crypto::*;
use drone_swarm_system::types::*;

#[cfg(test)]
mod crypto_context_tests {
    use super::*;

    #[test]
    fn test_crypto_context_new() {
        let seed = [42u8; 32];
        let ctx = CryptoContext::new(seed);

        // Verify context is created
        assert_eq!(ctx.public_key().as_bytes().len(), 32);
    }

    #[test]
    fn test_encrypt_decrypt_roundtrip() {
        let mut ctx1 = CryptoContext::new([1u8; 32]);
        let ctx2 = CryptoContext::new([1u8; 32]);

        let plaintext = b"Test message for roundtrip";
        let aad = b"associated data";

        let encrypted = ctx1.encrypt_and_sign(plaintext, aad).unwrap();
        let decrypted = ctx2
            .verify_and_decrypt(&encrypted, aad, ctx1.public_key())
            .unwrap();

        assert_eq!(&decrypted[..], plaintext);
    }

    #[test]
    fn test_encrypt_empty_message() {
        let mut ctx = CryptoContext::new([2u8; 32]);

        let plaintext = b"";
        let encrypted = ctx.encrypt_and_sign(plaintext, b"").unwrap();

        // Should succeed even with empty plaintext
        assert!(encrypted.len() > 0);
    }

    #[test]
    fn test_encrypt_with_empty_aad() {
        let mut ctx = CryptoContext::new([3u8; 32]);

        let plaintext = b"Message without AAD";
        let encrypted = ctx.encrypt_and_sign(plaintext, b"").unwrap();

        let decrypted = ctx
            .verify_and_decrypt(&encrypted, b"", ctx.public_key())
            .unwrap();
        assert_eq!(&decrypted[..], plaintext);
    }

    #[test]
    fn test_decrypt_wrong_aad() {
        let mut ctx1 = CryptoContext::new([4u8; 32]);
        let ctx2 = CryptoContext::new([4u8; 32]);

        let plaintext = b"Secret message";
        let aad = b"correct aad";

        let encrypted = ctx1.encrypt_and_sign(plaintext, aad).unwrap();

        // Try to decrypt with wrong AAD
        let result = ctx2.verify_and_decrypt(&encrypted, b"wrong aad", ctx1.public_key());
        assert!(result.is_err());
    }

    #[test]
    fn test_decrypt_wrong_public_key() {
        let mut ctx1 = CryptoContext::new([5u8; 32]);
        let ctx2 = CryptoContext::new([6u8; 32]); // Different key

        let plaintext = b"Secret message";
        let encrypted = ctx1.encrypt_and_sign(plaintext, b"").unwrap();

        // Try to decrypt with wrong public key
        let result = ctx2.verify_and_decrypt(&encrypted, b"", ctx2.public_key());
        assert!(result.is_err());
    }

    #[test]
    fn test_decrypt_invalid_message_too_short() {
        let ctx = CryptoContext::new([7u8; 32]);

        let invalid_message = b"short";
        let result = ctx.verify_and_decrypt(invalid_message, b"", ctx.public_key());

        assert!(result.is_err());
    }

    #[test]
    fn test_decrypt_corrupted_signature() {
        let mut ctx = CryptoContext::new([8u8; 32]);

        let plaintext = b"Important message";
        let mut encrypted = ctx.encrypt_and_sign(plaintext, b"").unwrap();

        // Corrupt the last byte of signature
        if let Some(last) = encrypted.last_mut() {
            *last ^= 0xFF;
        }

        let result = ctx.verify_and_decrypt(&encrypted, b"", ctx.public_key());
        assert!(result.is_err());
    }

    #[test]
    fn test_decrypt_corrupted_ciphertext() {
        let mut ctx = CryptoContext::new([9u8; 32]);

        let plaintext = b"Important message";
        let mut encrypted = ctx.encrypt_and_sign(plaintext, b"").unwrap();

        // Corrupt a byte in the middle (ciphertext area)
        encrypted[20] ^= 0xFF;

        let result = ctx.verify_and_decrypt(&encrypted, b"", ctx.public_key());
        assert!(result.is_err());
    }

    #[test]
    fn test_multiple_encryptions_different_nonces() {
        let mut ctx = CryptoContext::new([10u8; 32]);

        let msg1 = ctx.encrypt_and_sign(b"Message 1", b"").unwrap();
        let msg2 = ctx.encrypt_and_sign(b"Message 2", b"").unwrap();
        let msg3 = ctx.encrypt_and_sign(b"Message 3", b"").unwrap();

        // Nonces should be different (check first 12 bytes)
        assert_ne!(&msg1[..12], &msg2[..12]);
        assert_ne!(&msg2[..12], &msg3[..12]);
    }

    #[test]
    fn test_public_key_retrieval() {
        let ctx = CryptoContext::new([11u8; 32]);

        let public_key = ctx.public_key();
        assert_eq!(public_key.as_bytes().len(), 32);
    }

    #[test]
    fn test_needs_rekeying_initial() {
        let ctx = CryptoContext::new([12u8; 32]);

        // Should not need rekeying initially
        assert!(!ctx.needs_rekeying());
    }

    #[test]
    fn test_large_message_encryption() {
        let mut ctx = CryptoContext::new([13u8; 32]);

        // Create a 1KB message
        let large_message = vec![0x42u8; 1024];
        let encrypted = ctx.encrypt_and_sign(&large_message, b"").unwrap();

        let decrypted = ctx
            .verify_and_decrypt(&encrypted, b"", ctx.public_key())
            .unwrap();
        assert_eq!(&decrypted[..], &large_message[..]);
    }
}

#[cfg(test)]
mod hashing_tests {
    use super::*;

    #[test]
    fn test_fast_hash_blake3() {
        let data = b"Test data for BLAKE3";
        let hash = CryptoContext::fast_hash(data);

        // BLAKE3 produces 32 bytes
        assert_eq!(hash.len(), 32);
    }

    #[test]
    fn test_fast_hash_deterministic() {
        let data = b"Same data";

        let hash1 = CryptoContext::fast_hash(data);
        let hash2 = CryptoContext::fast_hash(data);

        assert_eq!(hash1, hash2);
    }

    #[test]
    fn test_fast_hash_different_inputs() {
        let hash1 = CryptoContext::fast_hash(b"input1");
        let hash2 = CryptoContext::fast_hash(b"input2");

        assert_ne!(hash1, hash2);
    }

    #[test]
    fn test_fast_hash_empty_input() {
        let hash = CryptoContext::fast_hash(b"");
        assert_eq!(hash.len(), 32);
    }

    #[test]
    fn test_secure_hash_sha3() {
        let data = b"Test data for SHA3-256";
        let hash = CryptoContext::secure_hash(data);

        // SHA3-256 produces 32 bytes
        assert_eq!(hash.len(), 32);
    }

    #[test]
    fn test_secure_hash_deterministic() {
        let data = b"Same data";

        let hash1 = CryptoContext::secure_hash(data);
        let hash2 = CryptoContext::secure_hash(data);

        assert_eq!(hash1, hash2);
    }

    #[test]
    fn test_secure_hash_different_inputs() {
        let hash1 = CryptoContext::secure_hash(b"input1");
        let hash2 = CryptoContext::secure_hash(b"input2");

        assert_ne!(hash1, hash2);
    }

    #[test]
    fn test_secure_hash_empty_input() {
        let hash = CryptoContext::secure_hash(b"");
        assert_eq!(hash.len(), 32);
    }

    #[test]
    fn test_fast_vs_secure_hash_different() {
        let data = b"Compare BLAKE3 vs SHA3";

        let fast = CryptoContext::fast_hash(data);
        let secure = CryptoContext::secure_hash(data);

        // Different algorithms should produce different hashes
        assert_ne!(fast, secure);
    }
}

#[cfg(test)]
mod key_exchange_tests {
    use super::*;

    #[test]
    fn test_key_exchange_x25519() {
        let alice_private = [1u8; 32];
        let bob_private = [2u8; 32];

        // Derive public keys
        let alice_secret = x25519_dalek::StaticSecret::from(alice_private);
        let bob_secret = x25519_dalek::StaticSecret::from(bob_private);

        let alice_public = x25519_dalek::PublicKey::from(&alice_secret);
        let bob_public = x25519_dalek::PublicKey::from(&bob_secret);

        // Perform key exchange
        let alice_shared =
            CryptoContext::key_exchange(&alice_private, bob_public.as_bytes()).unwrap();
        let bob_shared =
            CryptoContext::key_exchange(&bob_private, alice_public.as_bytes()).unwrap();

        // Both should compute the same shared secret
        assert_eq!(alice_shared, bob_shared);
    }

    #[test]
    fn test_derive_session_key() {
        let shared_secret = [0x42u8; 32];
        let context = b"session-context-1";

        let session_key = CryptoContext::derive_session_key(&shared_secret, context);

        assert_eq!(session_key.len(), 32);
    }

    #[test]
    fn test_derive_session_key_deterministic() {
        let shared_secret = [0x55u8; 32];
        let context = b"same-context";

        let key1 = CryptoContext::derive_session_key(&shared_secret, context);
        let key2 = CryptoContext::derive_session_key(&shared_secret, context);

        assert_eq!(key1, key2);
    }

    #[test]
    fn test_derive_session_key_different_context() {
        let shared_secret = [0x66u8; 32];

        let key1 = CryptoContext::derive_session_key(&shared_secret, b"context1");
        let key2 = CryptoContext::derive_session_key(&shared_secret, b"context2");

        // Different contexts should produce different keys
        assert_ne!(key1, key2);
    }

    #[test]
    fn test_derive_session_key_different_secret() {
        let context = b"same-context";

        let key1 = CryptoContext::derive_session_key(&[0x11u8; 32], context);
        let key2 = CryptoContext::derive_session_key(&[0x22u8; 32], context);

        // Different secrets should produce different keys
        assert_ne!(key1, key2);
    }
}

#[cfg(test)]
mod keystore_tests {
    use super::*;

    #[test]
    fn test_keystore_new() {
        let keystore = KeyStore::new();

        // Should be empty initially
        assert!(!keystore.has_key(DroneId::new(1)));
    }

    #[test]
    fn test_keystore_default() {
        let keystore = KeyStore::default();

        assert!(!keystore.has_key(DroneId::new(1)));
    }

    #[test]
    fn test_keystore_add_key() {
        let mut keystore = KeyStore::new();
        let ctx = CryptoContext::new([1u8; 32]);

        let result = keystore.add_key(DroneId::new(1), *ctx.public_key());
        assert!(result.is_ok());
    }

    #[test]
    fn test_keystore_has_key() {
        let mut keystore = KeyStore::new();
        let ctx = CryptoContext::new([2u8; 32]);
        let drone_id = DroneId::new(42);

        assert!(!keystore.has_key(drone_id));

        keystore.add_key(drone_id, *ctx.public_key()).unwrap();

        assert!(keystore.has_key(drone_id));
    }

    #[test]
    fn test_keystore_get_key() {
        let mut keystore = KeyStore::new();
        let ctx = CryptoContext::new([3u8; 32]);
        let drone_id = DroneId::new(100);

        keystore.add_key(drone_id, *ctx.public_key()).unwrap();

        let retrieved_key = keystore.get_key(drone_id).unwrap();
        assert_eq!(retrieved_key.as_bytes(), ctx.public_key().as_bytes());
    }

    #[test]
    fn test_keystore_get_nonexistent_key() {
        let keystore = KeyStore::new();

        let result = keystore.get_key(DroneId::new(999));
        assert!(result.is_err());
    }

    #[test]
    fn test_keystore_remove_key() {
        let mut keystore = KeyStore::new();
        let ctx = CryptoContext::new([4u8; 32]);
        let drone_id = DroneId::new(50);

        keystore.add_key(drone_id, *ctx.public_key()).unwrap();
        assert!(keystore.has_key(drone_id));

        let result = keystore.remove_key(drone_id);
        assert!(result.is_ok());
        assert!(!keystore.has_key(drone_id));
    }

    #[test]
    fn test_keystore_remove_nonexistent_key() {
        let mut keystore = KeyStore::new();

        let result = keystore.remove_key(DroneId::new(123));
        assert!(result.is_err());
    }

    #[test]
    fn test_keystore_multiple_keys() {
        let mut keystore = KeyStore::new();

        for i in 1..=10 {
            let ctx = CryptoContext::new([i as u8; 32]);
            keystore
                .add_key(DroneId::new(i as u64), *ctx.public_key())
                .unwrap();
        }

        // All keys should be present
        for i in 1..=10 {
            assert!(keystore.has_key(DroneId::new(i as u64)));
        }
    }

    #[test]
    fn test_keystore_update_key() {
        let mut keystore = KeyStore::new();
        let drone_id = DroneId::new(1);

        let ctx1 = CryptoContext::new([1u8; 32]);
        let ctx2 = CryptoContext::new([2u8; 32]);

        keystore.add_key(drone_id, *ctx1.public_key()).unwrap();

        // Update with new key (should replace)
        keystore.add_key(drone_id, *ctx2.public_key()).unwrap();

        let retrieved = keystore.get_key(drone_id).unwrap();
        assert_eq!(retrieved.as_bytes(), ctx2.public_key().as_bytes());
    }
}

#[cfg(test)]
mod nonce_tracker_tests {
    use super::*;

    #[test]
    fn test_nonce_tracker_new() {
        let tracker = NonceTracker::new();
        // Should be empty, test by trying to check a nonce
        // (implementation detail - we can't inspect internals)
        drop(tracker);
    }

    #[test]
    fn test_nonce_tracker_default() {
        let tracker = NonceTracker::default();
        drop(tracker);
    }

    #[test]
    fn test_nonce_first_use() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        let result = tracker.check_nonce(drone, 1);
        assert!(result.is_ok());
    }

    #[test]
    fn test_nonce_replay_attack() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        tracker.check_nonce(drone, 100).unwrap();

        // Replay same nonce should fail
        let result = tracker.check_nonce(drone, 100);
        assert!(result.is_err());
    }

    #[test]
    fn test_nonce_strictly_increasing() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        assert!(tracker.check_nonce(drone, 1).is_ok());
        assert!(tracker.check_nonce(drone, 2).is_ok());
        assert!(tracker.check_nonce(drone, 3).is_ok());
        assert!(tracker.check_nonce(drone, 10).is_ok());
    }

    #[test]
    fn test_nonce_backwards_fails() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        tracker.check_nonce(drone, 100).unwrap();

        // Going backwards should fail
        let result = tracker.check_nonce(drone, 50);
        assert!(result.is_err());
    }

    #[test]
    fn test_nonce_zero_after_nonzero() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        tracker.check_nonce(drone, 10).unwrap();

        // Zero nonce after higher value should fail
        let result = tracker.check_nonce(drone, 0);
        assert!(result.is_err());
    }

    #[test]
    fn test_nonce_multiple_drones() {
        let mut tracker = NonceTracker::new();
        let drone1 = DroneId::new(1);
        let drone2 = DroneId::new(2);

        // Each drone has independent nonce tracking
        assert!(tracker.check_nonce(drone1, 1).is_ok());
        assert!(tracker.check_nonce(drone2, 1).is_ok());

        assert!(tracker.check_nonce(drone1, 2).is_ok());
        assert!(tracker.check_nonce(drone2, 2).is_ok());
    }

    #[test]
    fn test_nonce_large_gaps() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        assert!(tracker.check_nonce(drone, 1).is_ok());
        assert!(tracker.check_nonce(drone, 1000).is_ok());
        assert!(tracker.check_nonce(drone, 1000000).is_ok());
    }

    #[test]
    fn test_nonce_max_value() {
        let mut tracker = NonceTracker::new();
        let drone = DroneId::new(1);

        // Test with maximum u64 value
        let result = tracker.check_nonce(drone, u64::MAX);
        assert!(result.is_ok());
    }

    #[test]
    fn test_nonce_same_value_different_drones() {
        let mut tracker = NonceTracker::new();

        // Same nonce value is OK for different drones
        assert!(tracker.check_nonce(DroneId::new(1), 100).is_ok());
        assert!(tracker.check_nonce(DroneId::new(2), 100).is_ok());
        assert!(tracker.check_nonce(DroneId::new(3), 100).is_ok());
    }
}

#[cfg(test)]
mod constants_tests {
    use super::*;

    #[test]
    fn test_tag_size() {
        assert_eq!(TAG_SIZE, 16);
    }

    #[test]
    fn test_signature_size() {
        assert_eq!(SIGNATURE_SIZE, 64);
    }

    #[test]
    fn test_crypto_overhead() {
        assert_eq!(CRYPTO_OVERHEAD, TAG_SIZE + SIGNATURE_SIZE + 12);
        assert_eq!(CRYPTO_OVERHEAD, 92);
    }

    #[test]
    fn test_max_safe_encryptions() {
        assert_eq!(MAX_SAFE_ENCRYPTIONS, 1_000_000_000);
    }
}
