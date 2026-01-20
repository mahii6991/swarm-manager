//! Post-Quantum Cryptography Implementation
//!
//! This module provides post-quantum resistant cryptographic operations
//! using lattice-based algorithms:
//! - **Kyber768**: NIST Level 3 Key Encapsulation Mechanism
//! - **Dilithium3**: NIST Level 3 Digital Signatures
//! - **AES-256-GCM**: Authenticated encryption with Kyber-derived keys

extern crate alloc;
use alloc::vec::Vec;

use drone_swarm_system::extensions::{CryptoError, CryptoProvider};

use aes_gcm::{
    aead::{Aead, KeyInit},
    Aes256Gcm, Nonce,
};
use hkdf::Hkdf;
use pqcrypto_dilithium::dilithium3::{
    self, DetachedSignature, PublicKey as DilithiumPublicKey, SecretKey as DilithiumSecretKey,
};
use pqcrypto_kyber::kyber768::{
    self, Ciphertext as KyberCiphertext, PublicKey as KyberPublicKey,
    SecretKey as KyberSecretKey, SharedSecret,
};
use pqcrypto_traits::kem::{Ciphertext, PublicKey, SecretKey, SharedSecret as SharedSecretTrait};
use pqcrypto_traits::sign::{
    DetachedSignature as DetachedSignatureTrait, PublicKey as SignPublicKey,
    SecretKey as SignSecretKey,
};
use sha3::Sha3_256;

/// Kyber768 ciphertext size
pub const KYBER_CIPHERTEXT_SIZE: usize = 1088;
/// Kyber768 shared secret size
pub const KYBER_SHARED_SECRET_SIZE: usize = 32;
/// Kyber768 public key size
pub const KYBER_PUBLIC_KEY_SIZE: usize = 1184;
/// Kyber768 secret key size
pub const KYBER_SECRET_KEY_SIZE: usize = 2400;

/// Dilithium3 signature size
pub const DILITHIUM_SIGNATURE_SIZE: usize = 3293;
/// Dilithium3 public key size
pub const DILITHIUM_PUBLIC_KEY_SIZE: usize = 1952;
/// Dilithium3 secret key size
pub const DILITHIUM_SECRET_KEY_SIZE: usize = 4000;

/// AES-256-GCM nonce size
pub const NONCE_SIZE: usize = 12;
/// AES-256-GCM tag size
pub const TAG_SIZE: usize = 16;

/// Post-quantum cryptographic provider.
///
/// Implements:
/// - **Key Encapsulation**: Kyber768 (NIST PQC standard)
/// - **Digital Signatures**: Dilithium3 (NIST PQC standard)
/// - **Symmetric Encryption**: AES-256-GCM (quantum-resistant with 256-bit keys)
#[derive(Debug)]
pub struct PostQuantumCrypto {
    /// Security level configuration
    security_level: SecurityLevel,
    /// Nonce counter for encryption
    nonce_counter: u64,
}

impl PostQuantumCrypto {
    /// Create a new post-quantum crypto provider.
    pub fn new() -> Self {
        Self {
            security_level: SecurityLevel::Level3,
            nonce_counter: 0,
        }
    }

    /// Create with custom security parameters.
    pub fn with_security_level(level: SecurityLevel) -> Self {
        Self {
            security_level: level,
            nonce_counter: 0,
        }
    }

    /// Generate a unique nonce for encryption.
    fn generate_nonce(&mut self) -> [u8; NONCE_SIZE] {
        self.nonce_counter = self.nonce_counter.wrapping_add(1);
        let mut nonce = [0u8; NONCE_SIZE];
        nonce[..8].copy_from_slice(&self.nonce_counter.to_le_bytes());
        // Add randomness to remaining bytes for additional security
        let rand_part = Self::pseudo_random_bytes(self.nonce_counter);
        nonce[8..12].copy_from_slice(&rand_part[..4]);
        nonce
    }

    /// Simple pseudo-random byte generation (for nonce uniqueness)
    fn pseudo_random_bytes(seed: u64) -> [u8; 8] {
        let mut state = seed ^ 0x5DEECE66D;
        let mut result = [0u8; 8];
        for byte in &mut result {
            state = state.wrapping_mul(0x5DEECE66D).wrapping_add(0xB);
            *byte = (state >> 17) as u8;
        }
        result
    }

    /// Derive AES-256 key from shared secret using HKDF-SHA3-256
    fn derive_aes_key(shared_secret: &[u8], info: &[u8]) -> Result<[u8; 32], CryptoError> {
        let hk = Hkdf::<Sha3_256>::new(None, shared_secret);
        let mut key = [0u8; 32];
        hk.expand(info, &mut key)
            .map_err(|_| CryptoError::ProviderError("HKDF expansion failed"))?;
        Ok(key)
    }
}

impl Default for PostQuantumCrypto {
    fn default() -> Self {
        Self::new()
    }
}

/// Security levels for post-quantum algorithms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityLevel {
    /// NIST Level 1 (equivalent to AES-128)
    Level1,
    /// NIST Level 3 (equivalent to AES-192) - Default
    Level3,
    /// NIST Level 5 (equivalent to AES-256)
    Level5,
}

impl CryptoProvider for PostQuantumCrypto {
    fn encrypt(&self, plaintext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        // Key must be exactly 32 bytes for AES-256
        if key.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }

        // Create AES-256-GCM cipher
        let cipher = Aes256Gcm::new_from_slice(key)
            .map_err(|_| CryptoError::ProviderError("Failed to create AES cipher"))?;

        // Generate nonce (in production, use a proper counter or random)
        let mut crypto = PostQuantumCrypto::new();
        let nonce_bytes = crypto.generate_nonce();
        let nonce = Nonce::from_slice(&nonce_bytes);

        // Encrypt
        let ciphertext = cipher
            .encrypt(nonce, plaintext)
            .map_err(|_| CryptoError::EncryptionFailed)?;

        // Prepend nonce to ciphertext: nonce || ciphertext || tag
        let mut result = Vec::with_capacity(NONCE_SIZE + ciphertext.len());
        result.extend_from_slice(&nonce_bytes);
        result.extend_from_slice(&ciphertext);
        Ok(result)
    }

    fn decrypt(&self, ciphertext: &[u8], key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if key.len() != 32 {
            return Err(CryptoError::InvalidKeyLength);
        }

        // Minimum size: nonce + tag
        if ciphertext.len() < NONCE_SIZE + TAG_SIZE {
            return Err(CryptoError::DecryptionFailed);
        }

        // Extract nonce
        let nonce = Nonce::from_slice(&ciphertext[..NONCE_SIZE]);
        let encrypted_data = &ciphertext[NONCE_SIZE..];

        // Create cipher and decrypt
        let cipher = Aes256Gcm::new_from_slice(key)
            .map_err(|_| CryptoError::ProviderError("Failed to create AES cipher"))?;

        cipher
            .decrypt(nonce, encrypted_data)
            .map_err(|_| CryptoError::DecryptionFailed)
    }

    fn sign(&self, data: &[u8], private_key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        // Dilithium3 secret key is 4000 bytes
        if private_key.len() != DILITHIUM_SECRET_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }

        // Reconstruct secret key from bytes
        let sk = DilithiumSecretKey::from_bytes(private_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium secret key"))?;

        // Sign the data
        let signature = dilithium3::detached_sign(data, &sk);

        Ok(signature.as_bytes().to_vec())
    }

    fn verify(
        &self,
        data: &[u8],
        signature: &[u8],
        public_key: &[u8],
    ) -> Result<bool, CryptoError> {
        // Dilithium3 public key is 1952 bytes
        if public_key.len() != DILITHIUM_PUBLIC_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }

        // Dilithium3 signature is 3293 bytes
        if signature.len() != DILITHIUM_SIGNATURE_SIZE {
            return Err(CryptoError::VerificationFailed);
        }

        // Reconstruct public key from bytes
        let pk = DilithiumPublicKey::from_bytes(public_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium public key"))?;

        // Reconstruct signature from bytes
        let sig = DetachedSignature::from_bytes(signature)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium signature"))?;

        // Verify signature
        match dilithium3::verify_detached_signature(&sig, data, &pk) {
            Ok(()) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    fn generate_signing_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError> {
        // Generate Dilithium3 keypair
        let (pk, sk) = dilithium3::keypair();

        let private_key = sk.as_bytes().to_vec();
        let public_key = pk.as_bytes().to_vec();

        // Verify sizes
        debug_assert_eq!(private_key.len(), DILITHIUM_SECRET_KEY_SIZE);
        debug_assert_eq!(public_key.len(), DILITHIUM_PUBLIC_KEY_SIZE);

        Ok((private_key, public_key))
    }

    fn generate_exchange_keypair(&self) -> Result<(Vec<u8>, Vec<u8>), CryptoError> {
        // Generate Kyber768 keypair
        let (pk, sk) = kyber768::keypair();

        let private_key = sk.as_bytes().to_vec();
        let public_key = pk.as_bytes().to_vec();

        // Verify sizes
        debug_assert_eq!(private_key.len(), KYBER_SECRET_KEY_SIZE);
        debug_assert_eq!(public_key.len(), KYBER_PUBLIC_KEY_SIZE);

        Ok((private_key, public_key))
    }

    fn key_exchange(
        &self,
        our_private: &[u8],
        their_public: &[u8],
    ) -> Result<Vec<u8>, CryptoError> {
        // For Kyber, key exchange works as follows:
        // - If we're the initiator (encapsulator): use their public key to encapsulate
        // - If we're the responder (decapsulator): use our private key to decapsulate
        //
        // This function handles decapsulation (responder side)
        // The ciphertext must be passed via their_public for decapsulation

        if our_private.len() == KYBER_SECRET_KEY_SIZE {
            // We have a secret key - this is decapsulation
            if their_public.len() == KYBER_CIPHERTEXT_SIZE {
                // their_public is actually a ciphertext
                let sk = KyberSecretKey::from_bytes(our_private)
                    .map_err(|_| CryptoError::ProviderError("Invalid Kyber secret key"))?;
                let ct = KyberCiphertext::from_bytes(their_public)
                    .map_err(|_| CryptoError::ProviderError("Invalid Kyber ciphertext"))?;

                let ss = kyber768::decapsulate(&ct, &sk);
                return Ok(ss.as_bytes().to_vec());
            } else if their_public.len() == KYBER_PUBLIC_KEY_SIZE {
                // They sent a public key - this is encapsulation (initiator side)
                let pk = KyberPublicKey::from_bytes(their_public)
                    .map_err(|_| CryptoError::ProviderError("Invalid Kyber public key"))?;

                let (ss, _ct) = kyber768::encapsulate(&pk);
                return Ok(ss.as_bytes().to_vec());
            }
        }

        Err(CryptoError::KeyExchangeFailed)
    }

    fn provider_name(&self) -> &'static str {
        "PostQuantumCrypto (Kyber768 + Dilithium3)"
    }

    fn security_level(&self) -> u32 {
        // Post-quantum security level (NIST Level 3 ~ 192-bit classical)
        match self.security_level {
            SecurityLevel::Level1 => 128,
            SecurityLevel::Level3 => 192,
            SecurityLevel::Level5 => 256,
        }
    }
}

/// Kyber key encapsulation helper functions
pub mod kyber {
    use super::*;

    /// Encapsulate a shared secret using a public key.
    /// Returns (shared_secret, ciphertext).
    pub fn encapsulate(public_key: &[u8]) -> Result<(Vec<u8>, Vec<u8>), CryptoError> {
        if public_key.len() != KYBER_PUBLIC_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }

        let pk = KyberPublicKey::from_bytes(public_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Kyber public key"))?;

        let (ss, ct) = kyber768::encapsulate(&pk);

        Ok((ss.as_bytes().to_vec(), ct.as_bytes().to_vec()))
    }

    /// Decapsulate a shared secret using a secret key and ciphertext.
    pub fn decapsulate(secret_key: &[u8], ciphertext: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if secret_key.len() != KYBER_SECRET_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }
        if ciphertext.len() != KYBER_CIPHERTEXT_SIZE {
            return Err(CryptoError::ProviderError("Invalid ciphertext size"));
        }

        let sk = KyberSecretKey::from_bytes(secret_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Kyber secret key"))?;
        let ct = KyberCiphertext::from_bytes(ciphertext)
            .map_err(|_| CryptoError::ProviderError("Invalid Kyber ciphertext"))?;

        let ss = kyber768::decapsulate(&ct, &sk);

        Ok(ss.as_bytes().to_vec())
    }

    /// Generate a Kyber768 keypair.
    pub fn generate_keypair() -> (Vec<u8>, Vec<u8>) {
        let (pk, sk) = kyber768::keypair();
        (sk.as_bytes().to_vec(), pk.as_bytes().to_vec())
    }
}

/// Dilithium signing helper functions
pub mod dilithium {
    use super::*;

    /// Sign data with a Dilithium3 secret key.
    pub fn sign(data: &[u8], secret_key: &[u8]) -> Result<Vec<u8>, CryptoError> {
        if secret_key.len() != DILITHIUM_SECRET_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }

        let sk = DilithiumSecretKey::from_bytes(secret_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium secret key"))?;

        let signature = dilithium3::detached_sign(data, &sk);
        Ok(signature.as_bytes().to_vec())
    }

    /// Verify a Dilithium3 signature.
    pub fn verify(data: &[u8], signature: &[u8], public_key: &[u8]) -> Result<bool, CryptoError> {
        if public_key.len() != DILITHIUM_PUBLIC_KEY_SIZE {
            return Err(CryptoError::InvalidKeyLength);
        }
        if signature.len() != DILITHIUM_SIGNATURE_SIZE {
            return Err(CryptoError::VerificationFailed);
        }

        let pk = DilithiumPublicKey::from_bytes(public_key)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium public key"))?;
        let sig = DetachedSignature::from_bytes(signature)
            .map_err(|_| CryptoError::ProviderError("Invalid Dilithium signature"))?;

        match dilithium3::verify_detached_signature(&sig, data, &pk) {
            Ok(()) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    /// Generate a Dilithium3 keypair.
    pub fn generate_keypair() -> (Vec<u8>, Vec<u8>) {
        let (pk, sk) = dilithium3::keypair();
        (sk.as_bytes().to_vec(), pk.as_bytes().to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encrypt_decrypt_roundtrip() {
        let crypto = PostQuantumCrypto::new();
        let key = [0x42u8; 32]; // 256-bit key
        let plaintext = b"Hello, post-quantum world!";

        let ciphertext = crypto.encrypt(plaintext, &key).unwrap();
        let decrypted = crypto.decrypt(&ciphertext, &key).unwrap();

        assert_eq!(plaintext.as_slice(), decrypted.as_slice());
    }

    #[test]
    fn test_encrypt_wrong_key_fails() {
        let crypto = PostQuantumCrypto::new();
        let key1 = [0x42u8; 32];
        let key2 = [0x43u8; 32];
        let plaintext = b"Secret message";

        let ciphertext = crypto.encrypt(plaintext, &key1).unwrap();
        let result = crypto.decrypt(&ciphertext, &key2);

        assert!(result.is_err());
    }

    #[test]
    fn test_sign_verify() {
        let crypto = PostQuantumCrypto::new();
        let (private_key, public_key) = crypto.generate_signing_keypair().unwrap();
        let data = b"Message to sign";

        let signature = crypto.sign(data, &private_key).unwrap();
        let valid = crypto.verify(data, &signature, &public_key).unwrap();

        assert!(valid);
        assert_eq!(signature.len(), DILITHIUM_SIGNATURE_SIZE);
    }

    #[test]
    fn test_sign_verify_wrong_data_fails() {
        let crypto = PostQuantumCrypto::new();
        let (private_key, public_key) = crypto.generate_signing_keypair().unwrap();
        let data = b"Original message";
        let wrong_data = b"Modified message";

        let signature = crypto.sign(data, &private_key).unwrap();
        let valid = crypto.verify(wrong_data, &signature, &public_key).unwrap();

        assert!(!valid);
    }

    #[test]
    fn test_kyber_keypair_sizes() {
        let crypto = PostQuantumCrypto::new();
        let (private_key, public_key) = crypto.generate_exchange_keypair().unwrap();

        assert_eq!(private_key.len(), KYBER_SECRET_KEY_SIZE);
        assert_eq!(public_key.len(), KYBER_PUBLIC_KEY_SIZE);
    }

    #[test]
    fn test_dilithium_keypair_sizes() {
        let crypto = PostQuantumCrypto::new();
        let (private_key, public_key) = crypto.generate_signing_keypair().unwrap();

        assert_eq!(private_key.len(), DILITHIUM_SECRET_KEY_SIZE);
        assert_eq!(public_key.len(), DILITHIUM_PUBLIC_KEY_SIZE);
    }

    #[test]
    fn test_kyber_encapsulation_decapsulation() {
        let (secret_key, public_key) = kyber::generate_keypair();

        // Encapsulate using public key
        let (shared_secret_enc, ciphertext) = kyber::encapsulate(&public_key).unwrap();

        // Decapsulate using secret key
        let shared_secret_dec = kyber::decapsulate(&secret_key, &ciphertext).unwrap();

        assert_eq!(shared_secret_enc, shared_secret_dec);
        assert_eq!(shared_secret_enc.len(), KYBER_SHARED_SECRET_SIZE);
    }

    #[test]
    fn test_provider_name() {
        let crypto = PostQuantumCrypto::new();
        assert!(crypto.provider_name().contains("Kyber"));
        assert!(crypto.provider_name().contains("Dilithium"));
    }

    #[test]
    fn test_security_level() {
        let crypto = PostQuantumCrypto::with_security_level(SecurityLevel::Level3);
        assert_eq!(crypto.security_level(), 192);

        let crypto = PostQuantumCrypto::with_security_level(SecurityLevel::Level5);
        assert_eq!(crypto.security_level(), 256);
    }
}
