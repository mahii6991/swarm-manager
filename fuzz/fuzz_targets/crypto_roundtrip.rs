#![no_main]

use libfuzzer_sys::fuzz_target;
use drone_swarm_system::crypto::CryptoContext;

fuzz_target!(|data: &[u8]| {
    // Use fuzz data as seed (pad or truncate to 32 bytes)
    let mut seed = [0u8; 32];
    let copy_len = data.len().min(32);
    seed[..copy_len].copy_from_slice(&data[..copy_len]);

    let mut crypto = CryptoContext::new(seed);

    // Try encrypting the fuzz data
    if data.len() > 32 {
        let plaintext = &data[32..];
        if let Ok(ciphertext) = crypto.encrypt(plaintext) {
            // Verify decryption works
            if let Ok(decrypted) = crypto.decrypt(&ciphertext) {
                // Roundtrip should preserve data
                assert_eq!(plaintext, decrypted.as_slice());
            }
        }
    }
});
