#![no_main]

use libfuzzer_sys::fuzz_target;
use drone_swarm_system::network::NetworkMessage;
use postcard;

fuzz_target!(|data: &[u8]| {
    // Try to deserialize arbitrary bytes as NetworkMessage
    if let Ok(msg) = postcard::from_bytes::<NetworkMessage>(data) {
        // If deserialization succeeds, try to serialize it back
        let _ = postcard::to_allocvec(&msg);
    }
});
