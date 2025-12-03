#![no_main]

use libfuzzer_sys::fuzz_target;
use drone_swarm_system::consensus::ConsensusMessage;
use postcard;

fuzz_target!(|data: &[u8]| {
    // Fuzz consensus message parsing
    if let Ok(msg) = postcard::from_bytes::<ConsensusMessage>(data) {
        // Verify we can serialize back
        let _ = postcard::to_allocvec(&msg);
    }
});
