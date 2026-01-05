//! Merkle Tree implementation for tamper-evident logging
//!
//! Part of the SwarmRaft consensus enhancement.
//! Provides O(log n) verification of log entries and a root hash
//! representing the entire state of the distributed ledger.

use crate::crypto::CryptoContext;
use crate::types::{Result, SwarmError};
use heapless::Vec;

/// Merkle Tree node
#[derive(Debug, Clone, PartialEq)]
pub enum MerkleNode {
    /// Leaf node containing data hash
    Leaf([u8; 32]),
    /// Internal node containing combined hash of children
    Node {
        hash: [u8; 32],
        left_child: usize,  // Index in storage vector
        right_child: usize, // Index in storage vector
    },
}

impl MerkleNode {
    pub fn hash(&self) -> [u8; 32] {
        match self {
            MerkleNode::Leaf(h) => *h,
            MerkleNode::Node { hash, .. } => *hash,
        }
    }
}

/// Fixed-size Merkle Tree for embedded systems
pub struct MerkleTree<const N: usize> {
    /// Flattened tree storage
    nodes: Vec<MerkleNode, N>,
    /// Indices of the current level being built
    current_level_indices: Vec<usize, N>,
}

impl<const N: usize> Default for MerkleTree<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> MerkleTree<N> {
    /// Create a new empty Merkle Tree
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            current_level_indices: Vec::new(),
        }
    }

    /// Compute the Merkle Root of a list of data items
    ///
    /// This rebuilds the tree from scratch. For an append-only log,
    /// an incremental update strategy would be more efficient,
    /// but this ensures correctness for the foundation phase.
    pub fn compute_root(&mut self, data_items: &[&[u8]]) -> Result<[u8; 32]> {
        if data_items.is_empty() {
            return Ok([0u8; 32]); // Empty tree hash
        }

        self.nodes.clear();
        self.current_level_indices.clear();

        // 1. Create leaves
        for item in data_items {
            let hash = CryptoContext::secure_hash(item);
            if self.nodes.push(MerkleNode::Leaf(hash)).is_err() {
                return Err(SwarmError::BufferFull);
            }
            if self
                .current_level_indices
                .push(self.nodes.len() - 1)
                .is_err()
            {
                return Err(SwarmError::BufferFull);
            }
        }

        // 2. Build tree upwards
        while self.current_level_indices.len() > 1 {
            let mut next_level_indices = Vec::<usize, N>::new();
            let mut i = 0;

            while i < self.current_level_indices.len() {
                let left_idx = self.current_level_indices[i];
                let right_idx = if i + 1 < self.current_level_indices.len() {
                    self.current_level_indices[i + 1]
                } else {
                    // Duplicate last node if odd number of nodes
                    left_idx
                };

                // Compute combined hash: H(left || right)
                let left_hash = self.nodes[left_idx].hash();
                let right_hash = self.nodes[right_idx].hash();

                let mut combined = Vec::<u8, 64>::new();
                combined
                    .extend_from_slice(&left_hash)
                    .map_err(|_| SwarmError::BufferFull)?;
                combined
                    .extend_from_slice(&right_hash)
                    .map_err(|_| SwarmError::BufferFull)?;

                let parent_hash = CryptoContext::secure_hash(&combined);

                // Store parent
                if self
                    .nodes
                    .push(MerkleNode::Node {
                        hash: parent_hash,
                        left_child: left_idx,
                        right_child: right_idx,
                    })
                    .is_err()
                {
                    return Err(SwarmError::BufferFull);
                }

                if next_level_indices.push(self.nodes.len() - 1).is_err() {
                    return Err(SwarmError::BufferFull);
                }

                i += 2;
            }

            self.current_level_indices = next_level_indices;
        }

        // Return root hash
        if let Some(&root_idx) = self.current_level_indices.first() {
            Ok(self.nodes[root_idx].hash())
        } else {
            Ok([0u8; 32])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_root() {
        let mut tree = MerkleTree::<100>::new();
        let root = tree.compute_root(&[]).unwrap();
        assert_eq!(root, [0u8; 32]);
    }

    #[test]
    fn test_single_leaf() {
        let mut tree = MerkleTree::<10>::new();
        let data = b"test";
        let root = tree.compute_root(&[data]).unwrap();

        // Root should be hash of leaf
        let expected = CryptoContext::secure_hash(data);
        assert_eq!(root, expected);
    }

    #[test]
    fn test_consistency() {
        let mut tree1 = MerkleTree::<100>::new();
        let mut tree2 = MerkleTree::<100>::new();

        let data = vec![b"one".as_slice(), b"two".as_slice(), b"three".as_slice()];

        let root1 = tree1.compute_root(&data).unwrap();
        let root2 = tree2.compute_root(&data).unwrap();

        assert_eq!(root1, root2);
    }

    #[test]
    fn test_tamper_evidence() {
        let mut tree = MerkleTree::<100>::new();

        let data1 = vec![b"one".as_slice(), b"two".as_slice()];
        let root1 = tree.compute_root(&data1).unwrap();

        let data2 = vec![b"one".as_slice(), b"two_modified".as_slice()];
        let root2 = tree.compute_root(&data2).unwrap();

        assert_ne!(root1, root2);
    }
}
