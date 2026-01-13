#!/bin/bash
# =============================================================================
# Military Extensions Setup Script
# =============================================================================
# This script initializes the military extensions submodule for authorized users.
#
# PREREQUISITES:
#   - SSH access to the private military repository
#   - Appropriate security clearance and authorization
#   - Compliance with ITAR/EAR export control regulations
#
# USAGE:
#   ./scripts/setup-military.sh
#
# =============================================================================

set -e

MILITARY_REPO="git@github.com:mahii6991/swarm-manager-military.git"
MILITARY_DIR="military"

echo "==========================================="
echo "  Military Extensions Setup"
echo "==========================================="
echo ""
echo "WARNING: This will initialize the military extensions submodule."
echo "Ensure you have:"
echo "  1. SSH access to the private repository"
echo "  2. Appropriate authorization"
echo "  3. Export control compliance (ITAR/EAR)"
echo ""
read -p "Continue? [y/N] " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

# Check if submodule already exists
if [ -d "$MILITARY_DIR/.git" ] || [ -f "$MILITARY_DIR/.git" ]; then
    echo "Military submodule already initialized."
    echo "Updating to latest..."
    git submodule update --remote "$MILITARY_DIR"
else
    # Remove placeholder if exists
    if [ -f "$MILITARY_DIR/.gitkeep" ]; then
        rm "$MILITARY_DIR/.gitkeep"
        rmdir "$MILITARY_DIR" 2>/dev/null || true
    fi

    echo "Adding military submodule..."
    git submodule add "$MILITARY_REPO" "$MILITARY_DIR"
fi

echo ""
echo "==========================================="
echo "  Setup Complete"
echo "==========================================="
echo ""
echo "To build with military extensions:"
echo "  cargo build --features military"
echo ""
echo "To run tests with military extensions:"
echo "  cargo test --features military"
echo ""
