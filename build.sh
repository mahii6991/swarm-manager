#!/bin/bash

# Drone Swarm System - Build Script
# This script builds the system for various targets

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Print banner
echo "========================================"
echo "  Drone Swarm System - Build Script"
echo "========================================"
echo ""

# Check if Rust is installed
if ! command -v cargo &> /dev/null; then
    print_error "Rust/Cargo not found. Please install Rust from https://rustup.rs/"
    exit 1
fi

print_info "Rust version: $(rustc --version)"
echo ""

# Parse command line arguments
BUILD_TYPE=${1:-debug}
TARGET=${2:-native}

print_info "Build type: $BUILD_TYPE"
print_info "Target: $TARGET"
echo ""

# Function to build for native (development)
build_native() {
    print_info "Building for native platform..."

    if [ "$BUILD_TYPE" = "release" ]; then
        cargo build --release
        print_success "Release build completed"
        print_info "Binary: target/release/libdrone_swarm_system.rlib"
    else
        cargo build
        print_success "Debug build completed"
        print_info "Binary: target/debug/libdrone_swarm_system.rlib"
    fi
}

# Function to build for embedded (ARM Cortex-M)
build_embedded() {
    local target_arch=$1

    print_info "Building for embedded target: $target_arch"

    # Check if target is installed
    if ! rustup target list | grep -q "$target_arch (installed)"; then
        print_warning "Target $target_arch not installed. Installing..."
        rustup target add "$target_arch"
    fi

    if [ "$BUILD_TYPE" = "release" ]; then
        cargo build --release --target "$target_arch" --no-default-features
        print_success "Embedded release build completed"
        print_info "Binary: target/$target_arch/release/"
    else
        cargo build --target "$target_arch" --no-default-features
        print_success "Embedded debug build completed"
        print_info "Binary: target/$target_arch/debug/"
    fi

    # Show binary size
    if [ "$BUILD_TYPE" = "release" ]; then
        print_info "Binary size analysis:"
        cargo size --release --target "$target_arch" -- -A
    fi
}

# Build based on target
case $TARGET in
    native)
        build_native
        ;;
    cortex-m4)
        build_embedded "thumbv7em-none-eabihf"
        ;;
    cortex-m3)
        build_embedded "thumbv7m-none-eabi"
        ;;
    cortex-m0)
        build_embedded "thumbv6m-none-eabi"
        ;;
    all)
        print_info "Building for all targets..."
        build_native
        build_embedded "thumbv7em-none-eabihf"
        build_embedded "thumbv7m-none-eabi"
        build_embedded "thumbv6m-none-eabi"
        ;;
    *)
        print_error "Unknown target: $TARGET"
        echo "Usage: $0 [debug|release] [native|cortex-m4|cortex-m3|cortex-m0|all]"
        exit 1
        ;;
esac

echo ""
print_success "Build completed successfully! 🚀"
