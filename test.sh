#!/bin/bash

# Drone Swarm System - Test Script
# Comprehensive testing suite

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

echo "========================================"
echo "  Drone Swarm System - Test Suite"
echo "========================================"
echo ""

# Check dependencies
if ! command -v cargo &> /dev/null; then
    print_error "Cargo not found"
    exit 1
fi

TEST_TYPE=${1:-all}

run_unit_tests() {
    print_info "Running unit tests..."
    cargo test --lib
    print_success "Unit tests passed ✓"
}

run_integration_tests() {
    print_info "Running integration tests..."
    cargo test --test '*'
    print_success "Integration tests passed ✓"
}

run_doc_tests() {
    print_info "Running documentation tests..."
    cargo test --doc
    print_success "Documentation tests passed ✓"
}

run_example_tests() {
    print_info "Testing examples..."
    cargo build --examples
    print_success "Examples built successfully ✓"
}

check_formatting() {
    print_info "Checking code formatting..."
    if cargo fmt -- --check; then
        print_success "Code formatting OK ✓"
    else
        print_warning "Code formatting issues found. Run 'cargo fmt' to fix."
    fi
}

run_clippy() {
    print_info "Running Clippy (linter)..."
    if cargo clippy -- -D warnings; then
        print_success "Clippy checks passed ✓"
    else
        print_error "Clippy found issues"
        exit 1
    fi
}

check_security() {
    print_info "Running security audit..."
    if command -v cargo-audit &> /dev/null; then
        cargo audit
        print_success "Security audit passed ✓"
    else
        print_warning "cargo-audit not installed. Run: cargo install cargo-audit"
    fi
}

check_dependencies() {
    print_info "Checking dependencies..."
    cargo tree
    print_success "Dependency tree displayed ✓"
}

run_benchmarks() {
    print_info "Running benchmarks..."
    if [ -d "benches" ]; then
        cargo bench
        print_success "Benchmarks completed ✓"
    else
        print_warning "No benchmarks found"
    fi
}

coverage_report() {
    print_info "Generating coverage report..."
    if command -v cargo-tarpaulin &> /dev/null; then
        cargo tarpaulin --out Html
        print_success "Coverage report generated ✓"
    else
        print_warning "cargo-tarpaulin not installed. Run: cargo install cargo-tarpaulin"
    fi
}

# Run tests based on argument
case $TEST_TYPE in
    unit)
        run_unit_tests
        ;;
    integration)
        run_integration_tests
        ;;
    doc)
        run_doc_tests
        ;;
    examples)
        run_example_tests
        ;;
    fmt)
        check_formatting
        ;;
    clippy)
        run_clippy
        ;;
    security)
        check_security
        ;;
    deps)
        check_dependencies
        ;;
    bench)
        run_benchmarks
        ;;
    coverage)
        coverage_report
        ;;
    all)
        print_info "Running comprehensive test suite..."
        echo ""
        run_unit_tests
        echo ""
        run_integration_tests
        echo ""
        run_doc_tests
        echo ""
        run_example_tests
        echo ""
        check_formatting
        echo ""
        run_clippy
        echo ""
        check_security
        echo ""
        print_success "All tests completed successfully! 🎉"
        ;;
    *)
        print_error "Unknown test type: $TEST_TYPE"
        echo "Usage: $0 [unit|integration|doc|examples|fmt|clippy|security|deps|bench|coverage|all]"
        exit 1
        ;;
esac

echo ""
print_success "Testing completed! ✅"
