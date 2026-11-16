#!/bin/bash
# =============================================================================
# RC Racer System Tests (local)
# =============================================================================
# Runs the same steps as the nightly workflow (.github/workflows/nightly.yml):
# builds the simulation image, then builds the workspace and runs the system
# tests inside it, headless with software rendering.
#
# Usage:
#   ./scripts/run-system-tests.sh               # all system tests
#   ./scripts/run-system-tests.sh <regex>       # only tests matching <regex>
#                                               # e.g. test_simulation_smoke
#
# The repo is mounted read-only; build artifacts stay inside the container.
# Test results are copied to test/system/results/ (gitignored).
# =============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="$REPO_ROOT/test/system/results"
SIM_IMAGE="rc-racer-simulation:latest"
TEST_FILTER="${1:-}"

command -v docker &> /dev/null || error "Docker is not installed"

info "Building simulation image (cached after first run)..."
docker compose -f "$REPO_ROOT/docker/docker-compose.yml" build simulation

mkdir -p "$RESULTS_DIR"

CTEST_ARGS=""
if [ -n "$TEST_FILTER" ]; then
    info "Running system tests matching: $TEST_FILTER"
    CTEST_ARGS="--ctest-args -R $TEST_FILTER"
else
    info "Running all system tests"
fi

# Isolated ROS domain so tests don't see other local ROS nodes
docker run --rm \
    -v "$REPO_ROOT:/ws/src/rc-racer:ro" \
    -v "$RESULTS_DIR:/results" \
    -w /ws \
    -e ROS_DOMAIN_ID=42 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e CTEST_ARGS="$CTEST_ARGS" \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    "$SIM_IMAGE" \
    bash -c '
        set -e
        colcon build --packages-up-to rc_racer_system_tests
        source install/setup.bash
        status=0
        colcon test --packages-select rc_racer_system_tests \
            --event-handlers console_direct+ $CTEST_ARGS || status=$?
        colcon test-result --verbose || status=$?
        cp -r build/rc_racer_system_tests/test_results/. /results/ 2>/dev/null || true
        chown -R "$HOST_UID:$HOST_GID" /results
        exit $status
    ' && success "System tests passed" || error "System tests failed (results in test/system/results/)"
