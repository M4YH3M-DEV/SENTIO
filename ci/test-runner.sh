#!/usr/bin/env bash
# Local CI Test Runner
# Runs all CI checks locally before pushing

set -e  # Exit on first error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Counters
PASSED=0
FAILED=0

# Helper functions
print_header() {
    echo -e "\n${YELLOW}=== $1 ===${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
    ((PASSED++))
}

print_failure() {
    echo -e "${RED}✗ $1${NC}"
    ((FAILED++))
}

run_command() {
    local cmd=$1
    local desc=$2
    
    print_header "$desc"
    if eval "$cmd"; then
        print_success "$desc"
        return 0
    else
        print_failure "$desc"
        return 1
    fi
}

# ============================================================================
# Python Checks
# ============================================================================

print_header "Python Environment"

# Check Python version
python_version=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "Python version: $python_version"

# Check required packages
required_packages=("black" "flake8" "isort" "pytest" "mypy")
for package in "${required_packages[@]}"; do
    if python3 -c "import ${package//-/_}" 2>/dev/null; then
        print_success "$package installed"
    else
        print_failure "$package not installed"
        echo "  Install with: pip install $package"
    fi
done

# ============================================================================
# Code Quality
# ============================================================================

run_command \
    "cd $PROJECT_ROOT && black --check sentio_ros2_ws/ ui/ sentio/tools/" \
    "Black Format Check" || true

run_command \
    "cd $PROJECT_ROOT && isort --check-only sentio_ros2_ws/ ui/ sentio/tools/" \
    "isort Import Sort Check" || true

run_command \
    "cd $PROJECT_ROOT && flake8 sentio_ros2_ws/ ui/ sentio/tools/ --max-line-length=100 --ignore=E203,W503" \
    "Flake8 Linting" || true

run_command \
    "cd $PROJECT_ROOT && mypy sentio_ros2_ws/ --ignore-missing-imports" \
    "MyPy Type Checking" || true

# ============================================================================
# Python Unit Tests
# ============================================================================

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_core && pytest tests/ -v" \
    "sentio_core Unit Tests" || true

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_perception && pytest tests/ -v" \
    "sentio_perception Unit Tests" || true

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_fusion && pytest tests/ -v" \
    "sentio_fusion Unit Tests" || true

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_policy && pytest tests/ -v" \
    "sentio_policy Unit Tests" || true

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_motion && pytest tests/ -v" \
    "sentio_motion Unit Tests" || true

run_command \
    "cd $PROJECT_ROOT/sentio_ros2_ws/src/sentio_demo && pytest tests/ -v" \
    "sentio_demo Unit Tests" || true

# ============================================================================
# ROS 2 Build & Tests
# ============================================================================

print_header "ROS 2 Build & Tests"

if command -v colcon &> /dev/null; then
    source /opt/ros/rolling/setup.bash
    
    run_command \
        "cd $PROJECT_ROOT/sentio_ros2_ws && colcon build --symlink-install" \
        "ROS 2 Build" || true
    
    run_command \
        "cd $PROJECT_ROOT/sentio_ros2_ws && colcon test --event-handlers console_direct+" \
        "ROS 2 Integration Tests" || true
else
    echo -e "${YELLOW}⚠ colcon not found, skipping ROS 2 tests${NC}"
fi

# ============================================================================
# Node.js UI Checks
# ============================================================================

print_header "Node.js UI Checks"

if command -v npm &> /dev/null; then
    run_command \
        "cd $PROJECT_ROOT/ui && npm ci --prefer-offline" \
        "Install UI Dependencies" || true
    
    run_command \
        "cd $PROJECT_ROOT/ui && npm run lint" \
        "ESLint Check" || true
    
    run_command \
        "cd $PROJECT_ROOT/ui && npm run type-check" \
        "TypeScript Check" || true
    
    run_command \
        "cd $PROJECT_ROOT/ui && npm test -- --coverage --watchAll=false" \
        "Jest Unit Tests" || true
    
    run_command \
        "cd $PROJECT_ROOT/ui && npm run build" \
        "Next.js Build" || true
else
    echo -e "${YELLOW}⚠ npm not found, skipping UI tests${NC}"
fi

# ============================================================================
# Firmware Build
# ============================================================================

print_header "Firmware Build"

if command -v platformio &> /dev/null; then
    run_command \
        "cd $PROJECT_ROOT/sentio/firmware/esp32_servo_bridge && platformio run -e esp32doit-devkit-v1" \
        "PlatformIO Build (Development)" || true
    
    run_command \
        "cd $PROJECT_ROOT/sentio/firmware/esp32_servo_bridge && platformio run -e esp32doit-devkit-v1-prod" \
        "PlatformIO Build (Production)" || true
else
    echo -e "${YELLOW}⚠ platformio not found, skipping firmware build${NC}"
    echo "  Install with: pip install platformio"
fi

# ============================================================================
# Summary
# ============================================================================

print_header "Test Summary"

total=$((PASSED + FAILED))
echo "Total Tests: $total"
echo -e "${GREEN}Passed: $PASSED${NC}"
echo -e "${RED}Failed: $FAILED${NC}"

if [ $FAILED -eq 0 ]; then
    echo -e "\n${GREEN}✓ All checks passed!${NC}"
    exit 0
else
    echo -e "\n${RED}✗ Some checks failed${NC}"
    echo "Review failures above and fix before committing"
    exit 1
fi
