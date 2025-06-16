#!/bin/bash

# Comprehensive Test Runner for Hesai LiDAR Driver
# Runs unit tests, integration tests, and generates test reports

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test configuration
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACKAGE_NAME="hesai_lidar_driver"
PROJECT_ROOT="/home/seth/Documents/PersonalProjects/SethFSAI/UWEAI-LidarDriver"
LOG_DIR="${PROJECT_ROOT}/.Logs/test_results"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Create log directory
mkdir -p "${LOG_DIR}"

echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                Hesai LiDAR Driver Test Suite                 ║${NC}"
echo -e "${GREEN}║                    $(date '+%Y-%m-%d %H:%M:%S')                    ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"

# Function to print test section headers
print_section() {
    echo -e "\n${YELLOW}▶ $1${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# Function to log test results
log_result() {
    local test_name="$1"
    local result="$2"
    local log_file="$3"
    
    echo "$(date '+%Y-%m-%d %H:%M:%S') | ${test_name} | ${result}" >> "${LOG_DIR}/test_summary_${TIMESTAMP}.log"
    
    if [ -n "$log_file" ] && [ -f "$log_file" ]; then
        cp "$log_file" "${LOG_DIR}/${test_name}_${TIMESTAMP}.log"
    fi
}

# Check if we're in a catkin workspace
print_section "Environment Check"
if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    echo -e "${RED}✗ Not in a built catkin workspace. Please run 'catkin_make' first.${NC}"
    exit 1
fi

# Source the workspace
source "${WORKSPACE_DIR}/devel/setup.bash"
echo -e "${GREEN}✓ Catkin workspace sourced${NC}"

# Check ROS environment
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo -e "${RED}✗ ROS environment not set up properly${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS environment verified${NC}"

# Build the package
print_section "Building Package"
cd "${WORKSPACE_DIR}"
if catkin_make --only-pkg-with-deps "${PACKAGE_NAME}" > "${LOG_DIR}/build_${TIMESTAMP}.log" 2>&1; then
    echo -e "${GREEN}✓ Package built successfully${NC}"
    log_result "build" "PASS" "${LOG_DIR}/build_${TIMESTAMP}.log"
else
    echo -e "${RED}✗ Package build failed${NC}"
    log_result "build" "FAIL" "${LOG_DIR}/build_${TIMESTAMP}.log"
    echo "Check ${LOG_DIR}/build_${TIMESTAMP}.log for details"
    exit 1
fi

# Run unit tests
print_section "Unit Tests"

# Test packet parser
echo "Running packet parser tests..."
if rostest "${PACKAGE_NAME}" test_packet_parser --text > "${LOG_DIR}/unit_packet_parser_${TIMESTAMP}.log" 2>&1; then
    echo -e "${GREEN}✓ Packet parser tests passed${NC}"
    log_result "unit_packet_parser" "PASS" "${LOG_DIR}/unit_packet_parser_${TIMESTAMP}.log"
else
    echo -e "${RED}✗ Packet parser tests failed${NC}"
    log_result "unit_packet_parser" "FAIL" "${LOG_DIR}/unit_packet_parser_${TIMESTAMP}.log"
fi

# Test point cloud processor
echo "Running point cloud processor tests..."
if rostest "${PACKAGE_NAME}" test_point_cloud_processor --text > "${LOG_DIR}/unit_processor_${TIMESTAMP}.log" 2>&1; then
    echo -e "${GREEN}✓ Point cloud processor tests passed${NC}"
    log_result "unit_processor" "PASS" "${LOG_DIR}/unit_processor_${TIMESTAMP}.log"
else
    echo -e "${RED}✗ Point cloud processor tests failed${NC}"
    log_result "unit_processor" "FAIL" "${LOG_DIR}/unit_processor_${TIMESTAMP}.log"
fi

# Test cone detector
echo "Running cone detector tests..."
if rostest "${PACKAGE_NAME}" test_cone_detector --text > "${LOG_DIR}/unit_cone_detector_${TIMESTAMP}.log" 2>&1; then
    echo -e "${GREEN}✓ Cone detector tests passed${NC}"
    log_result "unit_cone_detector" "PASS" "${LOG_DIR}/unit_cone_detector_${TIMESTAMP}.log"
else
    echo -e "${RED}✗ Cone detector tests failed${NC}"
    log_result "unit_cone_detector" "FAIL" "${LOG_DIR}/unit_cone_detector_${TIMESTAMP}.log"
fi

# Run integration tests
print_section "Integration Tests"

echo "Running system integration tests..."
if rostest "${PACKAGE_NAME}" hesai_integration.test --text > "${LOG_DIR}/integration_${TIMESTAMP}.log" 2>&1; then
    echo -e "${GREEN}✓ Integration tests passed${NC}"
    log_result "integration" "PASS" "${LOG_DIR}/integration_${TIMESTAMP}.log"
else
    echo -e "${YELLOW}⚠ Integration tests failed (may be due to missing hardware)${NC}"
    log_result "integration" "FAIL_NO_HARDWARE" "${LOG_DIR}/integration_${TIMESTAMP}.log"
fi

# Code quality checks
print_section "Code Quality Checks"

# Check for compilation warnings
echo "Checking for compilation warnings..."
if grep -i "warning" "${LOG_DIR}/build_${TIMESTAMP}.log" > "${LOG_DIR}/warnings_${TIMESTAMP}.log"; then
    warning_count=$(wc -l < "${LOG_DIR}/warnings_${TIMESTAMP}.log")
    echo -e "${YELLOW}⚠ Found ${warning_count} compilation warnings${NC}"
    log_result "warnings" "FOUND_${warning_count}" "${LOG_DIR}/warnings_${TIMESTAMP}.log"
else
    echo -e "${GREEN}✓ No compilation warnings found${NC}"
    log_result "warnings" "NONE" ""
fi

# Static analysis (if available)
if command -v cppcheck &> /dev/null; then
    echo "Running static analysis..."
    if cppcheck --enable=all --xml --xml-version=2 "${WORKSPACE_DIR}/src/${PACKAGE_NAME}/src/" 2> "${LOG_DIR}/cppcheck_${TIMESTAMP}.xml"; then
        echo -e "${GREEN}✓ Static analysis completed${NC}"
        log_result "static_analysis" "COMPLETED" "${LOG_DIR}/cppcheck_${TIMESTAMP}.xml"
    else
        echo -e "${YELLOW}⚠ Static analysis had issues${NC}"
        log_result "static_analysis" "ISSUES" "${LOG_DIR}/cppcheck_${TIMESTAMP}.xml"
    fi
else
    echo -e "${YELLOW}⚠ cppcheck not available for static analysis${NC}"
fi

# Generate test report
print_section "Test Report Generation"

report_file="${LOG_DIR}/test_report_${TIMESTAMP}.md"
cat > "$report_file" << EOF
# Hesai LiDAR Driver Test Report

**Date**: $(date '+%Y-%m-%d %H:%M:%S')  
**Test Session**: ${TIMESTAMP}  
**Package**: ${PACKAGE_NAME}

## Test Summary

$(cat "${LOG_DIR}/test_summary_${TIMESTAMP}.log" | while read line; do
    timestamp=$(echo "$line" | cut -d'|' -f1 | xargs)
    test_name=$(echo "$line" | cut -d'|' -f2 | xargs)
    result=$(echo "$line" | cut -d'|' -f3 | xargs)
    
    if [[ "$result" == "PASS" ]]; then
        echo "- ✓ **${test_name}**: PASSED"
    elif [[ "$result" == "FAIL"* ]]; then
        echo "- ✗ **${test_name}**: FAILED"
    else
        echo "- ⚠ **${test_name}**: ${result}"
    fi
done)

## Test Details

### Build Status
$(if grep -q "build.*PASS" "${LOG_DIR}/test_summary_${TIMESTAMP}.log"; then
    echo "Package built successfully without errors."
else
    echo "Package build encountered issues. Check build log for details."
fi)

### Unit Test Results
$(if grep -q "unit.*PASS" "${LOG_DIR}/test_summary_${TIMESTAMP}.log"; then
    echo "All unit tests passed successfully."
else
    echo "Some unit tests failed. Check individual test logs for details."
fi)

### Integration Test Results
$(if grep -q "integration.*PASS" "${LOG_DIR}/test_summary_${TIMESTAMP}.log"; then
    echo "Integration tests completed successfully."
else
    echo "Integration tests failed, possibly due to missing hardware connection."
fi)

## Recommendations

$(warning_count=$(wc -l < "${LOG_DIR}/warnings_${TIMESTAMP}.log" 2>/dev/null || echo "0")
if [ "$warning_count" -gt 0 ]; then
    echo "- Address ${warning_count} compilation warnings found"
fi

if ! grep -q "integration.*PASS" "${LOG_DIR}/test_summary_${TIMESTAMP}.log"; then
    echo "- Connect Hesai LiDAR hardware for complete integration testing"
fi

echo "- Review individual test logs for detailed failure analysis")

## Files Generated

$(ls -la "${LOG_DIR}"/*_${TIMESTAMP}.* | while read file_info; do
    filename=$(echo "$file_info" | awk '{print $NF}' | xargs basename)
    echo "- \`${filename}\`"
done)

EOF

echo -e "${GREEN}✓ Test report generated: ${report_file}${NC}"

# Final summary
print_section "Test Session Complete"

total_tests=$(wc -l < "${LOG_DIR}/test_summary_${TIMESTAMP}.log")
passed_tests=$(grep -c "PASS" "${LOG_DIR}/test_summary_${TIMESTAMP}.log" || true)
failed_tests=$(grep -c "FAIL" "${LOG_DIR}/test_summary_${TIMESTAMP}.log" || true)

echo "Total tests run: ${total_tests}"
echo -e "Passed: ${GREEN}${passed_tests}${NC}"
echo -e "Failed: ${RED}${failed_tests}${NC}"

if [ "$failed_tests" -eq 0 ]; then
    echo -e "\n${GREEN}🎉 All critical tests passed!${NC}"
    exit 0
else
    echo -e "\n${YELLOW}⚠ Some tests failed. Review logs for details.${NC}"
    exit 1
fi
