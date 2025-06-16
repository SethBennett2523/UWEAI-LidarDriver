# Test Data Directory

This directory contains sample data files for testing the Hesai LiDAR driver.

## Contents

- `sample_packets/` - Sample UDP packet data from Hesai Pandar 40p
- `calibration/` - Test calibration files
- `point_clouds/` - Sample point cloud data for testing processing algorithms
- `expected_results/` - Expected outputs for regression testing

## Usage

Test data files are used by the unit and integration tests to verify:
- Packet parsing accuracy
- Point cloud processing correctness
- Cone detection algorithm performance
- End-to-end system functionality

## Data Format

- Packet data: Binary files containing raw UDP packet data
- Point clouds: PCL-compatible .pcd files
- Calibration: CSV or XML files with sensor calibration parameters
- Results: JSON files with expected detection outcomes

## Generating Test Data

To capture new test data from actual hardware:

```bash
# Capture packet data (requires hardware)
rosrun hesai_lidar_driver capture_test_data.py --duration 10 --output sample_packets/

# Generate synthetic cone data for testing
rosrun hesai_lidar_driver generate_test_cones.py --output point_clouds/synthetic_cones.pcd
```

## Privacy and Safety

- Test data should not contain identifiable information
- Ensure test scenarios cover edge cases and failure modes
- Regular data should be representative of actual operational conditions
