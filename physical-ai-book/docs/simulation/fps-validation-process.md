# FPS Validation Process for Physics Simulation

## Overview

This document outlines the process for validating that Gazebo physics simulation maintains at least 30 FPS performance. Achieving this target ensures smooth, real-time simulation suitable for robot development and testing.

## Validation Requirements

### Target Performance
- **Minimum FPS**: 30 FPS for smooth real-time simulation
- **Target FPS**: 30+ FPS (higher is better)
- **Acceptable Range**: 25-30 FPS (with warnings)
- **Performance Goal**: Consistent performance over extended periods

### Validation Criteria
- Sustained performance at or above 30 FPS
- Minimal FPS fluctuations
- Stable physics behavior at target frame rate
- Acceptable real-time factor (RTF) close to 1.0

## Validation Tools

### FPS Validation Script

The `validate_fps.py` script monitors Gazebo's clock topic to calculate actual simulation frame rate:

```bash
# Run the validation script while Gazebo is running
ros2 run your_package validate_fps.py
```

The script provides:
- Real-time FPS monitoring
- Statistical analysis (average, min, max)
- Success rate calculation
- Performance alerts

### Key Metrics Tracked

1. **Current FPS**: Instantaneous frame rate
2. **Average FPS**: Average over recent samples
3. **Min/Max FPS**: Performance range
4. **Success Rate**: Percentage of samples meeting minimum requirements
5. **Real-time Factor (RTF)**: Simulation speed vs. real-time

## Validation Process

### 1. Setup Validation Environment

Before running validation:

```bash
# Launch Gazebo with your simulation
# Example:
ros2 launch your_robot_gazebo your_world.launch.py

# In a separate terminal, start the FPS validator
ros2 run your_package validate_fps.py
```

### 2. Baseline Testing

Start with simple scenarios:
- Single robot in empty world
- Verify baseline performance (should be high FPS)
- Gradually add complexity

### 3. Stress Testing

Test with increasing complexity:
- Multiple robots
- Complex environments
- Active sensors and controllers
- Dynamic obstacles

### 4. Extended Testing

Run validation for extended periods:
- 5-10 minutes for basic validation
- 30+ minutes for thorough validation
- Monitor for performance degradation over time

## Performance Analysis

### Real-time Factor (RTF) Correlation

FPS and RTF are related metrics:
- RTF = 1.0 means simulation runs at real-time speed
- Higher RTF means faster than real-time
- FPS = RTF × Real-time FPS (typically 30)

### Common Performance Patterns

1. **High FPS, Low RTF**: Simulation accurate but slow
2. **Low FPS, High RTF**: Simulation fast but potentially unstable
3. **High FPS, High RTF**: Optimal performance
4. **Low FPS, Low RTF**: Performance issues need addressing

## Validation Scenarios

### Scenario 1: Basic Robot in Simple World
```bash
# Target: >60 FPS
# Validate: Single robot with basic sensors
# Expected: High performance, baseline test
```

### Scenario 2: Robot with Full Sensor Suite
```bash
# Target: >30 FPS
# Validate: Robot with LIDAR, camera, IMU
# Expected: Performance under sensor load
```

### Scenario 3: Complex Environment
```bash
# Target: >30 FPS
# Validate: Robot in world with many obstacles
# Expected: Performance with complex collision
```

### Scenario 4: Multiple Robots
```bash
# Target: >30 FPS
# Validate: Multiple robots in same environment
# Expected: Performance with multiple dynamics
```

## Troubleshooting Low FPS

### Common Causes of Low FPS

1. **Complex Collision Meshes**: Use simpler collision geometries
2. **High Physics Update Rates**: Adjust `max_step_size` and `real_time_update_rate`
3. **Too Many Active Objects**: Reduce number of dynamic objects
4. **Resource Limitations**: Check CPU/memory usage
5. **Complex Sensors**: Reduce sensor update rates or resolution

### Optimization Strategies

1. **Adjust Physics Parameters**:
   - Increase `max_step_size` (reduces accuracy but improves performance)
   - Reduce solver iterations
   - Adjust ERP and CFM values

2. **Simplify Models**:
   - Use primitive shapes for collision
   - Reduce mesh complexity
   - Mark static objects as static

3. **Optimize Sensors**:
   - Reduce update rates
   - Lower resolution
   - Use fewer active sensors simultaneously

## Validation Results Interpretation

### Pass Criteria
- Average FPS ≥ 30
- Success rate > 95% (samples ≥ 25 FPS)
- Minimal performance fluctuations
- Stable physics behavior

### Warning Criteria
- Average FPS 25-30
- Occasional drops below 25 FPS
- Acceptable for development but needs monitoring

### Fail Criteria
- Average FPS < 25
- Frequent performance drops
- Unstable physics behavior
- RTF significantly different from target

## Automated Validation Script

The validation script (`validate_fps.py`) automatically checks performance and provides:

```python
# Example validation results
results = {
    'status': 'PASS',  # PASS, WARNING, or FAIL
    'message': 'Performance validation PASSED - Average FPS: 45.2',
    'current_fps': 42.1,
    'average_fps': 45.2,
    'min_fps': 38.5,
    'max_fps': 52.3,
    'success_rate': 100.0,  # Percentage of samples meeting minimum
    'total_samples': 1500,
    'valid_samples': 1500,
    'invalid_samples': 0
}
```

## Performance Reporting

### Validation Report Template

```
Gazebo Physics Simulation FPS Validation Results
================================================
Status: PASS
Message: Performance validation PASSED - Average FPS: 45.2
Current FPS: 42.1
Average FPS: 45.2
Min FPS: 38.5
Max FPS: 52.3
Success Rate: 100.0% (>= 25 FPS)
Total Samples: 1500
Valid Samples: 1500
Invalid Samples: 0
================================================
```

## Continuous Integration

For automated validation in CI/CD pipelines:

```bash
# Example CI validation script
#!/bin/bash
# Start Gazebo simulation
timeout 60 gzserver --verbose your_world.world &
GAZEBO_PID=$!

# Wait for simulation to start
sleep 5

# Run FPS validation for 30 seconds
timeout 30 ros2 run your_package validate_fps.py

# Stop simulation
kill $GAZEBO_PID

# Check exit code and validation results
if [ $? -eq 0 ]; then
    echo "FPS validation passed"
else
    echo "FPS validation failed"
    exit 1
fi
```

## Best Practices

1. **Regular Validation**: Test performance regularly during development
2. **Baseline Establishment**: Establish performance baselines early
3. **Incremental Testing**: Add complexity gradually while monitoring performance
4. **Documentation**: Record validation results for future reference
5. **Hardware Consideration**: Validate on target deployment hardware
6. **Scenario Testing**: Test with realistic use cases

## Conclusion

The FPS validation process ensures that Gazebo physics simulation meets the required 30 FPS target for smooth, real-time operation. Regular validation helps maintain performance standards and identifies issues before they impact development workflows. By following this validation process and using the provided tools, you can ensure that your simulation environment provides a stable, high-performance platform for robot development and testing.