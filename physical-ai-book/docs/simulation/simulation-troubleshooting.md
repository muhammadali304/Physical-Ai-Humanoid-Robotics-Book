# Troubleshooting Guide for Simulation-Specific Issues

## Overview

This guide provides solutions for common issues encountered when working with Gazebo simulation, robot models, sensors, and physics. Use this guide to diagnose and resolve simulation-specific problems.

## Common Simulation Issues and Solutions

### 1. Gazebo Won't Start or Crashes

#### Issue: Gazebo fails to launch
**Symptoms**: Gazebo crashes immediately or fails to start
**Solutions**:
1. Check for hardware acceleration:
   ```bash
   # Check if hardware acceleration is available
   glxinfo | grep -i "direct rendering"
   ```
2. Try software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gzserver your_world.world
   ```
3. Check for missing dependencies:
   ```bash
   sudo apt update && sudo apt install nvidia-prime nvidia-driver-XXX
   ```

#### Issue: Gazebo window doesn't appear
**Symptoms**: Gazebo server starts but GUI doesn't show
**Solutions**:
1. Launch GUI separately:
   ```bash
   gzclient
   ```
2. Check display settings:
   ```bash
   echo $DISPLAY
   export DISPLAY=:0
   ```

### 2. Robot Model Issues

#### Issue: Robot falls through the ground
**Symptoms**: Robot model falls through floor or other static objects
**Solutions**:
1. Check mass and inertia values in URDF:
   ```xml
   <inertial>
     <mass value="5.0"/>
     <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
   </inertial>
   ```
2. Verify collision geometries are properly defined
3. Check physics engine parameters in world file

#### Issue: Robot shakes or vibrates
**Symptoms**: Robot model oscillates or shakes when static
**Solutions**:
1. Adjust ERP (Error Reduction Parameter) and CFM (Constraint Force Mixing):
   ```xml
   <physics name="default" type="ode">
     <ode>
       <solver>
         <type>quick</type>
         <iters>10</iters>
       </solver>
       <constraints>
         <cfm>0.000001</cfm>
         <erp>0.2</erp>
       </constraints>
     </ode>
   </physics>
   ```
2. Increase solver iterations
3. Check for high mass ratios between connected links

#### Issue: Robot doesn't respond to commands
**Symptoms**: Robot ignores velocity or position commands
**Solutions**:
1. Verify joint types match controller expectations (continuous vs. revolute)
2. Check plugin configuration in URDF:
   ```xml
   <gazebo>
     <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
       <left_joint>left_wheel_joint</left_joint>
       <right_joint>right_wheel_joint</right_joint>
       <!-- Check that joint names match URDF -->
     </plugin>
   </gazebo>
   ```
3. Verify topic names match between controller and plugin

### 3. Sensor Issues

#### Issue: LIDAR returns all zeros or invalid values
**Symptoms**: `/scan` topic contains all zeros, inf, or NaN values
**Solutions**:
1. Check sensor configuration in URDF:
   ```xml
   <gazebo reference="lidar_link">
     <sensor type="ray" name="lidar_sensor">
       <ray>
         <scan>
           <horizontal>
             <samples>360</samples>
             <resolution>1</resolution>
             <min_angle>-3.14159</min_angle>
             <max_angle>3.14159</max_angle>
           </horizontal>
         </scan>
         <range>
           <min>0.1</min>
           <max>10.0</max>
           <resolution>0.01</resolution>
         </range>
       </ray>
     </sensor>
   </gazebo>
   ```
2. Verify the sensor link is properly positioned in the robot
3. Check for collisions between sensor and robot body

#### Issue: Camera image is black or distorted
**Symptoms**: Camera topic shows black image or visual artifacts
**Solutions**:
1. Check camera configuration:
   ```xml
   <gazebo reference="camera_link">
     <sensor type="camera" name="camera_sensor">
       <camera name="head_camera">
         <horizontal_fov>1.3962634</horizontal_fov>
         <image>
           <width>640</width>
           <height>480</height>
           <format>R8G8B8</format>
         </image>
         <clip>
           <near>0.05</near>
           <far>3</far>
         </clip>
       </camera>
     </sensor>
   </gazebo>
   ```
2. Verify camera link orientation and position
3. Check lighting in the environment

#### Issue: IMU readings are unstable or incorrect
**Symptoms**: IMU topic shows erratic values or incorrect gravity
**Solutions**:
1. Check IMU configuration:
   ```xml
   <gazebo reference="imu_link">
     <sensor type="imu" name="imu_sensor">
       <always_on>true</always_on>
       <update_rate>100</update_rate>
       <imu>
         <angular_velocity>
           <x>
             <noise type="gaussian">
               <mean>0.0</mean>
               <stddev>2e-4</stddev>
             </noise>
           </x>
           <!-- Similar for y and z axes -->
         </angular_velocity>
         <linear_acceleration>
           <x>
             <noise type="gaussian">
               <mean>0.0</mean>
               <stddev>1.7e-2</stddev>
             </noise>
           </x>
           <!-- Similar for y and z axes -->
         </linear_acceleration>
       </imu>
     </sensor>
   </gazebo>
   ```
2. Verify IMU link is rigidly attached (no floating joints)
3. Check that the robot is properly initialized in the world

### 4. Physics Performance Issues

#### Issue: Low FPS (under 30)
**Symptoms**: Simulation runs slowly, jerky motion
**Solutions**:
1. Simplify collision geometries:
   ```xml
   <!-- Use simple shapes instead of complex meshes -->
   <collision name="collision">
     <geometry>
       <box size="0.5 0.3 0.15"/>
     </geometry>
   </collision>
   ```
2. Adjust physics parameters:
   ```xml
   <physics name="fast_physics" type="ode">
     <max_step_size>0.01</max_step_size>  <!-- Increase for better performance -->
     <real_time_update_rate>100.0</real_time_update_rate>
     <ode>
       <solver>
         <iters>10</iters>  <!-- Reduce iterations -->
       </solver>
     </ode>
   </physics>
   ```
3. Reduce sensor update rates
4. Use fewer complex models in the scene

#### Issue: Unstable physics simulation
**Symptoms**: Objects behave erratically, explosions in simulation
**Solutions**:
1. Decrease max step size:
   ```xml
   <max_step_size>0.001</max_step_size>  <!-- Smaller for more stability -->
   ```
2. Increase solver iterations:
   ```xml
   <iters>50</iters>
   ```
3. Check for high mass ratios (should be < 1000:1)
4. Verify proper inertial properties

### 5. Communication Issues

#### Issue: Topics not connecting between Gazebo and ROS
**Symptoms**: No communication between simulation and ROS nodes
**Solutions**:
1. Check topic names match:
   ```bash
   # List all topics
   ros2 topic list
   # Check specific topic
   ros2 topic info /cmd_vel
   ```
2. Verify plugin configuration:
   ```xml
   <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
     <command_topic>cmd_vel</command_topic>  <!-- Check this matches your controller -->
     <odometry_topic>odom</odometry_topic>
   </plugin>
   ```
3. Check network configuration if running distributed

#### Issue: TF tree issues
**Symptoms**: TF transforms not available or incorrect
**Solutions**:
1. Check joint names match between URDF and controller
2. Verify robot_state_publisher is running:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher
   ```
3. Check joint_state_publisher if needed:
   ```bash
   ros2 run joint_state_publisher joint_state_publisher
   ```

## Diagnostic Tools

### 1. Gazebo Built-in Tools
```bash
# Check simulation statistics
gz stats

# View model information
gz model -m your_robot_name

# Check physics properties
gz physics
```

### 2. ROS 2 Diagnostic Commands
```bash
# Check topic health
ros2 topic list
ros2 topic info /your_topic
ros2 topic hz /your_topic

# Check service availability
ros2 service list
ros2 service info /gazebo/spawn_entity

# Check TF tree
ros2 run tf2_tools view_frames
```

### 3. Performance Monitoring
```bash
# Monitor system resources
htop
# Monitor simulation performance
ros2 run your_package fps_validator
```

## Debugging Strategies

### 1. Isolation Method
1. Start with minimal world (empty world)
2. Add robot model only
3. Add sensors one by one
4. Add controllers gradually
5. This helps identify which component causes issues

### 2. Logging and Monitoring
```xml
<!-- Enable detailed logging in URDF/plugins -->
<plugin name="debug_plugin" filename="libgazebo_ros_diff_drive.so">
  <!-- Add debug parameters -->
  <ros>
    <remapping>cmd_vel:=/debug_cmd_vel</remapping>
  </ros>
</plugin>
```

### 3. Configuration Testing
1. Test with default physics parameters first
2. Make one change at a time
3. Document what works and what doesn't
4. Keep backups of working configurations

## Common Configuration Issues

### 1. URDF/Xacro Issues
- **Missing xmlns**: Ensure proper XML namespace declarations
- **Invalid joint limits**: Check min/max values are reasonable
- **Inconsistent units**: Use consistent units (meters, radians, kg)
- **Wrong joint types**: Verify joint type matches intended motion

### 2. Gazebo Plugin Issues
- **Wrong library names**: Verify plugin library names are correct
- **Missing dependencies**: Check all required packages are installed
- **Topic name mismatches**: Verify topic names match between nodes
- **Frame name inconsistencies**: Check frame names match TF tree

### 3. Environment Issues
- **Path problems**: Ensure GAZEBO_MODEL_PATH is set correctly
- **File permissions**: Check read/write permissions on model files
- **Resource limits**: Verify sufficient memory and CPU resources

## Prevention Best Practices

### 1. Model Design
- Use consistent units throughout
- Verify mass and inertia properties
- Test with simple geometries first
- Include proper visual and collision elements

### 2. Physics Configuration
- Start with conservative parameters
- Gradually optimize for performance
- Document parameter changes and effects
- Test with various scenarios

### 3. Sensor Configuration
- Verify sensor placement and orientation
- Test individual sensors before integration
- Monitor resource usage
- Validate sensor data quality

## Advanced Troubleshooting

### 1. Gazebo Server Debugging
```bash
# Run with verbose output
gzserver --verbose your_world.world

# Run with debugging enabled
gdb --args gzserver your_world.world
```

### 2. Physics Engine Debugging
```xml
<!-- Enable physics debugging -->
<physics name="debug_physics" type="ode">
  <debug>1</debug>  <!-- Enable if supported -->
</physics>
```

### 3. Custom Diagnostic Scripts
```python
#!/usr/bin/env python3
"""
Custom diagnostic script for simulation issues
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time

class SimulationDiagnostics(Node):
    def __init__(self):
        super().__init__('simulation_diagnostics')

        self.joint_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.status_publisher = self.create_publisher(
            String, '/simulation_status', 10)

        self.last_joint_update = time.time()
        self.get_logger().info('Simulation diagnostics started')

    def joint_callback(self, msg):
        current_time = time.time()
        time_diff = current_time - self.last_joint_update

        if time_diff > 1.0:  # No joint updates for 1 second
            status_msg = String()
            status_msg.data = f'JOINT_UPDATE_ISSUE: Last update {time_diff:.2f}s ago'
            self.status_publisher.publish(status_msg)

        self.last_joint_update = current_time

def main(args=None):
    rclpy.init(args=args)
    diagnostics = SimulationDiagnostics()

    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        diagnostics.get_logger().info('Diagnostics stopped by user')

    diagnostics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quick Reference Checklist

### Before Launching Simulation
- [ ] Verify all model files exist and are readable
- [ ] Check URDF syntax with `check_urdf your_robot.urdf`
- [ ] Confirm Gazebo plugins are properly configured
- [ ] Ensure required ROS packages are installed

### If Simulation Runs Slowly
- [ ] Reduce physics update rate
- [ ] Simplify collision meshes
- [ ] Lower sensor resolutions
- [ ] Check system resources (CPU, memory)

### If Robot Behaves Unexpectedly
- [ ] Verify mass and inertia values
- [ ] Check joint limits and types
- [ ] Validate controller parameters
- [ ] Examine physics engine settings

### If Sensors Don't Work
- [ ] Confirm sensor plugins are loaded
- [ ] Check topic names match expectations
- [ ] Verify sensor placement in URDF
- [ ] Test with simple sensor configurations

## Conclusion

This troubleshooting guide covers the most common simulation-specific issues you'll encounter when working with Gazebo and ROS 2. Remember to approach problems systematically, starting with simple configurations and gradually adding complexity. Keep detailed notes of working configurations, and don't hesitate to test components in isolation when diagnosing complex issues. Regular validation and monitoring will help maintain a stable and performant simulation environment.