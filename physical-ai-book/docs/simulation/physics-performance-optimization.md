# Physics Simulation Performance Optimization Guide

## Overview

This guide provides strategies and techniques for optimizing physics simulation performance in Gazebo to achieve stable 30+ FPS performance. Proper optimization is essential for real-time simulation and responsive robot behavior.

## Performance Metrics

### Target Performance
- **Minimum**: 30 FPS for smooth real-time simulation
- **Recommended**: 60 FPS for high-quality real-time simulation
- **Maximum**: 100+ FPS for fast simulation (when real-time factor is not critical)

### Key Performance Indicators
- Simulation update rate (Hz)
- Real-time factor (RTF)
- CPU utilization
- Memory usage
- Physics engine step time

## Physics Engine Configuration

### ODE (Open Dynamics Engine) Optimization

The most common physics engine in Gazebo is ODE. Here's how to optimize its configuration:

#### Basic Physics Configuration
```xml
<physics name="fast_physics" type="ode">
  <!-- Smaller step size for accuracy, larger for performance -->
  <max_step_size>0.001</max_step_size>

  <!-- Target real-time update rate -->
  <real_time_update_rate>1000.0</real_time_update_rate>

  <!-- Gravity setting -->
  <gravity>0 0 -9.8</gravity>

  <ode>
    <solver>
      <!-- Use 'quick' solver for better performance -->
      <type>quick</type>
      <!-- Iterations: fewer = faster but less accurate -->
      <iters>20</iters>
      <!-- Successive Over Relaxation parameter -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <!-- Constraint Force Mixing -->
      <cfm>0.0</cfm>
      <!-- Error Reduction Parameter -->
      <erp>0.2</erp>
      <!-- Maximum correcting velocity -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <!-- Contact surface layer thickness -->
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

#### Performance vs. Accuracy Trade-offs
- **Higher `max_step_size`**: Better performance, less accuracy
- **Lower `iters`**: Better performance, less accuracy
- **Higher `sor`**: Potentially better convergence but may be unstable
- **Higher `contact_surface_layer`**: More stable contacts but less accurate

### Alternative Physics Engines

Consider using DART or Bullet for specific use cases:
- **DART**: Better for articulated robots with many joints
- **Bullet**: Better for complex collision detection

## Model Optimization Strategies

### 1. Simplified Collision Geometries

Use simplified collision meshes to reduce computation:

```xml
<!-- Good: Use simple shapes for collision -->
<link name="simple_box">
  <collision name="collision">
    <geometry>
      <box>
        <size>1 1 1</size>
      </box>
    </geometry>
  </collision>
  <visual name="visual">
    <geometry>
      <mesh filename="complex_robot_model.dae"/>
    </geometry>
  </visual>
</link>

<!-- Avoid: Complex meshes for collision -->
<link name="complex_mesh">
  <collision name="collision">
    <geometry>
      <mesh filename="complex_robot_model.dae"/>  <!-- This is expensive! -->
    </geometry>
  </collision>
</link>
```

### 2. Level of Detail (LOD)

Implement LOD for complex models:

```xml
<link name="detailed_link">
  <visual name="visual_lod_0">
    <geometry>
      <mesh filename="high_detail.dae"/>
    </geometry>
    <material>
      <script>Gazebo/Blue</script>
    </material>
  </visual>
  <visual name="visual_lod_1">
    <geometry>
      <mesh filename="medium_detail.dae"/>
    </geometry>
    <material>
      <script>Gazebo/Blue</script>
    </material>
    <!-- Enable LOD -->
    <lod>
      <max lod="1"/>
    </lod>
  </visual>
</link>
```

### 3. Efficient Mesh Formats

Use efficient mesh formats and simplify where possible:
- **DAE**: Good for complex models with textures
- **STL**: Good for collision meshes
- **OBJ**: Good balance of features and performance

## World Optimization

### 1. Terrain Optimization

For large outdoor environments:

```xml
<!-- Use heightmap with appropriate resolution -->
<model name="terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>model://my_terrain/heightmap.png</uri>
          <!-- Reduce resolution for better performance -->
          <size>100 100 20</size>
          <use_terrain_paging>false</use_terrain_paging>
        </heightmap>
      </geometry>
    </collision>
  </link>
</model>
```

### 2. Static vs Dynamic Objects

Mark static objects as static:

```xml
<!-- Static objects don't need physics calculations -->
<model name="building">
  <static>true</static>  <!-- This is key! -->
  <link name="main_link">
    <!-- ... -->
  </link>
</model>

<!-- Dynamic objects need physics calculations -->
<model name="moving_robot">
  <static>false</static>  <!-- Default behavior -->
  <link name="main_link">
    <!-- ... -->
  </link>
</model>
```

### 3. Occlusion and Culling

Remove objects that don't affect physics:

```xml
<!-- Decorative objects can be visual-only -->
<model name="decorative_tree">
  <link name="visual_only">
    <visual name="visual">
      <geometry>
        <mesh filename="tree.dae"/>
      </geometry>
    </visual>
    <!-- No collision element = no physics impact -->
  </link>
</model>
```

## Sensor Optimization

### 1. Sensor Update Rates

Optimize sensor update rates based on application needs:

```xml
<!-- For navigation: 10-20 Hz LIDAR is often sufficient -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <update_rate>10</update_rate>  <!-- Lower for better performance -->
    <!-- ... -->
  </sensor>
</gazebo>

<!-- For precise control: Higher update rates needed -->
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <update_rate>100</update_rate>  <!-- Higher for IMU -->
    <!-- ... -->
  </sensor>
</gazebo>
```

### 2. Sensor Resolution

Reduce sensor resolution where possible:

```xml
<!-- LIDAR: Reduce samples for better performance -->
<ray>
  <scan>
    <horizontal>
      <samples>360</samples>  <!-- Reduce from default 720 -->
      <resolution>1</resolution>
      <min_angle>-1.570796</min_angle>  <!-- Reduce FOV if possible -->
      <max_angle>1.570796</max_angle>
    </horizontal>
  </scan>
</ray>

<!-- Camera: Reduce resolution -->
<camera name="head_camera">
  <image>
    <width>320</width>  <!-- Reduce from 640 -->
    <height>240</height>  <!-- Reduce from 480 -->
    <format>R8G8B8</format>
  </image>
</camera>
```

## Plugin Optimization

### 1. Efficient Controller Plugins

Optimize controller update rates:

```xml
<!-- Diff drive controller: Balance update rate and performance -->
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <update_rate>30</update_rate>  <!-- Match your target FPS -->
  <!-- ... -->
</plugin>

<!-- Joint state publisher: Don't update too frequently -->
<plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
  <update_rate>30</update_rate>
  <!-- ... -->
</plugin>
```

### 2. Asynchronous Processing

Where possible, implement asynchronous sensor processing:

```xml
<!-- Example: Processing-intensive sensor plugin -->
<gazebo>
  <plugin name="complex_sensor" filename="libcomplex_sensor_plugin.so">
    <!-- Enable threading for intensive processing -->
    <threaded>true</threaded>
    <!-- Separate processing from physics update -->
    <update_rate>10</update_rate>
  </plugin>
</gazebo>
```

## Hardware Considerations

### CPU Optimization

1. **Multi-threading**: Enable multi-threading where possible
2. **Core allocation**: Assign dedicated cores for physics simulation
3. **CPU affinity**: Bind simulation processes to specific cores

### Memory Management

1. **Model caching**: Cache frequently used models
2. **Texture compression**: Use compressed textures
3. **Resource limits**: Set reasonable limits on dynamic allocations

### GPU Acceleration

While physics is CPU-bound, GPU can help with:
- Rendering optimization
- Sensor simulation (camera, LIDAR raycasting)
- Visualization

## Gazebo-Specific Optimizations

### 1. Real-Time Factor Configuration

```bash
# Launch Gazebo with specific real-time factor
gzserver --lockstep --threads 4 world_file.world

# Or in launch files
<param name="real_time_update_rate" value="1000"/>
<param name="max_step_size" value="0.001"/>
```

### 2. Threading Configuration

```xml
<!-- Enable multi-threading in physics engine -->
<physics name="multi_threaded" type="ode">
  <ode>
    <threaded_collision>true</threaded_collision>
  </ode>
</physics>
```

### 3. Scene Graph Optimization

Minimize complex scene graphs:
- Reduce number of nested models
- Combine static objects into single models when possible
- Use instancing for repeated objects

## Performance Monitoring Tools

### 1. Built-in Gazebo Tools

```bash
# Monitor simulation performance
gz stats

# Monitor specific topics
ros2 topic hz /clock
```

### 2. System Monitoring

```bash
# Monitor CPU usage
htop
# Monitor memory usage
free -h
# Monitor disk I/O if loading models from disk
iotop
```

### 3. Custom Performance Scripts

```python
#!/usr/bin/env python3
"""
Performance monitoring script for Gazebo simulation
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Publishers for performance metrics
        self.rtf_publisher = self.create_publisher(Float32, '/gazebo/real_time_factor', 10)
        self.fps_publisher = self.create_publisher(Float32, '/gazebo/fps', 10)

        # Store timing information
        self.last_time = self.get_clock().now()
        self.frame_count = 0

        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.publish_performance_metrics)

        self.get_logger().info('Performance monitor started')

    def publish_performance_metrics(self):
        # Calculate FPS and RTF
        current_time = self.get_clock().now()
        elapsed_time = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        fps = Float32()
        fps.data = float(self.frame_count) / elapsed_time if elapsed_time > 0 else 0.0

        self.fps_publisher.publish(fps)

        # Calculate RTF (simplified - in real application you'd get this from Gazebo)
        rtf = Float32()
        rtf.data = fps.data * 0.033  # Assuming 30ms per frame target
        self.rtf_publisher.publish(rtf)

        self.get_logger().info(f'FPS: {fps.data:.2f}, RTF: {rtf.data:.2f}')

        # Reset for next calculation
        self.last_time = current_time
        self.frame_count = 0

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance monitor stopped by user')

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Performance Issues and Solutions

### Issue: Low FPS (Under 30)
**Solutions**:
- Reduce `max_step_size` to improve accuracy at the cost of performance
- Simplify collision geometries
- Reduce sensor update rates
- Use fewer complex models in the scene

### Issue: Unstable Physics
**Solutions**:
- Decrease `max_step_size` to improve stability
- Increase `iters` in the solver
- Adjust `erp` and `cfm` values
- Reduce model complexity

### Issue: High CPU Usage
**Solutions**:
- Increase `max_step_size` (within stability limits)
- Reduce number of active joints/models
- Use simpler collision geometries
- Optimize plugin update rates

### Issue: Jittery Movement
**Solutions**:
- Decrease `max_step_size`
- Increase solver iterations
- Check for high mass ratios in joints
- Verify proper inertial properties

## Best Practices Summary

1. **Start Simple**: Begin with basic models and add complexity gradually
2. **Profile Regularly**: Monitor performance as you add elements
3. **Balance Accuracy and Performance**: Choose parameters based on your application needs
4. **Use Appropriate Geometries**: Simple collision shapes with detailed visuals
5. **Optimize Update Rates**: Match sensor/controller rates to application requirements
6. **Test on Target Hardware**: Always validate performance on deployment hardware
7. **Document Trade-offs**: Keep track of performance vs. accuracy decisions

## Performance Validation Checklist

- [ ] Simulation runs at 30+ FPS consistently
- [ ] Real-time factor stays close to 1.0
- [ ] Physics behavior is stable and realistic
- [ ] Sensor data is accurate and timely
- [ ] Robot controllers respond appropriately
- [ ] No excessive CPU/memory usage
- [ ] All models behave as expected

## Conclusion

Physics simulation optimization requires balancing accuracy, stability, and performance. The optimal configuration depends on your specific application requirements. Start with the default settings and adjust parameters based on your performance monitoring and application needs. Regular testing and profiling will help you achieve the best possible simulation performance for your use case.