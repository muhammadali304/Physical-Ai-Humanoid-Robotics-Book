# Custom World Creation with SDF Format and Obstacles

## Overview

This guide provides comprehensive instructions for creating custom simulation environments in Gazebo using the Simulation Description Format (SDF). You'll learn to design complex worlds with various obstacles, physics properties, and environmental elements for realistic robotics simulation.

## Understanding SDF (Simulation Description Format)

### What is SDF?
SDF (Simulation Description Format) is an XML-based format used to describe environments, robots, and objects in Gazebo. It provides a flexible way to define:
- World properties and physics
- Models and their properties
- Terrain and obstacles
- Lighting and environmental effects

### SDF vs URDF
- **SDF**: Used for complete simulation environments and world descriptions
- **URDF**: Used primarily for robot descriptions
- **Relationship**: URDF can be embedded in SDF, but SDF is more comprehensive for environment description

## Basic SDF World Structure

### Minimal SDF World File

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- World properties and entities go here -->

    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Default light source -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### World Properties
- **gravity**: Default is `0 0 -9.8` (m/s²)
- **physics**: Defines the physics engine (ODE, Bullet, SimBody)
- **max_step_size**: Simulation time step
- **real_time_factor**: Target simulation speed relative to real time

## Creating Basic Obstacles

### Static Obstacles

#### Simple Box Obstacle

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="obstacle_world">
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Static box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### Cylinder and Sphere Obstacles

```xml
<!-- Cylinder obstacle -->
<model name="cylinder_obstacle">
  <pose>0 2 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>2</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>2</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>

<!-- Sphere obstacle -->
<model name="sphere_obstacle">
  <pose>-2 0 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Advanced Obstacle Types

### Wall and Maze Creation

```xml
<!-- Wall obstacle -->
<model name="wall_1">
  <pose>0 -3 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>6 0.2 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>6 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </visual>
  </link>
</model>

<!-- Multiple walls to create a simple maze -->
<model name="maze_wall_1">
  <pose>-2 0 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.2 3 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.2 3 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.4 0.4 0.4 1</diffuse>
      </material>
    </visual>
  </link>
</model>

<model name="maze_wall_2">
  <pose>2 1 1 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.4 0.4 0.4 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Custom Obstacle with Physics Properties

```xml
<!-- Obstacle with specific friction and restitution -->
<model name="physics_obstacle">
  <pose>3 3 1 0 0 0</pose>
  <static>false</static> <!-- Non-static = can move/interact -->
  <link name="link">
    <!-- Collision properties -->
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.5</mu> <!-- Friction coefficient -->
            <mu2>0.5</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.3</restitution_coefficient> <!-- Bounciness -->
          <threshold>100000</threshold>
        </bounce>
        <contact>
          <ode>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>1e+13</kp>
            <kd>1</kd>
          </ode>
        </contact>
      </surface>
    </collision>

    <!-- Visual properties -->
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.2 1</ambient>
        <diffuse>0.8 0.6 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Creating Complex Environments

### Room with Furniture

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="room_world">
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room walls -->
    <model name="wall_north">
      <pose>0 5 2.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table obstacle -->
    <model name="table">
      <pose>0 0 0.45 0 0 0</pose>
      <static>true</static>
      <link name="top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Table legs -->
      <link name="leg1">
        <pose>-0.65 -0.35 0.225 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg2">
        <pose>0.65 -0.35 0.225 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <joint name="top_to_leg1" type="fixed">
        <parent>top</parent>
        <child>leg1</child>
        <pose>-0.65 -0.35 0.225 0 0 0</pose>
      </joint>

      <joint name="top_to_leg2" type="fixed">
        <parent>top</parent>
        <child>leg2</child>
        <pose>0.65 -0.35 0.225 0 0 0</pose>
      </joint>
    </model>
  </world>
</sdf>
```

## SDF Physics Configuration

### Advanced Physics Settings

```xml
<physics name="advanced_physics" type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- ODE-specific parameters -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Working with Models from Model Database

### Including Standard Models

```xml
<!-- Include a model from the Gazebo model database -->
<include>
  <name>my_cafe_table</name>
  <pose>1 1 0 0 0 0</pose>
  <uri>model://cafe_table</uri>
</include>

<include>
  <name>my_sonar_sensor</name>
  <pose>0 0 0.5 0 0 0</pose>
  <uri>model://sonar</uri>
</include>
```

## Creating and Using Custom Models

### Model Directory Structure

```
~/.gazebo/models/my_custom_obstacle/
├── model.config      # Model metadata
└── model.sdf         # Model definition
```

### Model Configuration File (model.config)

```xml
<?xml version="1.0"?>
<model>
  <name>My Custom Obstacle</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>

  <description>
    A custom obstacle for robotics simulation
  </description>
</model>
```

## Advanced Features

### Dynamic Obstacles

```xml
<!-- Moving obstacle with joint actuation -->
<model name="moving_obstacle">
  <pose>0 0 1 0 0 0</pose>
  <link name="base_link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.2</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.2</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>

  <!-- Joint to allow movement -->
  <joint name="slider_joint" type="prismatic">
    <parent>world</parent>
    <child>base_link</child>
    <axis>
      <xyz>1 0 0</xyz>
      <limit>
        <lower>-5</lower>
        <upper>5</upper>
      </limit>
    </axis>
  </joint>
</model>
```

### Lighting and Environmental Effects

```xml
<!-- Custom lighting -->
<light name="room_light" type="point">
  <pose>0 0 5 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <cast_shadows>true</cast_shadows>
</light>

<!-- Fog effect -->
<scene>
  <fog type="linear">
    <color>0.8 0.8 0.8</color>
    <density>0.01</density>
    <start>1</start>
    <end>100</end>
  </fog>
</scene>
```

## Best Practices for SDF Worlds

### 1. Performance Optimization
- Use simple geometric shapes for collision detection
- Reduce polygon count in visual meshes
- Use appropriate physics update rates
- Keep static objects truly static

### 2. Organization
- Group related objects by function
- Use meaningful names for models and links
- Comment complex sections
- Use relative positioning when possible

### 3. Testing and Validation
- Start simple and add complexity gradually
- Test physics behavior with real robots
- Verify collision properties work as expected
- Check visual appearance at different angles

## Common SDF Elements Reference

### Model Properties
- `<pose>`: Position and orientation (x y z roll pitch yaw)
- `<static>`: Whether the model moves
- `<link>`: Individual rigid body parts
- `<joint>`: Connections between links

### Physics Properties
- `<collision>`: Collision geometry and properties
- `<visual>`: Visual appearance
- `<inertial>`: Mass properties for dynamics
- `<surface>`: Friction, bounce, and contact properties

### Geometry Types
- `<box>`: Rectangular prism
- `<cylinder>`: Cylinder
- `<sphere>`: Sphere
- `<mesh>`: Custom mesh files
- `<plane>`: Infinite plane

## Troubleshooting Common Issues

### Issues with Collisions
**Problem**: Objects fall through or don't collide properly
**Solution**:
- Check collision geometry is properly defined
- Verify objects aren't initially penetrating
- Adjust physics parameters

### Issues with Visuals
**Problem**: Objects not appearing or appearing incorrectly
**Solution**:
- Check visual geometry matches collision geometry
- Verify material properties are correct
- Ensure pose values are reasonable

### Performance Issues
**Problem**: Simulation running slowly
**Solution**:
- Reduce physics update rate if possible
- Simplify collision geometry
- Use fewer complex models

## Testing Your Custom World

### Loading and Testing

```bash
# Load your custom world in Gazebo
gazebo ~/.gazebo/worlds/your_custom_world.sdf

# Or launch with ROS 2
ros2 launch gazebo_ros empty_world.launch.py world_name:=$HOME/.gazebo/worlds/your_custom_world.sdf
```

### Verification Steps
1. Open Gazebo and load your world
2. Check all obstacles appear correctly
3. Test robot navigation around obstacles
4. Verify physics behavior is realistic
5. Check for any error messages in console

## Example Complete World File

Here's a complete example that combines multiple concepts:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="robotics_test_world">
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Obstacles -->
    <!-- Center pillar -->
    <model name="center_pillar">
      <pose>0 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Perimeter walls -->
    <model name="wall_north">
      <pose>0 3 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <pose>0 -3 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_east">
      <pose>3 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_west">
      <pose>-3 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Next Steps

After creating your custom SDF world:

1. **Test the environment** in Gazebo to ensure all obstacles behave correctly
2. **Integrate with robot models** to test navigation and interaction
3. **Optimize performance** based on testing results
4. **Document the environment** for others to use
5. **Share with the community** if appropriate

## Additional Resources

- [Gazebo SDF Documentation](http://sdformat.org/)
- [SDF Tutorial](http://gazebosim.org/tutorials?tut=ros2_overview_with_gazebo)
- [Model Database](http://models.gazebosim.org/)
- [Physics Tuning Guide](http://gazebosim.org/tutorials?tut=physics_tuning)

---

**Note**: SDF is a powerful format that allows for complex simulation environments. Start with simple shapes and gradually add complexity as you become more familiar with the format and its capabilities.