# Path Planning Algorithms Implementation

## Overview

This guide provides comprehensive instructions for implementing path planning algorithms in ROS 2 Navigation2. Path planning is a critical component of robot navigation that finds optimal or feasible paths from a start position to a goal while avoiding obstacles. This guide covers both built-in algorithms and custom implementations.

## Understanding Path Planning

### Types of Path Planning Algorithms

1. **Global Path Planning**: Finds optimal path from start to goal using known map
2. **Local Path Planning**: Finds immediate path to follow global plan while avoiding dynamic obstacles
3. **Anytime Algorithms**: Provide initial solution quickly and improve over time

### Common Algorithms
- **A* (A-star)**: Optimal path with heuristic
- **Dijkstra**: Optimal path without heuristic
- **NavFn**: Fast approximate path planning
- **Theta***: Any-angle path planning
- **RRT**: Rapidly-exploring Random Tree (for complex environments)

## Built-in Navigation2 Planners

### 1. NavFn Planner Configuration

NavFn is the default global planner in Navigation2, based on Dijkstra's algorithm:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5          # Distance to consider goal reached
      use_astar: false        # Use A* instead of Dijkstra
      allow_unknown: true     # Allow planning through unknown space
      visualize_potential: false  # Publish potential field for visualization
```

### 2. A* Planner Configuration

For A* implementation (if available in your Navigation2 version):

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true         # Use A* heuristic
      allow_unknown: false    # Don't plan through unknown space
      use_quadratic: true     # Use quadratic approximation
      use_dijkstra: false     # Use Dijkstra instead of A*
      use_grid_path: false    # Output grid path instead of smooth path
```

### 3. Smoother Configuration

Path smoothing for better navigation:

```yaml
smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.9          # Weight for smoothing
      w_data: 0.1            # Weight for data fitting
```

## Custom Path Planning Implementation

### 1. A* Algorithm Implementation

Create a custom A* planner node `custom_astar_planner.py`:

```python
#!/usr/bin/env python3
"""
Custom A* Path Planner for ROS 2 Navigation2
"""
import math
from typing import List, Tuple, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header


class Node:
    """A* search node"""
    def __init__(self, x, y, cost=0.0, heuristic=0.0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost  # g-cost (actual cost from start)
        self.heuristic = heuristic  # h-cost (estimated cost to goal)
        self.f_cost = cost + heuristic  # f-cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('custom_astar_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        # Costmap service client
        self.costmap_client = self.create_client(
            GetCostmap, 'get_costmap', callback_group=ReentrantCallbackGroup())

        # Wait for costmap service
        while not self.costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Costmap service not available, waiting...')

        # Path publisher
        self.path_publisher = self.create_publisher(Path, 'plan', 1)

        self.get_logger().info('Custom A* Planner initialized')

    def execute_callback(self, goal_handle):
        """Execute the path planning action"""
        self.get_logger().info('Executing A* path planning...')

        # Get current costmap
        costmap = self.get_current_costmap()
        if costmap is None:
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert goal to grid coordinates
        start_grid = self.world_to_grid(
            goal_handle.request.start.pose.position.x,
            goal_handle.request.start.pose.position.y,
            costmap)

        goal_grid = self.world_to_grid(
            goal_handle.request.goal.pose.position.x,
            goal_handle.request.goal.pose.position.y,
            costmap)

        # Plan path using A*
        path_grid = self.astar_plan(start_grid, goal_grid, costmap)

        if path_grid is None:
            self.get_logger().warn('No path found')
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert grid path to world coordinates
        path_world = self.grid_path_to_world(path_grid, costmap)

        # Create result
        result = ComputePathToPose.Result()
        result.path = path_world
        goal_handle.succeed()

        # Publish path
        self.path_publisher.publish(path_world)

        self.get_logger().info('A* path planning completed successfully')
        return result

    def get_current_costmap(self):
        """Get current costmap from service"""
        request = GetCostmap.Request()
        future = self.costmap_client.call_async(request)

        # Wait for response (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().map
        else:
            self.get_logger().error('Failed to get costmap')
            return None

    def world_to_grid(self, x_world, y_world, costmap):
        """Convert world coordinates to grid coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        grid_x = int((x_world - origin_x) / resolution)
        grid_y = int((y_world - origin_y) / resolution)

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, costmap):
        """Convert grid coordinates to world coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        world_x = grid_x * resolution + origin_x + resolution / 2.0
        world_y = grid_y * resolution + origin_y + resolution / 2.0

        return (world_x, world_y)

    def grid_path_to_world(self, grid_path, costmap):
        """Convert grid path to world path"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = costmap.header.frame_id

        for grid_x, grid_y in grid_path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, costmap)

            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = costmap.header.frame_id
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path.poses.append(pose)

        return path

    def heuristic(self, node, goal):
        """Calculate heuristic (Manhattan distance)"""
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def get_neighbors(self, node, costmap):
        """Get valid neighbors for a node"""
        neighbors = []
        grid_width = costmap.info.width
        grid_height = costmap.info.height

        # 8-directional movement (can be changed to 4-directional)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        for dx, dy in directions:
            new_x = node[0] + dx
            new_y = node[1] + dy

            # Check bounds
            if 0 <= new_x < grid_width and 0 <= new_y < grid_height:
                # Check if cell is free (cost < 50 to avoid unknown areas)
                cost_index = new_y * grid_width + new_x
                if costmap.data[cost_index] < 50:  # Free space threshold
                    neighbors.append((new_x, new_y))

        return neighbors

    def astar_plan(self, start, goal, costmap):
        """A* path planning algorithm"""
        import heapq

        # Check if start and goal are valid
        start_cost_idx = start[1] * costmap.info.width + start[0]
        goal_cost_idx = goal[1] * costmap.info.width + goal[0]

        if (start_cost_idx >= len(costmap.data) or goal_cost_idx >= len(costmap.data) or
            costmap.data[start_cost_idx] >= 50 or costmap.data[goal_cost_idx] >= 50):
            self.get_logger().warn('Start or goal position is in obstacle')
            return None

        # Initialize open and closed sets
        open_set = []
        closed_set = set()

        # Create start node
        start_node = (start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_set, (start_node[2] + start_node[3], start_node))

        # Track costs
        g_costs = {start: 0}
        parents = {start: None}

        while open_set:
            current_f, current = heapq.heappop(open_set)
            current_pos = (current[0], current[1])

            # Check if we reached the goal
            if current_pos == goal:
                # Reconstruct path
                path = []
                current_path = current_pos
                while current_path is not None:
                    path.append(current_path)
                    current_path = parents[current_path]
                path.reverse()
                return path

            closed_set.add(current_pos)

            # Get neighbors
            for neighbor in self.get_neighbors(current_pos, costmap):
                if neighbor in closed_set:
                    continue

                # Calculate tentative g-cost
                tentative_g = g_costs[current_pos] + 1  # Simple distance

                # If this path is better than previous one
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    h_cost = self.heuristic(neighbor, goal)
                    f_cost = tentative_g + h_cost

                    # Add to open set
                    node_info = (neighbor[0], neighbor[1], tentative_g, h_cost)
                    heapq.heappush(open_set, (f_cost, node_info))
                    parents[neighbor] = current_pos

        # No path found
        self.get_logger().warn('A* failed to find a path')
        return None


def main(args=None):
    rclpy.init(args=args)

    planner = AStarPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        planner.get_logger().info('A* planner interrupted')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Dijkstra Algorithm Implementation

Create a custom Dijkstra planner `custom_dijkstra_planner.py`:

```python
#!/usr/bin/env python3
"""
Custom Dijkstra Path Planner for ROS 2 Navigation2
"""
import math
from typing import List, Tuple, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
import heapq


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('custom_dijkstra_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        # Costmap service client
        self.costmap_client = self.create_client(
            GetCostmap, 'get_costmap', callback_group=ReentrantCallbackGroup())

        # Wait for costmap service
        while not self.costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Costmap service not available, waiting...')

        # Path publisher
        self.path_publisher = self.create_publisher(Path, 'plan', 1)

        self.get_logger().info('Custom Dijkstra Planner initialized')

    def execute_callback(self, goal_handle):
        """Execute the path planning action"""
        self.get_logger().info('Executing Dijkstra path planning...')

        # Get current costmap
        costmap = self.get_current_costmap()
        if costmap is None:
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert goal to grid coordinates
        start_grid = self.world_to_grid(
            goal_handle.request.start.pose.position.x,
            goal_handle.request.start.pose.position.y,
            costmap)

        goal_grid = self.world_to_grid(
            goal_handle.request.goal.pose.position.x,
            goal_handle.request.goal.pose.position.y,
            costmap)

        # Plan path using Dijkstra
        path_grid = self.dijkstra_plan(start_grid, goal_grid, costmap)

        if path_grid is None:
            self.get_logger().warn('No path found')
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert grid path to world coordinates
        path_world = self.grid_path_to_world(path_grid, costmap)

        # Create result
        result = ComputePathToPose.Result()
        result.path = path_world
        goal_handle.succeed()

        # Publish path
        self.path_publisher.publish(path_world)

        self.get_logger().info('Dijkstra path planning completed successfully')
        return result

    def get_current_costmap(self):
        """Get current costmap from service"""
        request = GetCostmap.Request()
        future = self.costmap_client.call_async(request)

        # Wait for response (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().map
        else:
            self.get_logger().error('Failed to get costmap')
            return None

    def world_to_grid(self, x_world, y_world, costmap):
        """Convert world coordinates to grid coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        grid_x = int((x_world - origin_x) / resolution)
        grid_y = int((y_world - origin_y) / resolution)

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, costmap):
        """Convert grid coordinates to world coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        world_x = grid_x * resolution + origin_x + resolution / 2.0
        world_y = grid_y * resolution + origin_y + resolution / 2.0

        return (world_x, world_y)

    def grid_path_to_world(self, grid_path, costmap):
        """Convert grid path to world path"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = costmap.header.frame_id

        for grid_x, grid_y in grid_path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, costmap)

            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = costmap.header.frame_id
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path.poses.append(pose)

        return path

    def get_neighbors(self, node, costmap):
        """Get valid neighbors for a node"""
        neighbors = []
        grid_width = costmap.info.width
        grid_height = costmap.info.height

        # 8-directional movement
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        for dx, dy in directions:
            new_x = node[0] + dx
            new_y = node[1] + dy

            # Check bounds
            if 0 <= new_x < grid_width and 0 <= new_y < grid_height:
                # Check if cell is free (cost < 50 to avoid unknown areas)
                cost_index = new_y * grid_width + new_x
                if costmap.data[cost_index] < 50:  # Free space threshold
                    # Calculate cost based on direction and terrain
                    if abs(dx) + abs(dy) == 2:  # Diagonal move
                        cost = math.sqrt(2)  # Diagonal distance
                    else:  # Horizontal/vertical move
                        cost = 1.0

                    neighbors.append(((new_x, new_y), cost))

        return neighbors

    def dijkstra_plan(self, start, goal, costmap):
        """Dijkstra path planning algorithm"""
        # Check if start and goal are valid
        start_cost_idx = start[1] * costmap.info.width + start[0]
        goal_cost_idx = goal[1] * costmap.info.width + goal[0]

        if (start_cost_idx >= len(costmap.data) or goal_cost_idx >= len(costmap.data) or
            costmap.data[start_cost_idx] >= 50 or costmap.data[goal_cost_idx] >= 50):
            self.get_logger().warn('Start or goal position is in obstacle')
            return None

        # Initialize distances and priority queue
        distances = {}
        previous = {}
        pq = []

        # Set all distances to infinity except start
        for y in range(costmap.info.height):
            for x in range(costmap.info.width):
                pos = (x, y)
                distances[pos] = float('inf')

        # Set start distance to 0
        distances[start] = 0
        heapq.heappush(pq, (0, start))

        while pq:
            current_dist, current = heapq.heappop(pq)

            # If we reached the goal, reconstruct path
            if current == goal:
                path = []
                current_path = goal
                while current_path is not None:
                    path.append(current_path)
                    current_path = previous.get(current_path)
                path.reverse()
                return path

            # If we've already processed this node with a shorter distance, skip
            if current_dist > distances[current]:
                continue

            # Explore neighbors
            for neighbor_pos, edge_cost in self.get_neighbors(current, costmap):
                new_dist = distances[current] + edge_cost

                if new_dist < distances[neighbor_pos]:
                    distances[neighbor_pos] = new_dist
                    previous[neighbor_pos] = current
                    heapq.heappush(pq, (new_dist, neighbor_pos))

        # No path found
        self.get_logger().warn('Dijkstra failed to find a path')
        return None


def main(args=None):
    rclpy.init(args=args)

    planner = DijkstraPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        planner.get_logger().info('Dijkstra planner interrupted')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Theta* Any-Angle Path Planning

Create a custom Theta* planner for any-angle path planning:

```python
#!/usr/bin/env python3
"""
Custom Theta* Any-Angle Path Planner for ROS 2 Navigation2
"""
import math
from typing import List, Tuple, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
import heapq


class ThetaStarPlanner(Node):
    def __init__(self):
        super().__init__('custom_theta_star_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        # Costmap service client
        self.costmap_client = self.create_client(
            GetCostmap, 'get_costmap', callback_group=ReentrantCallbackGroup())

        # Wait for costmap service
        while not self.costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Costmap service not available, waiting...')

        # Path publisher
        self.path_publisher = self.create_publisher(Path, 'plan', 1)

        self.get_logger().info('Custom Theta* Planner initialized')

    def execute_callback(self, goal_handle):
        """Execute the path planning action"""
        self.get_logger().info('Executing Theta* path planning...')

        # Get current costmap
        costmap = self.get_current_costmap()
        if costmap is None:
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert goal to grid coordinates
        start_grid = self.world_to_grid(
            goal_handle.request.start.pose.position.x,
            goal_handle.request.start.pose.position.y,
            costmap)

        goal_grid = self.world_to_grid(
            goal_handle.request.goal.pose.position.x,
            goal_handle.request.goal.pose.position.y,
            costmap)

        # Plan path using Theta*
        path_grid = self.theta_star_plan(start_grid, goal_grid, costmap)

        if path_grid is None:
            self.get_logger().warn('No path found')
            goal_handle.abort()
            return ComputePathToPose.Result()

        # Convert grid path to world coordinates
        path_world = self.grid_path_to_world(path_grid, costmap)

        # Create result
        result = ComputePathToPose.Result()
        result.path = path_world
        goal_handle.succeed()

        # Publish path
        self.path_publisher.publish(path_world)

        self.get_logger().info('Theta* path planning completed successfully')
        return result

    def get_current_costmap(self):
        """Get current costmap from service"""
        request = GetCostmap.Request()
        future = self.costmap_client.call_async(request)

        # Wait for response (with timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().map
        else:
            self.get_logger().error('Failed to get costmap')
            return None

    def world_to_grid(self, x_world, y_world, costmap):
        """Convert world coordinates to grid coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        grid_x = int((x_world - origin_x) / resolution)
        grid_y = int((y_world - origin_y) / resolution)

        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y, costmap):
        """Convert grid coordinates to world coordinates"""
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution

        world_x = grid_x * resolution + origin_x + resolution / 2.0
        world_y = grid_y * resolution + origin_y + resolution / 2.0

        return (world_x, world_y)

    def grid_path_to_world(self, grid_path, costmap):
        """Convert grid path to world path"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = costmap.header.frame_id

        for grid_x, grid_y in grid_path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, costmap)

            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = costmap.header.frame_id
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path.poses.append(pose)

        return path

    def line_of_sight(self, pos1, pos2, costmap):
        """Check if there's a line of sight between two positions"""
        x0, y0 = pos1
        x1, y1 = pos2

        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            # Check if current point is an obstacle
            if x >= 0 and x < costmap.info.width and y >= 0 and y < costmap.info.height:
                cost_idx = y * costmap.info.width + x
                if costmap.data[cost_idx] >= 50:  # Obstacle
                    return False
            else:
                return False  # Out of bounds

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    def heuristic(self, pos1, pos2):
        """Calculate Euclidean distance heuristic"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def get_neighbors(self, node, costmap):
        """Get valid neighbors for a node"""
        neighbors = []
        grid_width = costmap.info.width
        grid_height = costmap.info.height

        # 8-directional movement
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        for dx, dy in directions:
            new_x = node[0] + dx
            new_y = node[1] + dy

            # Check bounds
            if 0 <= new_x < grid_width and 0 <= new_y < grid_height:
                # Check if cell is free (cost < 50 to avoid unknown areas)
                cost_index = new_y * grid_width + new_x
                if costmap.data[cost_index] < 50:  # Free space threshold
                    # Calculate cost based on direction
                    if abs(dx) + abs(dy) == 2:  # Diagonal move
                        cost = math.sqrt(2)
                    else:  # Horizontal/vertical move
                        cost = 1.0

                    neighbors.append(((new_x, new_y), cost))

        return neighbors

    def theta_star_plan(self, start, goal, costmap):
        """Theta* path planning algorithm"""
        # Check if start and goal are valid
        start_cost_idx = start[1] * costmap.info.width + start[0]
        goal_cost_idx = goal[1] * costmap.info.width + goal[0]

        if (start_cost_idx >= len(costmap.data) or goal_cost_idx >= len(costmap.data) or
            costmap.data[start_cost_idx] >= 50 or costmap.data[goal_cost_idx] >= 50):
            self.get_logger().warn('Start or goal position is in obstacle')
            return None

        # Initialize open and closed sets
        open_set = []
        closed_set = set()

        # Initialize g-costs and parents
        g_costs = {start: 0}
        parents = {start: start}

        # Add start node to open set
        f_cost = self.heuristic(start, goal)
        heapq.heappush(open_set, (f_cost, start))

        while open_set:
            current_f, current = heapq.heappop(open_set)
            current = tuple(current)  # Ensure tuple type

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == goal:
                # Reconstruct path by following parent pointers
                path = []
                current_path = current
                while current_path != parents[current_path]:
                    path.append(current_path)
                    current_path = parents[current_path]
                path.append(start)
                path.reverse()
                return path

            # Get neighbors
            for neighbor, edge_cost in self.get_neighbors(current, costmap):
                if neighbor in closed_set:
                    continue

                # Calculate tentative g-cost
                parent = parents[current]

                # Check line of sight from parent to neighbor
                if self.line_of_sight(parent, neighbor, costmap):
                    # Update through parent
                    tentative_g = g_costs[parent] + self.heuristic(parent, neighbor)
                else:
                    # Update through current
                    tentative_g = g_costs[current] + edge_cost

                # If this path is better than previous one
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    parents[neighbor] = parent if self.line_of_sight(parent, neighbor, costmap) else current
                    f_cost = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_cost, neighbor))

        # No path found
        self.get_logger().warn('Theta* failed to find a path')
        return None


def main(args=None):
    rclpy.init(args=args)

    planner = ThetaStarPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        planner.get_logger().info('Theta* planner interrupted')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Path Planning Features

### 1. Dynamic Path Planning with Obstacle Avoidance

```yaml
# Configuration for dynamic path planning
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    use_sim_time: false
    planner_plugins: ["GridBased", "DynamicPlanner"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false

    DynamicPlanner:
      plugin: "nav2_dynamic_planner::DynamicPlanner"
      max_velocity_x: 0.5
      min_velocity_x: 0.1
      max_velocity_theta: 1.0
      min_velocity_theta: 0.1
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
      max_obstacle_height: 2.0
      min_obstacle_height: 0.0
```

### 2. Multi-Goal Path Planning

```python
#!/usr/bin/env python3
"""
Multi-Goal Path Planner
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
import math


class MultiGoalPlanner(Node):
    def __init__(self):
        super().__init__('multi_goal_planner')

        # Action server for multi-goal path computation
        self._action_server = ActionServer(
            self,
            ComputePathThroughPoses,
            'compute_path_through_poses',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Multi-Goal Planner initialized')

    def execute_callback(self, goal_handle):
        """Execute multi-goal path planning"""
        self.get_logger().info('Executing multi-goal path planning...')

        # For simplicity, connect all goals with basic path planning
        # In practice, you'd implement more sophisticated multi-goal planning
        total_path = []

        # Start from initial pose
        current_pose = goal_handle.request.start
        total_path.append(current_pose)

        # Plan to each goal sequentially
        for goal_pose in goal_handle.request.poses:
            # Here you would call your path planning algorithm
            # For now, we'll just add the goal directly (not realistic)
            # In practice, implement proper path planning between waypoints
            path_segment = self.plan_between_poses(current_pose, goal_pose)
            total_path.extend(path_segment.poses[1:])  # Skip first pose to avoid duplication
            current_pose = goal_pose

        # Create result
        result = ComputePathThroughPoses.Result()
        result.path.header = Header()
        result.path.header.stamp = self.get_clock().now().to_msg()
        result.path.header.frame_id = "map"
        result.path.poses = total_path

        goal_handle.succeed()
        return result

    def plan_between_poses(self, start, goal):
        """Plan path between two poses (simplified implementation)"""
        from nav_msgs.msg import Path
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        # Add start and goal as simple path (in practice, use actual path planning)
        start_stamped = PoseStamped()
        start_stamped.header = path.header
        start_stamped.pose = start.pose
        path.poses.append(start_stamped)

        goal_stamped = PoseStamped()
        goal_stamped.header = path.header
        goal_stamped.pose = goal.pose
        path.poses.append(goal_stamped)

        return path


def main(args=None):
    rclpy.init(args=args)
    planner = MultiGoalPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Multi-goal planner interrupted')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Optimization

### 1. Path Planning Performance Tuning

```yaml
# Optimized path planning configuration
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0  # Higher frequency for faster planning
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.25          # Tighter tolerance for better accuracy
      use_astar: true          # Use A* for faster convergence
      allow_unknown: false     # Don't plan through unknown space
      visualize_potential: false  # Disable visualization for performance
      potential_scale: 3.0     # Scale for potential field
      obstacle_scale: 1.0      # Scale for obstacle inflation
      neutral_scale: 0.5       # Scale for neutral areas
```

### 2. Memory Management

```yaml
# Memory-optimized path planning
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # Lower frequency to reduce CPU usage
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 1.0           # Larger tolerance for faster planning
      use_astar: true
      allow_unknown: true
      memory_limit: 100        # Limit memory usage for large maps
```

## Path Planning Quality Metrics

### 1. Path Quality Assessment

```python
#!/usr/bin/env python3
"""
Path Quality Assessment Tool
"""
import math
from typing import List
from geometry_msgs.msg import PoseStamped


def calculate_path_metrics(path: List[PoseStamped]):
    """Calculate various path quality metrics"""
    if len(path) < 2:
        return {
            'length': 0,
            'smoothness': 0,
            'curvature': 0,
            'turning_points': 0
        }

    # Calculate path length
    length = 0
    for i in range(1, len(path)):
        p1 = path[i-1].pose.position
        p2 = path[i].pose.position
        dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        length += dist

    # Calculate smoothness (based on angle changes)
    smoothness = 0
    turning_points = 0
    for i in range(1, len(path)-1):
        p0 = path[i-1].pose.position
        p1 = path[i].pose.position
        p2 = path[i+1].pose.position

        # Calculate vectors
        v1_x, v1_y = p1.x - p0.x, p1.y - p0.y
        v2_x, v2_y = p2.x - p1.x, p2.y - p1.y

        # Calculate angle between vectors
        dot_product = v1_x * v2_x + v1_y * v2_y
        v1_mag = math.sqrt(v1_x**2 + v1_y**2)
        v2_mag = math.sqrt(v2_x**2 + v2_y**2)

        if v1_mag > 0 and v2_mag > 0:
            cos_angle = dot_product / (v1_mag * v2_mag)
            angle = math.acos(max(-1, min(1, cos_angle)))  # Clamp to [-1, 1]

            if angle > 0.1:  # Consider significant turns
                turning_points += 1
                smoothness += angle

    # Calculate average curvature
    curvature = smoothness / max(1, turning_points) if turning_points > 0 else 0

    return {
        'length': length,
        'smoothness': smoothness,
        'curvature': curvature,
        'turning_points': turning_points,
        'average_segment_length': length / max(1, len(path) - 1)
    }


def assess_path_quality(path: List[PoseStamped], max_length=None, max_turns=None):
    """Assess path quality against criteria"""
    metrics = calculate_path_metrics(path)

    quality_score = 1.0

    # Length penalty
    if max_length and metrics['length'] > max_length:
        quality_score *= 0.5  # Reduce quality if path is too long

    # Turn penalty
    if max_turns and metrics['turning_points'] > max_turns:
        quality_score *= 0.7  # Reduce quality if too many turns

    # Smoothness bonus
    if metrics['curvature'] < 0.5:  # Relatively straight path
        quality_score *= 1.2  # Bonus for smooth paths

    return {
        'metrics': metrics,
        'quality_score': min(quality_score, 1.0),  # Cap at 1.0
        'is_acceptable': quality_score > 0.5
    }
```

## Integration with Navigation System

### 1. Planner Integration Configuration

```yaml
# Complete navigation configuration with custom planners
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    navigate_through_poses: false
    navigate_to_pose: true
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
```

## Troubleshooting Common Issues

### 1. Path Planning Failures

#### Issue: Planner fails to find path
**Symptoms**: Navigation fails with "No valid path found"
**Solutions**:
1. Increase planner tolerance:
   ```yaml
   tolerance: 1.0  # Increase from 0.5
   ```
2. Allow planning through unknown space:
   ```yaml
   allow_unknown: true
   ```
3. Check costmap configuration for proper obstacle inflation

#### Issue: Path is not optimal
**Symptoms**: Robot takes unnecessarily long routes
**Solutions**:
1. Use A* instead of Dijkstra:
   ```yaml
   use_astar: true
   ```
2. Adjust heuristic function
3. Increase planning frequency

#### Issue: High CPU usage during planning
**Symptoms**: System becomes unresponsive during path planning
**Solutions**:
1. Reduce map resolution for planning
2. Lower planning frequency
3. Use simpler planning algorithms for real-time applications

### 2. Performance Monitoring

```bash
# Monitor path planning performance
ros2 action list | grep path
ros2 action info /compute_path_to_pose

# Check planning execution time
ros2 topic echo /planning_time

# Monitor path quality
ros2 run nav2_util path_analyzer
```

## Best Practices

### 1. Algorithm Selection Guidelines
- **A***: Best for optimal paths with good heuristics
- **Dijkstra**: Good for guaranteed optimal paths without heuristics
- **NavFn**: Fast for large maps with approximate solutions
- **Theta***: Best for any-angle paths with line-of-sight optimization

### 2. Configuration Best Practices
- Start with default Navigation2 configurations
- Gradually tune parameters based on performance
- Test with various map complexities
- Monitor computational requirements
- Validate path safety and optimality

### 3. Testing Strategies
- Test with different map sizes and complexities
- Validate behavior with dynamic obstacles
- Check path quality metrics
- Monitor computational performance
- Test edge cases (narrow passages, dead ends)

## Resources

- [Navigation2 Path Planning Documentation](https://navigation.ros.org/configuration/packages/configuring-planners.html)
- [Nav2 Planners Source Code](https://github.com/ros-planning/navigation2/tree/main/nav2_navfn)
- [Path Planning Algorithms Comparison](https://navigation.ros.org/tutorials/docs/path_planning_comparison.html)

## Conclusion

This guide provides comprehensive coverage of path planning algorithms for ROS 2 Navigation2. The choice of algorithm depends on your specific requirements for optimality, computation time, and path quality. Built-in planners like NavFn provide good performance for most applications, while custom implementations allow for specialized requirements. Proper configuration and tuning are essential for achieving optimal navigation performance in your specific environment and use case.