---
sidebar_position: 2
---

# ROS 2 Services and Actions - Advanced Communication

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the differences between topics, services, and actions in ROS 2
- Create and use ROS 2 services for request-response communication
- Implement ROS 2 actions for goal-oriented, long-running tasks
- Design appropriate communication patterns for different use cases
- Test and debug service and action implementations

## Prerequisites

Before starting this chapter, you should:
- Understand ROS 2 nodes and topics
- Have ROS 2 Humble Hawksbill installed and configured
- Be familiar with creating ROS 2 packages
- Completed the ROS 2 nodes and topics chapter

## Conceptual Overview

In the previous chapter, you learned about topics which use a publish-subscribe communication pattern. While topics are excellent for continuous data streams, many robotics applications require request-response interactions or long-running goal-oriented tasks. This is where services and actions come in.

### Services

**Services** provide a request-response communication pattern where:
- A client sends a request to a service
- The service processes the request and sends back a response
- Communication is synchronous from the client's perspective
- Services are ideal for operations that have a clear start and end

### Actions

**Actions** are designed for long-running tasks that may:
- Take a significant amount of time to complete
- Provide feedback during execution
- Be cancelable or preemptable
- Report progress and status during execution

### When to Use Each Pattern

- **Topics**: Continuous data streams, sensor data, robot state
- **Services**: Simple request-response tasks, configuration changes, data retrieval
- **Actions**: Complex tasks with progress, robot navigation, manipulation sequences

## Hands-On Implementation

### Services

#### Service Definition

First, let's define a simple service. Create a file called `AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

This service takes two integers as input and returns their sum.

#### Service Server (Python)

**Service Server (service_member_function.py):**

```python
#!/usr/bin/env python3
# service_member_function.py

"""
This example demonstrates how to create a simple service server using a class member function to handle
incoming requests.
"""

import sys
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # Create a service that will use the callback function to handle requests
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Service Client (Python)

**Service Client (client_member_function.py):**

```python
#!/usr/bin/env python3
# client_member_function.py

"""
This example demonstrates how to create a simple service client using a class member function to send
requests and handle responses.
"""

import sys
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Actions

#### Action Definition

Create an action definition file called `Fibonacci.action`:

```
#goal definition
int32 order

#result definition
int32[] sequence

#feedback
int32[] sequence
```

This action generates a Fibonacci sequence of a given order.

#### Action Server (Python)

**Action Server (action_server.py):**

```python
#!/usr/bin/env python3
# action_server.py

"""
This example demonstrates how to create a simple action server.
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Feedback and result
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        result = Fibonacci.Result()

        for i in range(1, goal_handle.request.order):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.sequence = feedback_msg.sequence
                return result

            # Update the sequence
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate work
            time.sleep(1)

        # Check if the goal was canceled before completing
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            result.sequence = feedback_msg.sequence
            return result

        # Complete the goal
        goal_handle.succeed()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(fibonacci_action_server, executor=executor)

    fibonacci_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Action Client (Python)

**Action Client (action_client.py):**

```python
#!/usr/bin/env python3
# action_client.py

"""
This example demonstrates how to create a simple action client.
"""

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup())

    def send_goal(self, order):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal
        self.get_logger().info(f'Sending goal: {order}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Wait for the result
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        self.get_logger().info(f'Result: {result.sequence}')
        return result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    # Send a goal
    action_client.send_goal(10)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating the Package

To create a ROS 2 package for these examples:

1. **Create the package:**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_srvcli
```

2. **Create the service definition directory:**
```bash
mkdir -p py_srvcli/py_srvcli
```

3. **Add the Python files to the package:**
```bash
# Copy service server to py_srvcli/py_srvcli/service_member_function.py
# Copy service client to py_srvcli/py_srvcli/client_member_function.py
# Copy action server to py_srvcli/py_srvcli/action_server.py
# Copy action client to py_srvcli/py_srvcli/action_client.py
```

4. **Make the Python files executable:**
```bash
chmod +x py_srvcli/py_srvcli/service_member_function.py
chmod +x py_srvcli/py_srvcli/client_member_function.py
chmod +x py_srvcli/py_srvcli/action_server.py
chmod +x py_srvcli/py_srvcli/action_client.py
```

5. **Update setup.py to include entry points:**
```python
entry_points={
    'console_scripts': [
        'add_two_ints_server = py_srvcli.service_member_function:main',
        'add_two_ints_client = py_srvcli.client_member_function:main',
        'fibonacci_server = py_srvcli.action_server:main',
        'fibonacci_client = py_srvcli.action_client:main',
    ],
},
```

6. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select py_srvcli
```

7. **Source the workspace:**
```bash
source install/setup.bash
```

## Testing & Verification

### Testing Services

1. **Open two terminal windows**

2. **In the first terminal, source your workspace and run the service server:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli add_two_ints_server
```

3. **In the second terminal, source your workspace and run the service client:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli add_two_ints_client 2 3
```

4. **You should see the server receive the request and the client get the response:**
```
[INFO] [1601234567.123456789] [minimal_service]: Incoming request
a: 2 b: 3
[INFO] [1601234567.123456789] [minimal_client]: Result of add_two_ints: for 2 + 3 = 5
```

### Testing Actions

1. **Open two terminal windows**

2. **In the first terminal, source your workspace and run the action server:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli fibonacci_server
```

3. **In the second terminal, source your workspace and run the action client:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_srvcli fibonacci_client
```

4. **You should see feedback during execution and the final result:**
```
[INFO] [1601234567.123456789] [fibonacci_action_client]: Waiting for action server...
[INFO] [1601234567.123456789] [fibonacci_action_client]: Sending goal: 10
[INFO] [1601234567.123456789] [fibonacci_action_client]: Received feedback: [0, 1, 1]
[INFO] [1601234567.123456789] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2]
...
[INFO] [1601234567.123456789] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

### Useful Commands for Service and Action Inspection

- **List all services:**
```bash
ros2 service list
```

- **Get info about a specific service:**
```bash
ros2 service info /add_two_ints
```

- **Call a service directly:**
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 1, b: 2}'
```

- **List all actions:**
```bash
ros2 action list
```

- **Get info about a specific action:**
```bash
ros2 action info /fibonacci
```

## Common Issues

### Issue: Service client cannot connect to server
**Solution**:
- Verify the service server is running
- Check that service names match exactly
- Ensure both nodes are in the same ROS domain
- Check network connectivity if nodes are on different machines

### Issue: Action client does not receive feedback
**Solution**:
- Verify the action server is publishing feedback
- Check that the action definition matches between client and server
- Ensure the action server is not completing too quickly

### Issue: Package build fails with service/action definitions
**Solution**:
- Make sure to add the correct dependencies in package.xml
- For services: add `<depend>example_interfaces</depend>`
- For actions: add `<depend>example_interfaces</depend>`
- Ensure the correct message types are imported

## Key Takeaways

- Services provide request-response communication, ideal for quick operations
- Actions are designed for long-running tasks with feedback and cancellation
- Services are synchronous from the client perspective
- Actions provide asynchronous communication with progress tracking
- Use topics for continuous data streams, services for simple requests, and actions for complex tasks
- Both services and actions have built-in tools for inspection and debugging

## Next Steps

In the next chapter, you'll learn about ROS 2 packages and how to organize your code effectively in the ROS 2 ecosystem.