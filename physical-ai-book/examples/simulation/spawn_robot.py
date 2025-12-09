#!/usr/bin/env python3
"""
Robot Spawning Script for Gazebo Simulation

This script demonstrates how to spawn robots in Gazebo with proper physics parameters.
It uses the gazebo_ros spawn_entity service to place robots in the simulation environment.
"""

import rospy
import sys
import os
from gazebo_msgs.srv import SpawnEntity, SpawnEntityRequest
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import xml.etree.ElementTree as ET


def spawn_robot(robot_name, robot_urdf_path, pose, reference_frame="world"):
    """
    Spawn a robot in Gazebo with specified parameters

    Args:
        robot_name (str): Name of the robot to spawn
        robot_urdf_path (str): Path to the robot URDF file
        pose (geometry_msgs.msg.Pose): Position and orientation of the robot
        reference_frame (str): Reference frame for the robot position
    """

    # Wait for the spawn service to be available
    rospy.wait_for_service('/spawn_entity')

    try:
        # Create the spawn service client
        spawn_client = rospy.ServiceProxy('/spawn_entity', SpawnEntity)

        # Read the URDF file
        with open(robot_urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()

        # Create the spawn request
        spawn_request = SpawnEntityRequest()
        spawn_request.name = robot_name
        spawn_request.xml = robot_description
        spawn_request.initial_pose = pose
        spawn_request.reference_frame = reference_frame

        # Call the spawn service
        response = spawn_client(spawn_request)

        if response.success:
            rospy.loginfo(f"Successfully spawned robot '{robot_name}' at pose: {pose}")
        else:
            rospy.logerr(f"Failed to spawn robot '{robot_name}': {response.status_message}")

        return response.success

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def reset_simulation():
    """
    Reset the Gazebo simulation
    """
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_client = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_client()
        rospy.loginfo("Simulation reset successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset simulation: {e}")


def pause_simulation():
    """
    Pause the Gazebo simulation
    """
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_client()
        rospy.loginfo("Simulation paused")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to pause simulation: {e}")


def unpause_simulation():
    """
    Unpause the Gazebo simulation
    """
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_client()
        rospy.loginfo("Simulation unpaused")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to unpause simulation: {e}")


def main():
    """
    Main function to demonstrate robot spawning
    """
    rospy.init_node('robot_spawner', anonymous=True)

    # Define robot parameters
    robot_name = "my_robot"
    robot_urdf_path = os.path.expanduser("~/ros2_ws/src/my_robot_description/urdf/robot.urdf")

    # Check if URDF file exists, if not, try to use one of our example URDFs
    if not os.path.exists(robot_urdf_path):
        # Try the basic robot URDF from our examples
        robot_urdf_path = os.path.join(os.path.dirname(__file__), "basic_robot.urdf")
        if not os.path.exists(robot_urdf_path):
            rospy.logerr(f"URDF file not found: {robot_urdf_path}")
            sys.exit(1)

    # Define initial pose for the robot
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.5  # Start slightly above ground to avoid collision
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0

    rospy.loginfo("Waiting for Gazebo services...")

    # Example: Spawn a basic wheeled robot
    success = spawn_robot(robot_name, robot_urdf_path, initial_pose)

    if success:
        rospy.loginfo("Robot spawned successfully!")

        # Wait a bit before spawning another robot
        rospy.sleep(2.0)

        # Example: Spawn a second robot at a different location
        robot2_pose = Pose()
        robot2_pose.position.x = 2.0
        robot2_pose.position.y = 2.0
        robot2_pose.position.z = 0.5
        robot2_pose.orientation.w = 1.0

        spawn_robot("robot2", robot_urdf_path, robot2_pose)

    else:
        rospy.logerr("Failed to spawn robot")

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Robot spawner interrupted by user")