"""
ros2 launch botdylan nopython.launch.py

This should start
  1) RVIZ, ready to view the robot
  2) The robot_state_publisher to broadcast the robot model
  3) The code to move the joints
  4) Joint sliders

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('botdylan'), 'rviz/urdf.rviz')

    # Locate the URDF file.
    urdf = os.path.join(pkgdir('sr_description'), 'robots/sr_hand_bimanual.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher = Node(
        name='robot_state_publisher', 
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Configure a node for RVIZ
    node_rviz = Node(
        name='rviz', 
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rvizcfg],
        on_exit=Shutdown()
    )
    
    # Configure a node for the joint trajectory
    node_trajectory = Node(
        name='kintest', 
        package='botdylan',
        executable='trajectory',
        output='screen'
    )

    # Configure the joint_state_publisher_gui node
    node_joint_state_publisher_gui = Node(
        name='joint_state_publisher_gui', 
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST
    return LaunchDescription([
        # Start the robot_state_publisher, RVIZ, the trajectory, and joint_state_publisher_gui
        node_robot_state_publisher,
        node_rviz,
        # node_trajectory,
        node_joint_state_publisher_gui,  
    ])
