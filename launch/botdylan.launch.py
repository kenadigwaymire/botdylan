import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import OpaqueFunction
from launch.actions                    import Shutdown
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node


def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file
    rvizcfg = os.path.join(pkgdir('botdylan'), 'rviz/config.rviz')
    #assert os.path.exists(rvizcfg), f"RVIZ config file not found: {rvizcfg}"

    # Locate the URDF file
    urdf = os.path.join(pkgdir('sr_description'), 'robots/sr_hand_bimanual.urdf')
    #assert os.path.exists(urdf), f"URDF file not found: {urdf}"

    # Load the robot's URDF file (XML)
    with open(urdf, 'r') as file:
        robot_description = file.read()

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Robot state publisher node
    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # RVIZ node
    node_rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rvizcfg],
        on_exit=Shutdown()  # Shut down all nodes when RVIZ exits
    )

    # Joint trajectory node
    node_trajectory = Node(
        name='trajectory_control',
        package='botdylan',
        executable='trajectory',
        output='screen'
    )

    # Joint state publisher GUI node
    node_joint_state_publisher_gui = Node(
        name='joint_state_publisher_gui',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        on_exit=Shutdown()
    )

    ######################################################################
    # RETURN THE LAUNCH DESCRIPTION
    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_trajectory,
        node_joint_state_publisher_gui
    ])
