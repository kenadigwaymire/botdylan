import os
import xacro
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ######################################################################
    # LOCATE FILES

    # Locate the folder containing RVIZ configuration files
    rviz_folder = os.path.join(pkgdir('botdylan'), 'rviz')
    
    # Default RVIZ file name
    default_rviz = 'viewurdfplus.rviz'

    # Locate the primary URDF file relative to its package
    primary_urdf = os.path.join(pkgdir('sr_description'), 'robots/sr_hand_bimanual.urdf')

    # Locate the second URDF file relative to its package
    second_urdf = os.path.join(pkgdir('botdylan'), 'urdf/guitar.urdf')

    # Preprocess the primary URDF file
    if primary_urdf.endswith('.xacro'):
        primary_robot_description = xacro.process_file(primary_urdf).toxml()
    else:
        with open(primary_urdf, 'r') as file:
            primary_robot_description = file.read()

    # Preprocess the second URDF file
    if second_urdf.endswith('.xacro'):
        second_robot_description = xacro.process_file(second_urdf).toxml()
    else:
        with open(second_urdf, 'r') as file:
            second_robot_description = file.read()

    ######################################################################
    # DECLARE LAUNCH ARGUMENTS

    # Declare an argument to specify the RVIZ configuration file
    rviz_arg = DeclareLaunchArgument(
        'rviz_file',
        default_value=os.path.join(rviz_folder, default_rviz),
        description='Path to the RVIZ configuration file'
    )

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the primary robot_state_publisher node
    node_primary_robot_state_publisher = Node(
        name='robot_state_publisher_primary',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': primary_robot_description}]
    )

    # Configure the second robot_state_publisher node
    node_second_robot_state_publisher = Node(
        name='robot_state_publisher_second',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': second_robot_description}]
    )

    # Add a static transform to position the guitar relative to the world frame or the robot
    node_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['1', '0', '0', '0', '0', '0', 'world', 'guitar_base_link']
    )

    # Configure the RVIZ node to use the selected configuration
    node_rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_file')],
        on_exit=Shutdown()
    )

    # Configure the joint trajectory node
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
        parameters=[{'robot_description': primary_robot_description}]
    )

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        rviz_arg,
        node_primary_robot_state_publisher,
        node_second_robot_state_publisher,
        #node_static_transform,
        node_rviz,
        node_trajectory
        #node_joint_state_publisher_gui,
    ])