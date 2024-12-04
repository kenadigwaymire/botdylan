import os
import xacro
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ######################################################################
    # LOCATE FILES

    # Locate the folder containing RVIZ configuration files
    rviz_folder = os.path.join(pkgdir('botdylan'), 'rviz')

    # Default RVIZ file name
    default_rviz = 'viewurdfplus.rviz'

    # Locate the hands URDF file relative to its package
    primary_urdf = os.path.join(pkgdir('sr_description'), 'robots/sr_hand_bimanual.urdf')

    # Locate the guitar URDF file relative to its package
    second_urdf = os.path.join(pkgdir('botdylan'), 'urdf/guitar.urdf')

    # Checking if xacro or normal to get robot_description
    if primary_urdf.endswith('.xacro'):
        primary_robot_description = xacro.process_file(primary_urdf).toxml()
    else:
        with open(primary_urdf, 'r') as file:
            primary_robot_description = file.read()

    if second_urdf.endswith('.xacro'):
        second_robot_description = xacro.process_file(second_urdf).toxml()
    else:
        with open(second_urdf, 'r') as file:
            second_robot_description = file.read()

    ######################################################################
    # DECLARE LAUNCH ARGUMENTS

    # Declare an argument to specify the RVIZ configuration file
    # Added because robot description topic refused to work. This way we 
    # can setup separate rviz files with different local file paths.
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

    # Configure the joint_state_publisher node
    node_joint_state_publisher = Node(
        name='joint_state_publisher',
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Add a static transform to position the guitar relative to the world frame or the robot
    # Decided to have a world node in both urdfs and make joints relative to this
    # node_static_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['1', '0', '0', '0', '0', '0', 'world', 'guitar_base_link']
    # )

    # Configure the RVIZ node to use the selected configuration
    node_rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_file')],
        on_exit=Shutdown()
    )

    # Configure the trajectory node
    node_trajectory = Node(
        name='kintest',
        package='botdylan',
        executable='trajectory',
        output='screen'
    )

    ######################################################################
    # SEQUENCE NODES

    # # RVIZ loads after the primary robot_state_publisher
    # rviz_after_primary = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=node_primary_robot_state_publisher,
    #         on_start=[node_rviz]
    #     )
    # )

    # Trajectory loads after RVIZ
    trajectory_after_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_rviz,
            on_start=[node_trajectory]
        )
    )

    # Second URDF loads after a timer
    second_urdf_timer = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[node_second_robot_state_publisher]
    )

    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        rviz_arg,
        node_primary_robot_state_publisher,
        node_rviz,
        node_joint_state_publisher
        #trajectory_after_rviz,
        #second_urdf_timer
    ])
