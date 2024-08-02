from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Define the arguments
    robot_sim_description_yaml_file_arg=DeclareLaunchArgument(
      'robot_sim_description_yaml_file',
      default_value=PathJoinSubstitution(['sim_robot', 'config_sim_robot_dji_m100.yaml']), 
      description='Path to the robot sim description yaml file'
    )

    robot_init_status_flying_arg = DeclareLaunchArgument(
        'robot_init_status_flying',
        default_value='False'
    )

    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )
    

    # Get the launch configuration for parameters
    robot_sim_description_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_sim_config'), 'config', LaunchConfiguration('robot_sim_description_yaml_file')])
    robot_init_status_flying = LaunchConfiguration('robot_init_status_flying')

    # Define the nodes
    ars_sim_robot_dynamics_node = Node(
        package='ars_sim_robot',
        executable='ars_sim_robot_dynamics_ros_node',
        name='ars_sim_robot_dynamics_node',
        output=LaunchConfiguration('screen'),
        parameters=[{'robot_sim_description_yaml_file': robot_sim_description_yaml_file}],
        remappings=[
            ('robot_cmd', '/robot_cmd_unstamped'),
            ('robot_cmd_stamped', '/robot_cmd_stamped'),
            ('robot_pose', 'robot_pose'),
            ('robot_velocity_world', 'robot_velocity_world'),
            ('robot_velocity_robot', 'robot_velocity_robot'),
            ('robot_acceleration_world', 'robot_acceleration_world'),
            ('robot_acceleration_robot', 'robot_acceleration_robot'),
            ('robot_cmd_control_enabled', 'robot_cmd_control_enabled'),
            ('robot_motion_enabled', 'robot_motion_enabled'),
        ]
    )
    
    ars_sim_robot_status_node = Node(
        package='ars_sim_robot',
        executable='ars_sim_robot_status_ros_node',
        name='ars_sim_robot_status_node',
        output=LaunchConfiguration('screen'),
        parameters=[{'robot_init_status_flying': robot_init_status_flying}],
        remappings=[
            ('takeoff', '/takeoff'),
            ('land', '/land'),
            ('robot_status', '/robot_status'),
            ('robot_cmd_control_enabled', 'robot_cmd_control_enabled'),
            ('robot_motion_enabled', 'robot_motion_enabled'),
        ]
    )

    # Define the launch description
    return LaunchDescription([
        robot_sim_description_yaml_file_arg,
        robot_init_status_flying_arg,
        screen_arg,
        GroupAction(
            actions=[
                PushRosNamespace('simulator'),
                GroupAction(
                    actions=[
                        PushRosNamespace('sim_robot'),
                        ars_sim_robot_dynamics_node,
                        ars_sim_robot_status_node,
                    ]
                )
            ]
        )
    ])
