from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Define the arguments
    environment_description_yaml_file_arg=DeclareLaunchArgument(
      'environment_description_yaml_file',
      default_value=PathJoinSubstitution([FindPackageShare('ars_config'), 'config', 'sim_environment', 'obstacles_env_00.yaml']), 
      description='Path to the environment description yaml file'
    )

    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    # Get the launch configuration for parameters
    environment_description_yaml_file = LaunchConfiguration('environment_description_yaml_file')

    # Define the node
    ars_sim_environment_node = Node(
        package='ars_sim_environment',
        executable='ars_sim_environment_ros_node',
        name='ars_sim_environment_node',
        output=LaunchConfiguration('screen'),
        parameters=[{'environment_description_yaml_file': environment_description_yaml_file}],
        remappings=[
            ('obstacles_static', '/simulator/sim_environment/obstacles_static'),
            ('obstacles_dynamic', '/simulator/sim_environment/obstacles_dynamic'),
        ]
    )

    # Define the launch description
    return LaunchDescription([
        environment_description_yaml_file_arg,
        screen_arg,
        GroupAction([
            PushRosNamespace('simulator'),
            GroupAction([
                PushRosNamespace('sim_environment'),
                ars_sim_environment_node
            ])
        ])
    ])
