from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    sim_obstacles_detector_params_yaml_file=DeclareLaunchArgument(
      'sim_obstacles_detector_params_yaml_file',
      default_value=PathJoinSubstitution([
          FindPackageShare('ars_config'),
          'config',
          'sim_obstacles_detector',
          'config_sim_obstacles_detector_long_range.yaml'
      ]),
      description='Path to the config file for the simulator obstacles detector'
    )

    # Define the nodes
    ars_sim_obstacles_detector_node=Node(
      package='ars_sim_obstacles_detector',
      executable='ars_sim_obstacles_detector_ros_node',
      name='ars_sim_obstacles_detector_node',
      output=LaunchConfiguration('screen'),
      parameters=[{'sim_obstacles_detector_params_yaml_file': LaunchConfiguration('sim_obstacles_detector_params_yaml_file')}],
      remappings=[
        ('robot_pose', '/simulator/sim_robot/robot_pose'),
        ('obstacles_static', '/simulator/sim_environment/obstacles_static'),
        ('obstacles_dynamic', '/simulator/sim_environment/obstacles_dynamic'),
        ('obstacles_detected_world', '/obstacles_detected_world'),
        ('obstacles_detected_robot', '/obstacles_detected_robot')
      ]
    )

    #
    return LaunchDescription([
      screen_arg,
      sim_obstacles_detector_params_yaml_file,
      GroupAction([
        PushRosNamespace('simulator'),
        GroupAction([
          PushRosNamespace('sim_obstacles_detector'),
          ars_sim_obstacles_detector_node,
        ]),
      ])
    ])
