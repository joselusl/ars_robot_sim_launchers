from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    # Define the nodes
    ars_sim_collision_detection_node=Node(
      package='ars_sim_collision_detection',
      executable='ars_sim_collision_detection_ros_node',
      name='ars_sim_collision_detection_node',
      output=LaunchConfiguration('screen'),
      parameters=[],
      remappings=[
        ('robot_pose', '/simulator/sim_robot/robot_pose'),
        ('obstacles_static', '/simulator/sim_environment/obstacles_static'),
        ('obstacles_dynamic', '/simulator/sim_environment/obstacles_dynamic'),
        ('robot_collision', '/simulator/sim_robot/robot_collision'),
      ]
    )

    #
    return LaunchDescription([
      screen_arg,
      GroupAction([
        PushRosNamespace('simulator'),
        GroupAction([
          PushRosNamespace('sim_environment'),
          ars_sim_collision_detection_node,
        ]),
      ])
    ])
