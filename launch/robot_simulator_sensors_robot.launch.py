from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    # Define the nodes
    ars_sim_sensor_posi_robot_node=Node(
      package='ars_sim_sensors_robot',
      executable='ars_sim_sensor_posi_robot_ros_node',
      name='ars_sim_sensor_posi_robot_node',
      output=LaunchConfiguration('screen'),
      remappings=[
          ('robot_pose', '/simulator/sim_robot/robot_pose'),
          ('meas_robot_position', '/meas_robot_position'),
      ]
    )

    ars_sim_sensor_atti_robot_ros_node=Node(
      package='ars_sim_sensors_robot',
      executable='ars_sim_sensor_atti_robot_ros_node',
      name='ars_sim_sensor_atti_robot_node',
      output=LaunchConfiguration('screen'),
      remappings=[
          ('robot_pose', '/simulator/sim_robot/robot_pose'),
          ('meas_robot_attitude', '/meas_robot_attitude'),
      ]
    )
    
    ars_sim_sensor_vel_robot_ros_node=Node(
      package='ars_sim_sensors_robot',
      executable='ars_sim_sensor_vel_robot_ros_node',
      name='ars_sim_sensor_vel_robot_node',
      output=LaunchConfiguration('screen'),
      remappings=[
          ('robot_velocity', '/simulator/sim_robot/robot_velocity_robot'),
          ('meas_robot_velocity', '/meas_robot_velocity_robot'),
          ('meas_robot_velocity_cov', '/meas_robot_velocity_robot_cov'),
      ]
    )


    #
    return LaunchDescription([
        screen_arg,
        GroupAction([
          PushRosNamespace('simulator'),
          GroupAction([
            PushRosNamespace('sim_sensors_robot'),
            ars_sim_sensor_posi_robot_node,
            ars_sim_sensor_atti_robot_ros_node,
            ars_sim_sensor_vel_robot_ros_node,
          ])
        ])
    ])
