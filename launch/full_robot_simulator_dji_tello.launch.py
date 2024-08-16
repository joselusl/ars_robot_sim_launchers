from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )


    robot_sim_description_yaml_file_arg=DeclareLaunchArgument(
      'robot_sim_description_yaml_file',
      default_value=PathJoinSubstitution(['sim_robot', 'config_sim_robot_dji_tello.yaml']), 
      description='Path to the robot sim description yaml file'
    )

    environment_description_yaml_file_arg=DeclareLaunchArgument(
      'environment_description_yaml_file',
      default_value=PathJoinSubstitution(['sim_environment', 'obstacles_env_00.yaml']), 
      description='Path to the environment description yaml file'
    )

    sim_obstacles_detector_params_yaml_file_arg=DeclareLaunchArgument(
      'sim_obstacles_detector_params_yaml_file',
      default_value=PathJoinSubstitution(['sim_obstacles_detector', 'config_sim_obstacles_detector_short_range.yaml']),
      description='Path to the config file for the simulator obstacles detector'
    )


    robot_description_path_arg=DeclareLaunchArgument(
      'robot_description_path',
      default_value=PathJoinSubstitution([
        FindPackageShare('ars_robot_models'),
        'urdf',
        'dji_tello.urdf'
      ]), 
      description='Path to the robot description file'
    )

    def read_file_content_and_set(context, *args, **kwargs):
      # Get the file path from the launch configuration
      file_path = LaunchConfiguration('robot_description_path').perform(context)
      
      # Read the file content
      try:
          with open(file_path, 'r') as file:
              content = file.read()
      except Exception as e:
          content = f"Error reading file {file_path}: {str(e)}"
      
      # Set the content as a launch configuration variable
      return [SetLaunchConfiguration('robot_description', content)]

    read_file_action = OpaqueFunction(function=read_file_content_and_set)


    robot_init_status_flying_arg = DeclareLaunchArgument(
        'robot_init_status_flying',
        default_value='False'
    )

    
    # Get the launch configuration for parameters
    robot_sim_description_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_sim_config'), 'config', LaunchConfiguration('robot_sim_description_yaml_file')])
    robot_init_status_flying = LaunchConfiguration('robot_init_status_flying')

    environment_description_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_sim_config'), 'config', LaunchConfiguration('environment_description_yaml_file')])

    sim_obstacles_detector_params_yaml_file = PathJoinSubstitution([FindPackageShare('ars_robot_sim_config'), 'config', LaunchConfiguration('sim_obstacles_detector_params_yaml_file')])



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
            ('robot_collision', '/simulator/sim_robot/robot_collision'),
        ]
    )

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

    ars_sim_obstacles_detector_node=Node(
      package='ars_sim_obstacles_detector',
      executable='ars_sim_obstacles_detector_ros_node',
      name='ars_sim_obstacles_detector_node',
      output=LaunchConfiguration('screen'),
      parameters=[{'sim_obstacles_detector_params_yaml_file': sim_obstacles_detector_params_yaml_file}],
      remappings=[
        ('robot_pose', '/simulator/sim_robot/robot_pose'),
        ('obstacles_static', '/simulator/sim_environment/obstacles_static'),
        ('obstacles_dynamic', '/simulator/sim_environment/obstacles_dynamic'),
        ('obstacles_detected_world', '/obstacles_detected_world'),
        ('obstacles_detected_robot', '/obstacles_detected_robot')
      ]
    )

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

    # Joint State Publisher Node
    joint_state_publisher=Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher',
      output=LaunchConfiguration('screen'),
      parameters=[]
    )
    
    # Robot State Publisher Node
    robot_state_publisher=Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output=LaunchConfiguration('screen'),
      parameters=[{'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)}]
    )


    # Define the launch description
    return LaunchDescription([
        screen_arg,
        robot_sim_description_yaml_file_arg,
        robot_init_status_flying_arg,
        environment_description_yaml_file_arg,
        sim_obstacles_detector_params_yaml_file_arg,
        robot_description_path_arg,
        read_file_action,
        GroupAction(
            actions=[
                PushRosNamespace('simulator'),
                GroupAction(
                    actions=[
                        PushRosNamespace('sim_robot'),
                        ars_sim_robot_dynamics_node,
                        ars_sim_robot_status_node,
                        ars_sim_collision_detection_node,
                        joint_state_publisher,
                        robot_state_publisher,
                    ]
                ),
                GroupAction([
                    PushRosNamespace('sim_environment'),
                    ars_sim_environment_node,
                ]),
                GroupAction([
                    PushRosNamespace('sim_obstacles_detector'),
                    ars_sim_obstacles_detector_node,
                ]),
                GroupAction([
                    PushRosNamespace('sim_sensors_robot'),
                    ars_sim_sensor_posi_robot_node,
                    ars_sim_sensor_atti_robot_ros_node,
                    ars_sim_sensor_vel_robot_ros_node,
                ]),
            ]
        )
    ])
