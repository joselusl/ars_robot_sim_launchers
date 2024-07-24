from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
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

    
    # Define the nodes
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
    

    #
    return LaunchDescription([
      screen_arg,
      robot_description_path_arg,
      read_file_action,
      GroupAction([
        PushRosNamespace('simulator'),
        GroupAction([
          PushRosNamespace('sim_robot'),
          joint_state_publisher,
          robot_state_publisher,
        ]),
      ]),
    ])
