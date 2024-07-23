<!-- -->
<launch>

  <arg name="sim_obstacles_detector_params_yaml_file" default="$(find ars_config)/config/sim_obstacles_detector/config_sim_obstacles_detector_long_range.yaml"/>

  <group ns="simulator">

    <group ns="sim_obstacles_detector">

      <node name="ars_sim_obstacles_detector_node" pkg="ars_sim_obstacles_detector" type="ars_sim_obstacles_detector_ros_node.py" output="screen" >
   
        <remap from="robot_pose" to="/simulator/sim_robot/robot_pose"/> 

        <remap from="obstacles_static" to="/simulator/sim_environment/obstacles_static"/>
        <remap from="obstacles_dynamic" to="/simulator/sim_environment/obstacles_dynamic"/>

        <remap from="obstacles_detected_world" to="/obstacles_detected_world"/>
        <remap from="obstacles_detected_robot" to="/obstacles_detected_robot"/>
    
        <param name="sim_obstacles_detector_params_yaml_file" value="$(arg sim_obstacles_detector_params_yaml_file)" />
    
      </node>
     
    </group>

  </group>

</launch>
