<!-- -->
<launch>

  <group ns="simulator">

    <group ns="sim_mapper">

      <node name="ars_sim_mapper_node" pkg="ars_sim_mapper" type="ars_sim_mapper_ros_node.py" output="screen" >

        <remap from="obstacles_static" to="/simulator/sim_environment/obstacles_static"/>
        <remap from="obstacles_dynamic" to="/simulator/sim_environment/obstacles_dynamic"/>

        <remap from="estim_map_world" to="/estim_map_world_sim"/>
    
      </node>
     
    </group>

  </group>

</launch>
