<!-- -->
<launch>


  <arg name="robot_description_path"       default="$(find ars_robot_models)/urdf/dji_m100.urdf" />



  <param name="robot_description" textfile="$(arg robot_description_path)" />


  <group ns="simulator">

    <group ns="sim_robot">

    <!-- Aerial robot -->

      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">

        <param name="~robot_description" textfile="$(arg robot_description_path)" />

      </node>

      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">

        <param name="~robot_description" textfile="$(arg robot_description_path)" />

      </node>

    </group>

  </group>

</launch>
