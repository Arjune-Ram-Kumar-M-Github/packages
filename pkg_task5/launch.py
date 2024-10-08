'''
task5_solution.launch

Task 5 Launch File 
::

    <launch>

        <!-- Spawn Task-2 Models in Gazebo -->
        <include file="$(find pkg_vb_sim)/launch/task5_simulation.launch" />


        <include file="$(find pkg_vb_sim)/launch/two_ur5_move_group.launch" />

        <!-- Parameter files -->
        <rosparam file ="$(find pkg_task5)/config/packages.yaml"/>
        <rosparam file ="$(find pkg_ros_iot_bridge)/config/config_pyiot.yaml"/>



        <!-- Node to connect ROS & IOT(MQTT) -->
        <node pkg="pkg_ros_iot_bridge" type="node_action_server_ros_iot_bridge.py" name="node_action_server_ros_iot_bridge" output="screen"/>

        <node pkg="pkg_task5" type="node_iot_ros_bridge_action_client.py" name="node_iot_ros_bridge_action_client" output="screen"/>

        <!-- Node to decode colors from image of Shelf  -->
        <node name= "node_camera" pkg= "pkg_task5" type="node_camera.py" output="screen"/>

        <!-- Node to run ur5_1 to pick & place box in Conveyor-->
        <node name= "ur5_1" pkg= "pkg_task5" type="node_ur5_1.py" output="screen"/>

        <!-- Node to run ur5_2 to pick & place box in bin-->
        <node name= "ur5_2" pkg= "pkg_task5" type="node_ur5_2.py" output="screen"/>



    </launch>

'''