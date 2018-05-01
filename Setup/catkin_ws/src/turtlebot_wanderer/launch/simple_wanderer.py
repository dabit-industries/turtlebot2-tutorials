<launch>
  <!--include file="$(find turtlebot_bringup)/launch/minimal.launch"></include-->
  <!--include file="$(find turtlebot_bringup)/launch/3dsensor.launch"></include-->
  <node name ="turtlebot_wanderer_simple" pkg="turtlebot_wanderer" type="simple_wanderer.py" cwd="node"/>
</launch>
