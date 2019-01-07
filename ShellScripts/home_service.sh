#!/bin/sh

export TURTLEBOT_3D_SENSOR=kinect
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/meow/udacity/robotics/term2/HomeService/catkin_ws/src/World/myworld.world" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/meow/udacity/robotics/term2/HomeService/catkin_ws/src/World/map.yaml 3d_sensor:=kinect" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node" &

