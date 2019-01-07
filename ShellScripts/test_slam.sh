#!/bin/sh

export TURTLEBOT_3D_SENSOR=kinect
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/meow/udacity/robotics/term2/HomeService/catkin_ws/src/World/myworld.world" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_navigation gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"

