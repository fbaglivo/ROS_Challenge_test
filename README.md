# ROS_Challenge_test

This repository was created for a ROS demo. The aim of it is to record a turtlebot teleoperated path and then reproduce it in autonomous way. 

## Tooks

- Ubuntu 16.04 LTS OS
- Kinetic ROS version with Kobuki model, Gazebo and RVIZ
- Catkin build tools http://wiki.ros.org/catkin
- Packages: gmapping, map_server, teleop, bag_recorder (provided in this repo), AMCL


## Map creation

A map creation is an important issue in this challenge. To navigate a map robot first need to "know" the map, so first robot should navigate through the entire map and record obstacle coordinates for example.

To make a map these steps should be followed:

	1. Launch gazebo session

		roslaunch turtlebot_gazebo turtlebot_world.launch

	2. Launch gmapping package

		roslaunch turtlebot_gazebo gmapping_demo.launch

	3. Launch RVIZ

		roslaunch turtlebot_rviz_launchers view_navigation.launch 

	4. Launch Teleoperation package

		roslaunch turtlebot_teleop keyboard_teleop.launch

	5. After a full map navigation was done, run map saver

		rosrun map_server map_saver -f <file name>


## Map testing


To navigate a map these steps should be followed:

	1. Launch gazebo session

		roslaunch turtlebot_gazebo turtlebot_world.launch

	2. Launch AMCL package with a map

		roslaunch turtlebot_gazebo amcl_demo.launch map_file:=<full path.yaml> 

	3. Launch RVIZ

		roslaunch turtlebot_rviz_launchers view_navigation.launch 

	4. Launch Teleoperation package

		roslaunch turtlebot_teleop keyboard_teleop.launch


## Record path service

With the porpose of record a path a recording service, that can be found in bag_recorder folder located inside this repository, is used. This service create a bag file and record all topic specified when rosservice is called. Gazebo and teleoperation package should be running before the service is called.
 
     1. bag_recorder folder should be located into catkin_ws to build the package:
        
           catkin_make -DCATKIN_WHITELIST_PACKAGES="bag_recorder"

     1.1 These package are requiered to do the build: geometry_msgs, rosbag, roscpp, rospy, rostopic, sensor_msgs, std_msgs and message_generation

     2. Start Gazebo, AMCL,Rviz and teleop as been done previously

	3. Start the service:

		rosrun bag_recorder recorder.py

	4. To begin a recording session:

		rosservice call start_record '{bag_path:<full path>, topic_list:['/tf','/scan'],make_path:TRUE}'

	5. To stop a recording session:

		rosserviec call stop_record '{bag_id:"bag_id"}'

Usefull commands: rosservice list  :  to list all started services
                  rosservice info srv_name : to know about service call and arguments 

## Replay navigation

For autonomus navigation AMCL package is used. This packages subscribe to /scan, /tf, /initialpose and /map topics to give a probabilistic location of the robot.

	1. Launch Gazebo

		roslaunch turtlebot_gazebo turtlebot_world.launch  #roslaunch turtlebot_gazebo gmapping_demo.launch

	2. Launch AMCL

		roslaunch turtlebot_gazebo amcl_demo.launch mapfile:=<full path . yaml>

	3. Launch RVIZ

		roslaunch turtlebot_rviz_launchers view_navigation.launch 

    	4. Run rosbag_play.py script to navigate  (IN PROGRESS)

		python rosbag_play.py <full path/bagfile.bag>


