roslaunch multi_slam multi_turtlebot3.launch & \
roslaunch multi_slam multi_turtlebot3_slam_decentralised.launch & \
#python ./catkin_ws/src/multi_slam/python/decentraliser.py & \
#roslaunch turtlebot3_gazebo multi_map_merge_decentralised.launch & \
#rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam.rviz