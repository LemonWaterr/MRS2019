roslaunch multi_slam multi_turtlebot3.launch & \
roslaunch multi_slam multi_turtlebot3_slam_decentralised.launch & \
python ./catkin_ws/src/multi_slam/python/decentraliser.py & \
roslaunch turtlebot3_gazebo multi_map_merge_decentralised.launch & \
rosrun rviz rviz -d `rospack find turtlebot3_gazebo`/rviz/multi_turtlebot3_slam_decentralised.rviz

"""
1. decentre on with init_poses worked, since it can always begin with local map as merged map. (local + empty merged + empty merged = local)
2. for without, this didn't work sice local + empty merged + empty merged = local for some robot / empty for others
3. so tried to fix it by merging local + local + (empty merged) on first encounter, didn't work. Maybe the package needs all three maps to merge
4. tried feeding empty map for the robots that are not involved in the pairwise encounter, didn't work. Looks like there's no guarantee that three(or n) maps
arrive just at the right time for the merger (publishers and threaded thus asynchronouse + X know merger behaviour), 
and even so, we can't just assume that the merger node takes maps from topics in the order of arrival.
We also don't know what happens if the map arrival is not in sync with the merger node (whether it waits for other maps to arrive or just merge without some map - 
in this case the order will go completely wrong)
5. Concluded that we can't mimic decentre without understanding the merger exactly. It would have been better to implement our own. But due to time 

"""