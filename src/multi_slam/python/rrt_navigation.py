#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion

# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
  import rrt
except ImportError:
  raise ImportError('Unable to import potential_field.py. Make sure this file is in "{}"'.format(directory))
# added to learn about the laserscan range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped


SPEED = .2
EPSILON = .1
REACH_EPS = rrt.REACH_EPS

X = 0
Y = 1
YAW = 2

parser = argparse.ArgumentParser(description='rrt_nav args')
parser.add_argument('--name', action='store', default='turtlebot3_burger', help='Method.')
args, unknown = parser.parse_known_args()

ROBOT_NS = args.name #rospy.get_param("/rrt_navigation/robot_ns")

MAP_TOPIC_NAME = rospy.get_param("/rrt_navigation_{}/map_topic_name".format(ROBOT_NS))
GOAL_TOPIC_NAME = rospy.get_param("/rrt_navigation_{}/goal_topic_name".format(ROBOT_NS))

if ROBOT_NS != "":
  ROBOT_NS = '/' + ROBOT_NS
MAP_TOPIC = ROBOT_NS + '/' + MAP_TOPIC_NAME
GOAL_TOPIC = ROBOT_NS + '/' + GOAL_TOPIC_NAME


def feedback_linearized(pose, velocity, epsilon):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # Implement feedback-linearization to follow the velocity
  # vector given as argument. Epsilon corresponds to the distance of
  # linearized point in front of the robot.

  u = np.dot(velocity, np.array([  np.cos(pose[YAW]), np.sin(pose[YAW])], dtype=np.float32))
  w = np.dot(velocity, np.array([- np.sin(pose[YAW]), np.cos(pose[YAW])], dtype=np.float32)) / epsilon

  return u, w


def get_velocity(position, path_points):
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < REACH_EPS:
    return v

  # Return the velocity needed to follow the
  # path defined by path_points. Assume holonomicity of the
  # point located at position.

  def point_segment_dist(point, p1, p2):
    dist = 0
    p1_p2 = p2 - p1
    p1_point = point - p1
    dot = np.dot(p1_point, p1_p2)
    length = np.linalg.norm(p1_p2, 2)
    param = dot / length
    direction = np.zeros_like(p1)
    if param <= 1 and param >= 0:
      xx_yy = p1 + param * p1_p2
      dist = np.linalg.norm(point - xx_yy, 2)
      direction = (point - xx_yy) / dist
    elif param > 1:
      dist = np.linalg.norm(p2 - point, 2)
      direction = (p2 - point) / dist
    # elif param < 0:
    else:
      dist = np.linalg.norm(p1 - point, 2)
      direction = (p1 - point) / dist

    return dist, direction

  distances = []

  for i in range(1, len(path_points)):
    distances.append(point_segment_dist(position, path_points[i-1], path_points[i])[0])

  min_d = min(distances)
  arg_min = int(np.argmin(np.array(distances)))
  point1, point2 = path_points[arg_min] , path_points[arg_min+1]
  if min_d < 0.1:
    v = (point2 - position) / np.linalg.norm((point2 - position), 2)
    
  else:
    v = point_segment_dist(position, point1, point2)[1]
    

  # v = np.array([1,1])


  return v * SPEED


class SLAM(object):
  def __init__(self):
    rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.callback)
    self._tf = TransformListener()
    self._occupancy_grid = None
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    
  def callback(self, msg):
    values = np.array(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
    processed = np.empty_like(values)
    processed[:] = rrt.FREE
    processed[values < 0] = rrt.UNKNOWN
    processed[values > 50] = rrt.OCCUPIED
    processed = processed.T
    origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
    resolution = msg.info.resolution
    self._occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)

  def update(self):
    # Get pose w.r.t. map.
    a = 'occupancy_grid'
    b = ROBOT_NS[1:] + '/base_link' if ROBOT_NS != "" else 'base_link'
    if self._tf.frameExists(a) and self._tf.frameExists(b):
      try:
        t = rospy.Time(0)
        position, orientation = self._tf.lookupTransform('/' + a, '/' +  b, t)
        self._pose[X] = position[X]
        self._pose[Y] = position[Y]
        _, _, self._pose[YAW] = euler_from_quaternion(orientation)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))
    pass

  @property
  def ready(self):
    return self._occupancy_grid is not None and not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose

  @property
  def occupancy_grid(self):
    return self._occupancy_grid


# class GoalPose(object):
#   def __init__(self):
#     rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.callback)
#     self._position = np.array([np.nan, np.nan], dtype=np.float32)
#
#   def callback(self, msg):
#     # The pose from RViz is with respect to the "map".
#     self._position[X] = msg.pose.position.x
#     self._position[Y] = msg.pose.position.y
#     print('Received new goal position:', self._position)
#
#   @property
#   def ready(self):
#     return not np.isnan(self._position[0])
#
#   @property
#   def position(self):
#     return self._position



class Explortaion(object):
  RANDOM_SAMPLE_UPPER_LIMIT = 400
  CONNECTIVITY_MAX_ITERATIONS = 500

  def __init__(self, slam_obj):
    self._slam_obj = slam_obj
    self._merged_occupancy_grid = None
    self._next_dest = None
    # success meaning don't need to explore anymore
    self._success = False
    self.scan_sub = rospy.Subscriber(ROBOT_NS + '/scan', LaserScan, self.get_range_callback)
    self.merged_map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.get_merged_map_callback)

    # show good pos
    self._good_pos_pub = rospy.Publisher('~good_pos', PointStamped, queue_size=1)
    self.rrt_final_node = None

  def get_range_callback(self, msg):
    self.laser_min_range = msg.range_min
    self.laser_max_range = msg.range_max
    self.min_of_reading = float(np.min(msg.ranges))
    self.max_of_reading = float(np.max(msg.ranges[msg.ranges != np.inf]))
    
  def get_merged_map_callback(self, msg):
    values = np.array(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
    processed = np.empty_like(values)
    processed[:] = rrt.FREE
    processed[values < 0] = rrt.UNKNOWN
    processed[values > 50] = rrt.OCCUPIED
    processed = processed.T
    origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
    resolution = msg.info.resolution
    self._merged_occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)


  def get_next_dest_for_local(self):
    # get the max size of the map and use it as max_dist
    grid_shape = self._merged_occupancy_grid.values.shape
    map_max_size = max(grid_shape[0], grid_shape[1])
    # design the probability decay function, wrt the laser ranges and max map size
    min_dist = (self.min_of_reading + self.max_of_reading) / 2
    max_dist = self.laser_max_range
    sigma = (self.max_of_reading - self.min_of_reading) / 2
    if np.isnan(sigma) or np.isinf(sigma):
      sigma = (self.laser_max_range - self.laser_min_range) / 2
    if np.isnan(min_dist) or np.isinf(min_dist):
      min_dist = self.laser_max_range


    # todo
    self.probability_decay_funciton = lambda d: np.random.randn() * sigma + min_dist

    dest_found = False
    i = 0
    # for debug
    outside_time = 0
    unknown_time = 0
    while not dest_found and i < self.RANDOM_SAMPLE_UPPER_LIMIT:
      i += 1
      sampled_dir = np.random.rand() * 2 * np.pi
      sampled_dir = np.array([np.cos(sampled_dir), np.sin(sampled_dir)])
      sampled_dist = self.probability_decay_funciton(None)
      pos = sampled_dir * sampled_dist + self._slam_obj.pose[:2]
      if self._merged_occupancy_grid.is_outside(pos) or not self._merged_occupancy_grid.is_unknown(pos):
        if self._merged_occupancy_grid.is_outside(pos):
          outside_time += 1
        if not self._merged_occupancy_grid.is_unknown(pos):
          unknown_time += 1
        continue
      else:
        print('good pos! ', pos)
        # for debug
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "map"
        point_stamped.point.x = pos[0]
        point_stamped.point.y = pos[1]
        self._good_pos_pub.publish(point_stamped)
        s, f = Explortaion.check_connectivity(self._slam_obj.pose, pos, self._merged_occupancy_grid)
        if f is None:
          print("no path to ", pos)
          continue
        else:
          # now we found the next_dest
          self._next_dest = pos
          self.rrt_final_node = f
          dest_found = True
          print('found next dest ', pos)
          return

    print("outside, ", outside_time, " unknown ", unknown_time, ' in ', self.RANDOM_SAMPLE_UPPER_LIMIT)

    # if we cannot find even one goal, we say we succeeded in exploring!
    self._success = True


  @staticmethod
  def check_connectivity(start_pose, goal_position, occupancy_grid):
    # RRT builds a graph one node at a time.
    graph = []
    start_node = rrt.Node(start_pose)
    final_node = None

    graph.append(start_node)
    for _ in range(Explortaion.CONNECTIVITY_MAX_ITERATIONS):
      position = rrt.sample_random_position(occupancy_grid)
      # With a random chance, draw the goal position.
      if np.random.rand() < .05:
        position = goal_position
      # Find closest node in graph.
      # In practice, one uses an efficient spatial structure (e.g., quadtree).
      potential_parent = sorted(((n, np.linalg.norm(position - n.position)) for n in graph), key=lambda x: x[1])
      # Pick a node at least some distance away but not too far.
      # We also verify that the angles are aligned (within pi / 4).
      u = None
      for n, d in potential_parent:
        if d > REACH_EPS and d < 1.5 and n.direction.dot(position - n.position) / d > 0.70710678118:
          u = n
          break
      else:
        continue
      v = rrt.adjust_pose(u, position, occupancy_grid)
      if v is None:
        continue
      u.add_neighbor(v)
      v.parent = u
      graph.append(v)
      if np.linalg.norm(v.position - goal_position) < REACH_EPS:
        final_node = v
        break
    return start_node, final_node

  # def keep_explore(self):
  #   if self._success:
  #     # already succeeded
  #     # todo go to your friends
  #     pass
  #   else:
  #     self.get_next_dest_for_local()

  @property
  def ready(self):
    if self._next_dest is None and self._merged_occupancy_grid is not None:
      # do the getting next destination for the first time
      self.get_next_dest_for_local()
    # if self.current_goal_reached:
    #   self.get_next_dest_for_local()

    return not self._success and (self._next_dest is not None)

  @property
  def position(self):
    if self._next_dest is None:
      self.get_next_dest_for_local()

    return self._next_dest

  @property
  def success(self):
    return self._success

  @property
  def current_goal_reached(self):
    return self._next_dest is None or \
           np.linalg.norm(self._slam_obj.pose[:2] + \
                          np.array([np.cos(self._slam_obj.pose[2]), np.sin(self._slam_obj.pose[2])]) * EPSILON \
                                                     - self._next_dest) < REACH_EPS


def get_path(final_node):
  # Construct path from RRT solution.
  if final_node is None:
    return []
  path_reversed = []
  path_reversed.append(final_node)
  while path_reversed[-1].parent is not None:
    path_reversed.append(path_reversed[-1].parent)
  path = list(reversed(path_reversed))
  # Put a point every 5 cm.
  distance = 0.05
  offset = 0.
  points_x = []
  points_y = []
  for u, v in zip(path, path[1:]):
    center, radius = rrt.find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    clockwise = np.cross(u.direction, du).item() > 0.
    # Generate a point every 5cm apart.
    da = distance / radius
    offset_a = offset / radius
    if clockwise:
      da = -da
      offset_a = -offset_a
      if theta2 > theta1:
        theta2 -= 2. * np.pi
    else:
      if theta2 < theta1:
        theta2 += 2. * np.pi
    angles = np.arange(theta1 + offset_a, theta2, da)
    offset = distance - (theta2 - angles[-1]) * radius
    points_x.extend(center[X] + np.cos(angles) * radius)
    points_y.extend(center[Y] + np.sin(angles) * radius)
  return zip(points_x, points_y)
  

def run(args):

  rospy.init_node('rrt_navigation')
  exploration_start = rospy.Time.now().to_sec()

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher(ROBOT_NS + '/cmd_vel', Twist, queue_size=5)
  path_publisher = rospy.Publisher(ROBOT_NS + '/path', Path, queue_size=1)
  slam = SLAM()
  goal = Explortaion(slam)
  frame_id = 0
  current_path = []
  previous_time = rospy.Time.now().to_sec()

  # Stop moving message.
  stop_msg = Twist()
  stop_msg.linear.x = 0.
  stop_msg.angular.z = 0.

  # Make sure the robot is stopped.
  i = 0
  while i < 10 and not rospy.is_shutdown():
    publisher.publish(stop_msg)
    rate_limiter.sleep()
    i += 1

  while not rospy.is_shutdown():
    if goal.success:
      print(rospy.Time.now().to_sec() - exploration_start, 's of exploration. finished! robot ', ROBOT_NS)
      i = 0
      while i < 10 and not rospy.is_shutdown():
        publisher.publish(stop_msg)
        rate_limiter.sleep()
        i += 1

      # for debug
      print('the grid ', slam.occupancy_grid.values.shape,' length:', float(slam.occupancy_grid.values.shape[0]) * slam.occupancy_grid.resolution)
      print(slam.occupancy_grid.values)
      break
      continue

    slam.update()
    current_time = rospy.Time.now().to_sec()

    # Make sure all measurements are ready.
    # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch
    if not slam.ready or not goal.ready:
      rate_limiter.sleep()
      continue

    # now we need to get the goal
    # if the goal is ready but not reached
    if not goal.current_goal_reached:
      # stick to the original goal
      print('now for goal ', goal.position[0], ' ', goal.position[1])
      # but remember to clear the RRT path calculated inside the exploration class
      goal.rrt_final_node = None
    # else start a new goal
    else:
      # Make sure the robot is stopped.
      i = 0
      while i < 10 and not rospy.is_shutdown():
        publisher.publish(stop_msg)
        rate_limiter.sleep()
        i += 1
      print('old goal reached, now for new goal')
      goal.rrt_final_node = None
      goal.get_next_dest_for_local()
      continue

    # goal_reached = goal.current_goal_reached
    # if goal_reached:
    #   publisher.publish(stop_msg)
    #   rate_limiter.sleep()
    #   continue

    # Follow path using feedback linearization.
    position = np.array([
        slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
        slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
    v = get_velocity(position, np.array(current_path, dtype=np.float32))
    u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
    vel_msg = Twist()
    vel_msg.linear.x = u
    vel_msg.angular.z = w
    publisher.publish(vel_msg)

    # Update plan every 1s.
    time_since = current_time - previous_time
    if current_path and time_since < 2.:
      rate_limiter.sleep()
      continue
    previous_time = current_time

    # Run RRT.
    # if we have got an RRT path from the class, return it
    if goal.rrt_final_node is None:
      start_node, final_node = rrt.rrt(slam.pose, goal.position, slam.occupancy_grid)
    else:
      final_node = goal.rrt_final_node
    current_path = get_path(final_node)
    if not current_path:
      print('Unable to reach goal position:', goal.position, ' now change goal')
      # Make sure the robot is stopped.
      i = 0
      while i < 10 and not rospy.is_shutdown():
        publisher.publish(stop_msg)
        rate_limiter.sleep()
        i += 1

      goal._next_dest = None
      goal.rrt_final_node = None
      # goal.get_next_dest_for_local()
    
    # Publish path to RViz.
    path_msg = Path()
    path_msg.header.seq = frame_id
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'map'
    for u in current_path:
      pose_msg = PoseStamped()
      pose_msg.header.seq = frame_id
      pose_msg.header.stamp = path_msg.header.stamp
      pose_msg.header.frame_id = 'map'
      pose_msg.pose.position.x = u[X]
      pose_msg.pose.position.y = u[Y]
      path_msg.poses.append(pose_msg)
    path_publisher.publish(path_msg)

    rate_limiter.sleep()
    frame_id += 1


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs RRT navigation')
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass
