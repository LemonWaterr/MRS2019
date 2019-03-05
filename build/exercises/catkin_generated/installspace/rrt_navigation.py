#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys
import math

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


SPEED = .2
EPSILON = .1

X = 0
Y = 1
YAW = 2

def normalize2D(v):
  return v / magnitude2D(v)

def magnitude2D(v):
  return math.sqrt(v[0]**2+v[1]**2)

#rotates anticlockwise
def rotate2D(v, rad):
  return np.array([np.cos(rad)*v[0] - np.sin(rad)*v[1], np.sin(rad)*v[0] + np.cos(rad)*v[1]])

def updateMag(v, mag):
  old_mag = magnitude2D(v)
  return np.array([v[0] * mag / old_mag, v[1] * mag / old_mag], dtype=np.float32)

def feedback_linearized(pose, velocity, epsilon):
  u = 0.  # [m/s]
  w = 0.  # [rad/s] going counter-clockwise.

  # MISSING: Implement feedback-linearization to follow the velocity
  # vector given as argument. Epsilon corresponds to the distance of
  # linearized point in front of the robot.
  
  vel_world = rotate2D(velocity, -pose[YAW])  #this should replace the next line if USE_RELATIVE_POSITIONS = False
  #vel_world = velocity
  
  speed = math.sqrt(vel_world[0]**2+vel_world[1]**2)
  angularV = np.arctan(vel_world[1]/vel_world[0])
  xpdot = speed * np.cos(pose[YAW]) + epsilon * angularV * -np.sin(pose[YAW])
  ypdot = speed * np.sin(pose[YAW]) + epsilon * angularV * np.cos(pose[YAW])
  
  u = xpdot * np.cos(pose[YAW]) + ypdot * np.sin(pose[YAW])
  w = (-xpdot * np.sin(pose[YAW]) + ypdot * np.cos(pose[YAW])) / epsilon
  
  return u, w

def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v

def get_velocity(position, path_points):
  v = np.zeros_like(position)
  if len(path_points) == 0:
    return v
  # Stop moving if the goal is reached.
  if np.linalg.norm(position - path_points[-1]) < .2:
    return v

  # MISSING: Return the velocity needed to follow the
  # path defined by path_points. Assume holonomicity of the
  # point located at position.
  
  '''
  def getSquare(path_pair):
    def perpendicular(v):
      w = np.zeros(2)
      w[0] = -v[1]
      w[1] = v[0]
      return w
    line = path_pair[1] - path_pair[0]
    thickness = magnitude2D(line) * 0.05
    vec = normalize2D(perpendicular(line))
    return path_pair[0]-vec*thickness, path_pair[1]+vec*thickness
  
  for i in range(0, path_points.shape[0]-1):
    vec = path_points[i+1] - path_points[i]
    vertices = getSquare([path_points[i], path_points[i+1]])
    v1, v2 = vertices[0], vertices[1]
    print("pp{}: {} pp{}: {} v1: {} v2: {} position: {}".format(i, path_points[i], i+1, path_points[i+1], v1, v2, position))
    if position[0] < max(v1[0], v2[0]) and position[0] > min(v1[0], v2[0]) \
       and position[1] < max(v1[1], v2[1]) and position[1] > min(v1[1], v2[1]):
      v += normalize2D(vec) * SPEED
      print("between pathpoints {} -- {}".format(i, i+1))
  '''
  
  def angleFromVectors(v1, v2):
    return np.arccos(np.dot(v1,v2)/(magnitude2D(v1)*magnitude2D(v2)))
  
  #get a path line closest to the position
  closest_i = 0
  distance = sys.maxint
 
  for i in range(0, path_points.shape[0]-1):
    v_local = None
    #vector from a point p to line a+tn: (a-p)-dot((a-p),n)*n according to wikipedia
    a = path_points[i]
    n = normalize2D(path_points[i+1] - path_points[i])
    pos2line = (a-position) - np.dot((a-position), n)*n
    #point of intersection to the line
    intersection = position + pos2line
    
    #if intersection out of line, get the distance from position to one of the ends of the line
    if not (intersection[0] < max(path_points[i+1][0],path_points[i][0]) and intersection[0] > min(path_points[i+1][0],path_points[i][0]) \
       and  intersection[1] < max(path_points[i+1][1],path_points[i][1]) and intersection[1] > min(path_points[i+1][1],path_points[i][1])):
      if magnitude2D(path_points[i]-position) < magnitude2D(path_points[i+1]-position):
        pos2line = path_points[i] - position
      else:
        pos2line = path_points[i+1] - position
      pos2end = path_points[i+1]-position
    else:
      #vector from intersection to the path end
      inter2end = path_points[closest_i+1] - intersection
      #vector from position to path end
      pos2end = pos2line + inter2end
      #make sure it's not too quick
      #pos2end = updateMag(pos2end, magnitude2D(pos2end))
    v_local = cap(pos2end, max_speed=SPEED)
      
    #update closest line
    d = magnitude2D(pos2line)
    if d < distance:
      v = v_local
  
  #print("return velocity: {}".format(v))
  #print("pp{}: {} pp{}: {} pos: {} inter: {} vel: {}".format(closest_i, path_points[closest_i], closest_i+1, path_points[closest_i+1], position, intersection, v))
  
  return v


class SLAM(object):
  def __init__(self):
    rospy.Subscriber('/map1', OccupancyGrid, self.callback)
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
    b = 'base_link'
    if self._tf.frameExists(a) and self._tf.frameExists(b):
      try:
        t = rospy.Time(0)
        position, orientation = self._tf.lookupTransform('/' + a, '/' + b, t)
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


class GoalPose(object):
  def __init__(self):
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
    self._position = np.array([np.nan, np.nan], dtype=np.float32)

  def callback(self, msg):
    # The pose from RViz is with respect to the "map".
    self._position[X] = msg.pose.position.x
    self._position[Y] = msg.pose.position.y
    print('Received new goal position:', self._position)

  @property
  def ready(self):
    return not np.isnan(self._position[0])

  @property
  def position(self):
    return self._position


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

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  path_publisher = rospy.Publisher('/path', Path, queue_size=1)
  slam = SLAM()
  goal = GoalPose()
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
    slam.update()
    current_time = rospy.Time.now().to_sec()

    # Make sure all measurements are ready.
     # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch
    if not goal.ready or not slam.ready:
      rate_limiter.sleep()
      continue

    goal_reached = np.linalg.norm(slam.pose[:2] - goal.position) < .2
    if goal_reached:
      publisher.publish(stop_msg)
      rate_limiter.sleep()
      continue

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
    start_node, final_node = rrt.rrt(slam.pose, goal.position, slam.occupancy_grid)
    current_path = get_path(final_node)
    if not current_path:
      print('Unable to reach goal position:', goal.position)
    
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
