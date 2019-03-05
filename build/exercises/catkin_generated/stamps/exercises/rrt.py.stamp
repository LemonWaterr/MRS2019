from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re
import scipy.signal
import yaml
import random
import math

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.105 / 2.
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)  # Any orientation is good.
START_POSE = np.array([-1.5, -1.5, 0.], dtype=np.float32)
MAX_ITERATIONS = 500


def sample_random_position(occupancy_grid):
  position = np.zeros(2, dtype=np.float32)
  # MISSING: Sample a valid random position (do not sample the yaw).
  # The corresponding cell must be free in the occupancy grid.
  
  #I have to hardcode the limits, because the actual map spans from (-2,-2) to (2,2) while the occupancy grid spans from (-10,-10) to (10,10)
  #width = occupancy_grid.values.shape[1]
  #height = occupancy_grid.values.shape[0]
  #end_pos1 = occupancy_grid.get_position(0,0)
  #end_pos2 = occupancy_grid.get_position(height, width)
  #x = random.uniform(end_pos1[0], end_pos2[0])
  #y = random.uniform(end_pos1[1], end_pos2[1])
  
  x = random.uniform(-2., 2.)
  y = random.uniform(-2., 2.)
  position[0], position[1] = x, y
  
  while not occupancy_grid.is_free(position):
    x = random.uniform(-2., 2.)
    y = random.uniform(-2., 2.)
    position[0], position[1] = x, y
  
  return position

#custom functions
def magnitude2D(v):
  return math.sqrt(v[0]**2+v[1]**2)

def normalize2D(v):
  return v / magnitude2D(v)

def rotate2D(v, rad):
  return np.array([np.cos(rad)*v[0] - np.sin(rad)*v[1], np.sin(rad)*v[0] + np.cos(rad)*v[1]])

def bearing(v):
  #1st quadrant
  if v[0] >=  0.0 and v[1] >  0.0:
    return np.arctan(float(v[0])/v[1])
  #2nd quadrant
  elif v[0] > 0.0 and v[1] <=  0.0:
    return np.pi/2. + np.arctan(float(-v[1])/v[0])
  #3rd quadrant
  elif v[0] <= 0.0 and v[1] < 0.0:
    return np.pi + np.arctan(float(v[0])/v[1])
  elif v[0] < 0.0 and v[1] >= 0.0:
    return np.pi * 3./2. + np.arctan(float(v[1])/-v[0])
  else:
    print("bearing calculation error: v: []".format(v))
    return 0.0
  
  
#finding circle with node_a, node_b, and node_a's YAW only
#explanation in the report
def find_circle(node_a, node_b):
  
  def perpendicular(v):
    w = np.zeros(2)
    w[0] = -v[1]
    w[1] = v[0]
    return w
  
  M = np.array((node_a.position + node_b.position)) / 2.
  i, j = M[0], M[1]
  t = np.array(node_a.direction)
  m = normalize2D(perpendicular(node_a.position - node_b.position)) #normalizing is optional
  k = ((node_a.position[0]-i)*t[0] + (node_a.position[1]-j)*t[1]) / (m[0]*t[0] + m[1]*t[1])
  center = M + k * m
  radius = magnitude2D(node_a.position - center)
  
  return center, radius

def adjust_pose(node, final_position, occupancy_grid):
  final_pose = node.pose.copy()
  final_pose[:2] = final_position
  final_node = Node(final_pose)
  
  # MISSING: Check whether there exists a simple path that links node.pose
  # to final_position. This function needs to return a new node that has
  # the same position as final_position and a valid yaw. The yaw is such that
  # there exists an arc of a circle that passes through node.pose and the
  # adjusted final pose. If no such arc exists (e.g., collision) return None.
  # Assume that the robot always goes forward.
  # Feel free to use the find_circle() function below.  
  
  circle = find_circle(node,final_node)
  center, radius = circle[0], circle[1]
  #find out if robot is moving long the circle clockwise or anticlockwise
  go_clockwise = True
  quadrant = 0
  if node.position[0] > center[0] and node.position[1] > center[1]:   #quadrant 1
    go_clockwise = True if node.direction[0] > 0.0 or node.direction[1] < 0.0 else False
  elif node.position[0] > center[0] and node.position[1] < center[1]: #quadrant 2
    go_clockwise = True if node.direction[0] < 0.0 or node.direction[1] < 0.0 else False
  elif node.position[0] < center[0] and node.position[1] < center[1]: #quadrant 3
    go_clockwise = True if node.direction[0] < 0.0 or node.direction[1] > 0.0 else False
  elif node.position[0] < center[0] and node.position[1] > center[1]: #quadrant 4
    go_clockwise = True if node.direction[0] > 0.0 or node.direction[1] > 0.0 else False
    
  elif node.position[0] > center[0] and node.position[1] == center[1]: #on x axis, right
    go_clockwise = True if node.direction[1] < 0.0 else False
  elif node.position[0] < center[0] and node.position[1] == center[1]: #on x axis, left
    go_clockwise = True if node.direction[1] > 0.0 else False
  elif node.position[0] == center[0] and node.position[1] > center[1]: #on y axis, above
    go_clockwise = True if node.direction[0] > 0.0 else False
  elif node.position[0] == center[0] and node.position[1] < center[1]: #on y axis, below
    go_clockwise = True if node.direction[1] < 0.0 else False
  else:
    print("node is on the center, which is weird. center: {} node: {}".format(center, node.position))
  
  #get bearings from center to node/final_position to get which direction to trace the circle
  node_bearing = bearing(node.pose[:2] - center)
  final_bearing = bearing(final_position - center)      
  trace_step = 0.02 #tracing 0.001 radians a time
  
  def addAngles(a, b):
    if a+b > np.pi*2:
      return a+b - np.pi*2
    elif a+b < 0.0:
      return np.pi*2 - a+b
    else:
      return a+b
  
  #trace and verify the arc  
  current_bearing = addAngles(node_bearing, trace_step) if go_clockwise else addAngles(node_bearing, -trace_step)
  #print("target_b: {} current_b: {}".format(final_bearing/np.pi * 180, current_bearing/np.pi * 180))
  while abs(addAngles(final_bearing, -current_bearing)) > 0.05:
    target_pos = center + rotate2D([0., radius], -current_bearing)
    if occupancy_grid.is_free(target_pos):
      current_bearing = addAngles(current_bearing, trace_step) if go_clockwise else addAngles(current_bearing, -trace_step)
      continue
    else:
      '''
      print("this does happen: {}".format(target_pos))
      print("center: {} node_bear: {} final_bear: {} current_bear: {}".format(center, node_bearing/np.pi * 180, final_bearing/np.pi * 180, current_bearing/np.pi * 180))
      '''
      return None
  
  #get tangent of final_position on the circle to get YAW
  #there are two tangent vectors for a point on circle, we need to choose the one that heads towards the path
  #we only need to know the angle, so we use the trace_step to know the direction and add/subtract pi/2 to the final_bearing to get the YAW
  final_yaw = final_bearing
  if trace_step < 0.0:
    final_yaw = final_yaw - np.pi/2. -np.pi/2. #-np.pi/2 since the bearing is based on the y axis but pose is based on the x axis
  else:
    final_yaw = final_yaw + np.pi/2. -np.pi/2.
  
  #apply the yaw
  final_pose[2] = final_yaw
  final_node = Node(final_pose)
  
  return final_node


# Defines an occupancy grid.
class OccupancyGrid(object):
  def __init__(self, values, origin, resolution):
    self._original_values = values.copy()
    self._values = values.copy()
    # Inflate obstacles (using a convolution).
    inflated_grid = np.zeros_like(values)
    inflated_grid[values == OCCUPIED] = 1.
    w = 2 * int(ROBOT_RADIUS / resolution) + 1
    inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:2], dtype=np.float32)
    self._origin -= resolution / 2.
    assert origin[YAW] == 0.
    self._resolution = resolution

  @property
  def values(self):
    return self._values

  @property
  def resolution(self):
    return self._resolution

  @property
  def origin(self):
    return self._origin

  def draw(self):
    plt.imshow(self._original_values.T, interpolation='none', origin='lower',
               extent=[self._origin[X],
                       self._origin[X] + self._values.shape[0] * self._resolution,
                       self._origin[Y],
                       self._origin[Y] + self._values.shape[1] * self._resolution])
    plt.set_cmap('gray_r')

  def get_index(self, position):
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
    return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

  def is_occupied(self, position):
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    return self._values[self.get_index(position)] == FREE


# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:2]

  @property
  def yaw(self):
    return self._pose[YAW]
  
  @property
  def direction(self):
    return np.array([np.cos(self._pose[YAW]), np.sin(self._pose[YAW])], dtype=np.float32)

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c


def rrt(start_pose, goal_position, occupancy_grid):
  # RRT builds a graph one node at a time.
  graph = []
  start_node = Node(start_pose)
  final_node = None
  if not occupancy_grid.is_free(goal_position):
    print('Goal position is not in the free space.')
    return start_node, final_node
  graph.append(start_node)
  for i in range(MAX_ITERATIONS): 
    position = sample_random_position(occupancy_grid)
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
      if d > .2 and d < 1.5 and n.direction.dot(position - n.position) / d > 0.70710678118:
        u = n
        break
    else:
      continue
    #print("found near/node after {} iter".format(i))
    v = adjust_pose(u, position, occupancy_grid)
    if v is None:
      #print("arc not valid")
      continue
    #print("arc valid, add the finalPos")
    #print("u: {} v: {}".format(u.pose,v.pose))
    u.add_neighbor(v)
    v.parent = u
    graph.append(v)
    if np.linalg.norm(v.position - goal_position) < .2:
      final_node = v
      break
  return start_node, final_node


def read_pgm(filename, byteorder='>'):
  """Read PGM file."""
  with open(filename, 'rb') as fp:
    buf = fp.read()
  try:
    header, width, height, maxval = re.search(
        b'(^P5\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n])*'
        b'(\d+)\s(?:\s*#.*[\r\n]\s)*)', buf).groups()
  except AttributeError:
    raise ValueError('Invalid PGM file: "{}"'.format(filename))
  maxval = int(maxval)
  height = int(height)
  width = int(width)
  img = np.frombuffer(buf,
                      dtype='u1' if maxval < 256 else byteorder + 'u2',
                      count=width * height,
                      offset=len(header)).reshape((height, width))
  return img.astype(np.float32) / 255.


def draw_solution(start_node, final_node=None):
  ax = plt.gca()

  def draw_path(u, v, arrow_length=.1, color=(.8, .8, .8), lw=1):
    du = u.direction
    plt.arrow(u.pose[X], u.pose[Y], du[0] * arrow_length, du[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    dv = v.direction
    plt.arrow(v.pose[X], v.pose[Y], dv[0] * arrow_length, dv[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    center, radius = find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    if np.cross(u.direction, du).item() > 0.:
      theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(center, radius * 2., radius * 2.,
                             theta1=theta1 / np.pi * 180., theta2=theta2 / np.pi * 180.,
                             color=color, lw=lw))

  points = []
  s = [(start_node, None)]  # (node, parent).
  while s:
    v, u = s.pop()
    if hasattr(v, 'visited'):
      continue
    v.visited = True
    # Draw path from u to v.
    if u is not None:
      draw_path(u, v)
    points.append(v.pose[:2])
    for w in v.neighbors:
      s.append((w, v))

  points = np.array(points)
  plt.scatter(points[:, 0], points[:, 1], s=10, marker='o', color=(.8, .8, .8))
  if final_node is not None:
    plt.scatter(final_node.position[0], final_node.position[1], s=10, marker='o', color='k')
    # Draw final path.
    v = final_node
    while v.parent is not None:
      draw_path(v.parent, v, color='k', lw=2)
      v = v.parent


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Uses RRT to reach the goal.')
  parser.add_argument('--map', action='store', default='map', help='Which map to use.')
  args, unknown = parser.parse_known_args()

  # Load map.
  with open(args.map + '.yaml') as fp:
    data = yaml.load(fp)
  img = read_pgm(os.path.join(os.path.dirname(args.map), data['image']))
  occupancy_grid = np.empty_like(img, dtype=np.int8)
  occupancy_grid[:] = UNKNOWN
  occupancy_grid[img < .1] = OCCUPIED
  occupancy_grid[img > .9] = FREE
  # Transpose (undo ROS processing).
  occupancy_grid = occupancy_grid.T
  # Invert Y-axis.
  occupancy_grid = occupancy_grid[:, ::-1]
  occupancy_grid = OccupancyGrid(occupancy_grid, data['origin'], data['resolution'])

  # Run RRT.
  start_node, final_node = rrt(START_POSE, GOAL_POSITION, occupancy_grid)

  # Plot environment.
  fig, ax = plt.subplots()
  occupancy_grid.draw()
  plt.scatter(.3, .2, s=10, marker='o', color='green', zorder=1000)
  draw_solution(start_node, final_node)
  plt.scatter(START_POSE[0], START_POSE[1], s=10, marker='o', color='green', zorder=1000)
  plt.scatter(GOAL_POSITION[0], GOAL_POSITION[1], s=10, marker='o', color='red', zorder=1000)
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - 2., 2. + .5])
  plt.ylim([-.5 - 2., 2. + .5])
  plt.show()
  