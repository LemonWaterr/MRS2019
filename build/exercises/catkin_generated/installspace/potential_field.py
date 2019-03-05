from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import numpy as np
import math
import random


WALL_OFFSET = 2.
#CYLINDER_POSITION = np.array([0.0, 0.0], dtype=np.float32)
#CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)
CYLINDER_POSITION1 = np.array([.5, 0.0], dtype=np.float32)
CYLINDER_POSITION2 = np.array([0.0, 0.5], dtype=np.float32)
CYLINDER_RADIUS = .3
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
START_POSITION = np.array([-1.5, -1.5], dtype=np.float32)
MAX_SPEED = .5

def magnitude2D(v):
  return math.sqrt(v[0]**2+v[1]**2)

start2goal = math.sqrt(magnitude2D(START_POSITION)**2 + magnitude2D(GOAL_POSITION)**2)

def normalize2D(v):
  return v / magnitude2D(v)

def updateMag(v, mag):
  old_mag = magnitude2D(v)
  return np.array([v[0] * mag / old_mag, v[1] * mag / old_mag], dtype=np.float32)
                  
def get_velocity_to_reach_goal(position, goal_position):
  v = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to reach goal_position
  # assuming that there are no obstacles.
  distance = magnitude2D(goal_position - position)
  velocity = updateMag(goal_position-position, MAX_SPEED * (distance/start2goal))
  
  #to make robot come back to the goal when it gets past it
  
  extra = goal_position - position
  if magnitude2D(extra) < 1.3 and magnitude2D(extra) > 0.3 :
    v_extra = updateMag(extra, max(MAX_SPEED*0.3, extra[0]**2 + extra[1]**2))
    velocity += v_extra
 
  return cap(velocity, max_speed=MAX_SPEED)

def get_velocity_to_avoid_obstacles(position, obstacle_positions, obstacle_radii):
  v_ret = np.zeros(2, dtype=np.float32)
  # MISSING: Compute the velocity field needed to avoid the obstacles
  # In the worst case there might a large force pushing towards the
  # obstacles (consider what is the largest force resulting from the
  # get_velocity_to_reach_goal function). Make sure to not create
  # speeds that are larger than max_speed for each obstacle. Both obstacle_positions
  # and obstacle_radii are lists.
  
  #rotating anticlockwise
  def rotate2D(v, rad):
    return np.array([np.cos(rad)*v[0] - np.sin(rad)*v[1], np.sin(rad)*v[0] + np.cos(rad)*v[1]])
    
  for p, r in zip(obstacle_positions, obstacle_radii):
    obj2pos = position - p
    distance = magnitude2D(obj2pos)
    v_add = np.zeros(2, dtype=np.float32)
    
    goal2start = START_POSITION - GOAL_POSITION
    
    #if angle between two vectors, goal to start and obstacle to position, is obtuse, go for normal way
    if np.arccos(min(1.0, np.dot(obj2pos, goal2start) / (magnitude2D(obj2pos)+magnitude2D(goal2start)))) > np.pi/2.:
      if distance < 0.000001: #if distance == 0.0
        v_add = updateMag(position - GOAL_POSITION, MAX_SPEED)  #other way round because we will subtract this later
      elif distance <= r:
        v_add = updateMag(obj2pos, MAX_SPEED)
      else:   
        v_add = updateMag(obj2pos, max(0.0, -obj2pos[0]**2 -obj2pos[1]**2 + MAX_SPEED)) #hyperbolic cone-like surface

      #resolving ex1e: every 0.1 meters, give it alternating 0.1~0.2 radian rotation
      if distance % 0.2 < 0.1:
        v_add = rotate2D(v_add, random.uniform(0.1, 0.2))
      else:
        v_add = rotate2D(v_add, -random.uniform(0.1, 0.2))

    #resolving ex1f: if angle between two vectors, goal to start and obstacle to position, is acute, just add perpendicular force
    else:
      if distance > 2*r:
        continue
      else:
        direction = 1. if np.cross(goal2start, obj2pos) > 0.0 else -1 #positive if obj2pos in anticlockwise direction of goal2start, negative if clockwise
        v_add = updateMag(rotate2D(goal2start, np.pi/2. * direction), -(MAX_SPEED/2*r)*distance + MAX_SPEED)
        
        
    v_ret += v_add
      
  return v_ret


def normalize(v):
  n = np.linalg.norm(v)
  if n < 1e-2:
    return np.zeros_like(v)
  return v / n


def cap(v, max_speed):
  n = np.linalg.norm(v)
  if n > max_speed:
    return v / n * max_speed
  return v


def get_velocity(position, mode='all'):
  if mode in ('goal', 'all'):
    v_goal = get_velocity_to_reach_goal(position, GOAL_POSITION)
  else:
    v_goal = np.zeros(2, dtype=np.float32)
  if mode in ('obstacle', 'all'):
    v_avoid = get_velocity_to_avoid_obstacles(
      position,
      [CYLINDER_POSITION1, CYLINDER_POSITION2],
      [CYLINDER_RADIUS, CYLINDER_RADIUS])
  else:
    v_avoid = np.zeros(2, dtype=np.float32)
  v = v_goal + v_avoid
  return cap(v, max_speed=MAX_SPEED)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Runs obstacle avoidance with a potential field')
  parser.add_argument('--mode', action='store', default='all', help='Which velocity field to plot.', choices=['obstacle', 'goal', 'all'])
  args, unknown = parser.parse_known_args()

  fig, ax = plt.subplots()
  # Plot field.
  X, Y = np.meshgrid(np.linspace(-WALL_OFFSET, WALL_OFFSET, 30),
                     np.linspace(-WALL_OFFSET, WALL_OFFSET, 30))
  U = np.zeros_like(X)
  V = np.zeros_like(X)
  for i in range(len(X)):
    for j in range(len(X[0])):
      velocity = get_velocity(np.array([X[i, j], Y[i, j]]), args.mode)
      U[i, j] = velocity[0]
      V[i, j] = velocity[1]
  plt.quiver(X, Y, U, V, units='width')

  # Plot environment.
  ax.add_artist(plt.Circle(CYLINDER_POSITION1, CYLINDER_RADIUS, color='gray'))
  ax.add_artist(plt.Circle(CYLINDER_POSITION2, CYLINDER_RADIUS, color='gray'))
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, -WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, WALL_OFFSET], [WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([-WALL_OFFSET, -WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')
  plt.plot([WALL_OFFSET, WALL_OFFSET], [-WALL_OFFSET, WALL_OFFSET], 'k')

  # Plot a simple trajectory from the start position.
  # Uses Euler integration.
  dt = 0.01
  x = START_POSITION
  positions = [x]
  for t in np.arange(0., 20., dt):
    v = get_velocity(x, args.mode)
    x = x + v * dt
    positions.append(x)
  positions = np.array(positions)
  plt.plot(positions[:, 0], positions[:, 1], lw=2, c='r')

  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.ylim([-.5 - WALL_OFFSET, WALL_OFFSET + .5])
  plt.show()