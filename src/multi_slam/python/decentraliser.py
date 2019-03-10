#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
import random
import math

# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
#from std_msgs.msg import String

class getDistance(object):
  def __init__(self, source, target):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self.pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self.source = source
    self.target = target
    self.dist = 99999.0

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self.source or n == self.target]
    if len(idx) != 2 :
      raise ValueError('Specified name "{} or {}" does not exist.'.format(self.source, self.target))
    x0 = msg.pose[idx[0]].position.x
    y0 = msg.pose[idx[0]].position.y
    x1 = msg.pose[idx[1]].position.x
    y1 = msg.pose[idx[1]].position.y
    self.dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def distance(self):
    return self.dist

class Intercept(object):  
  def __init__(self, target, merger, comm_range):
    self.target = target
    self.merger = merger
    self.comm_range = comm_range
    rospy.Subscriber('{}/map'.format(target), OccupancyGrid, self.callback_map)
    self.pub = rospy.Publisher('{}/map4{}'.format(target, merger), OccupancyGrid, queue_size=10)
    
  def callback_map(self, msg):
    if self.target == self.merger:
      self.pub.publish(msg)
    else:
      distance_get = getDistance(self.merger, self.target)
      distance = distance_get.distance

      if distance < self.comm_range:
        print("{} gets {}'s map since distance {} < {}".format(self.merger, self.target, distance, self.comm_range))
        self.pub.publish(msg)
      else:
        print("{}/{} {} > {}".format(self.merger, self.target, distance, self.comm_range))

    
    
def run():
  rospy.init_node('decentraliser')
  merger = 'tb3_0'
  comm_range = 5.0
  intercept1 = Intercept('tb3_0', merger, comm_range)
  intercept2 = Intercept('tb3_1', merger, comm_range)
  intercept3 = Intercept('tb3_2', merger, comm_range)
  
  sleep = rospy.Rate(5000)
  while not rospy.is_shutdown():
    #print("running...")
    sleep.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
