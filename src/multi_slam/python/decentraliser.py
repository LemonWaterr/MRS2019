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

class GetDistance(object):
  def __init__(self, source, target):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self.pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    self.source = source
    self.target = target
    self.dist = 99999.0

  def callback(self, msg):
    if self.source == self.target:
      self.dist = 0.0
    else:
      idx = [i for i, n in enumerate(msg.name) if n == self.source or n == self.target]
      if len(idx) != 2:
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

class InitMergeMapInput(object):
  def __init__(self, target, merger):
    self.target = target
    self.merger = merger
    self.mergeOut = rospy.Publisher('{}/map4{}'.format(target, merger), OccupancyGrid, queue_size=10)
    self.localMap = rospy.Subscriber('{}/map'.format(merger), OccupancyGrid, self.callback)
    self.done = False
    
  def callback(self, msg):
    if self.merger == self.target:
      self.mergeOut.publish(msg)
    else:
      if not self.done:
        EmptyDataMsg = msg
        EmptyDataMsg.data = [-1] * len(EmptyDataMsg.data)
        self.mergeOut.publish(EmptyDataMsg)
        print('init {}/map4{}'.format(self.target, self.merger))
        self.done = True
    
class Decentralise(object):  
  def __init__(self, target, merger, comm_range):
    self.target = target
    self.merger = merger
    self.comm_range = comm_range
    
    self.first = True
    
    self.distance_getter = GetDistance(self.merger, self.target)
    self.sub = None
    self.pub = rospy.Publisher('{}/map4{}'.format(target, merger), OccupancyGrid, queue_size=10)
    
    if target == merger:
      self.sub = rospy.Subscriber('{}/map'.format(merger), OccupancyGrid, self.callback_myself)
    else:
      self.sub = rospy.Subscriber('map_merged_{}'.format(target), OccupancyGrid, self.callback_target)
    
    
    
  def callback_myself(self, msg):
    self.pub.publish(msg)

  def callback_target(self, msg):
    distance = self.distance_getter.distance

    if distance < self.comm_range:
      print("{} gets {}'s map since distance {} < {}".format(self.merger, self.target, distance, self.comm_range))
      self.pub.publish(msg)
    else:
      print("{}/{} {} > {}".format(self.merger, self.target, distance, self.comm_range))
    
    
def run():
  rospy.init_node('decentraliser')
  robot1 = 'tb3_0'
  robot2 = 'tb3_1'
  robot3 = 'tb3_2'
  comm_range = 5.0
  
  #Init puts empty map with correct dimensions to each proxy (for decentralisation) topic. MUST BE RUN BEFORE Decentralise() 
  init1_1 = InitMergeMapInput(robot1, robot1)
  init1_2 = InitMergeMapInput(robot2, robot1)
  init1_3 = InitMergeMapInput(robot3, robot1)
  init2_1 = InitMergeMapInput(robot1, robot2)
  init2_2 = InitMergeMapInput(robot2, robot2)
  init2_3 = InitMergeMapInput(robot3, robot2)
  init3_1 = InitMergeMapInput(robot1, robot3)
  init3_2 = InitMergeMapInput(robot2, robot3)
  init3_3 = InitMergeMapInput(robot3, robot3)
  
  
  decentre1_1 = Decentralise(robot1, robot1, comm_range)
  decentre1_2 = Decentralise(robot2, robot1, comm_range)
  decentre1_3 = Decentralise(robot3, robot1, comm_range)
  
  decentre2_1 = Decentralise(robot1, robot2, comm_range)
  decentre2_2 = Decentralise(robot2, robot2, comm_range)
  decentre2_3 = Decentralise(robot3, robot2, comm_range)
  
  decentre3_1 = Decentralise(robot1, robot3, comm_range)
  decentre3_2 = Decentralise(robot2, robot3, comm_range)
  decentre3_3 = Decentralise(robot3, robot3, comm_range)
  
  sleep = rospy.Rate(5000)
  while not rospy.is_shutdown():
    #print("running...")
    sleep.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
