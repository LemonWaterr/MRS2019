#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from threading import Thread

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
    self.dist = None

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
    return self.dist != None

  @property
  def distance(self):
    return self.dist
  
    
class Decentralise(Thread):  
  def __init__(self, merger, robots, comm_range, rate):
    Thread.__init__(self)
    self.merger = merger
    self.all_robot_names = robots
    self.comm_range = comm_range
    self.rate = rate
    
    self.local_subs = [None] * len(self.all_robot_names)
    self.merged_subs = [None] * len(self.all_robot_names)
    self.input_pubs = [None] * len(self.all_robot_names)
    self.distance_getter = [None] * len(self.all_robot_names)
    
    for i in range(0, len(self.all_robot_names)):
      self.distance_getter[i] = GetDistance(merger, self.all_robot_names[i])
      self.local_subs[i]  = rospy.Subscriber('{}/map'.format(self.all_robot_names[i]), OccupancyGrid, self.callback_local, [i])
      self.merged_subs[i] = rospy.Subscriber('map_merged_{}'.format(self.all_robot_names[i]), OccupancyGrid, self.callback_merged, [i])
      self.input_pubs[i]  = rospy.Publisher('{}/map4{}'.format(self.all_robot_names[i], self.merger), OccupancyGrid, queue_size=10)
      
    self.local_maps = [None] * len(self.all_robot_names)
    self.merged_maps = [None] * len(self.all_robot_names)
    self.can_use_merged = [False] * len(self.all_robot_names)
   
  def callback_local(self, msg, args):
    distance_getter = self.distance_getter[args[0]]
    if not distance_getter.ready:
      print('distance not ready so cannot receive local from {}'.format(self.all_robot_names[args[0]]))
    else:
      if distance_getter.distance < self.comm_range:
        self.local_maps[args[0]] = msg
      else:
        if self.local_maps[args[0]] == None:
          EmptyMsg = msg
          EmptyMsg.data = [-1] * len(EmptyMsg.data)
          self.local_maps[args[0]] = EmptyMsg
          print('empty {} map init'.format(self.all_robot_names[args[0]]))
    
  def callback_merged(self, msg, args):
    distance_getter = self.distance_getter[args[0]]
    if not distance_getter.ready:
      print('distance not ready so cannot receive merged from {}'.format(self.all_robot_names[args[0]]))
    else:
      if distance_getter.distance < self.comm_range:
        self.merged_maps[args[0]] = msg
        if msg.data.count(-1) == len(msg.data):
          self.can_use_merged[args[0]] = False
        else:
          self.can_use_merged[args[0]] = True
    
  def run(self):
    while not rospy.is_shutdown():
      if None not in self.local_maps:
        for i in range(0, len(self.all_robot_names)):
          if self.can_use_merged[i]:
            self.input_pubs[i].publish(self.merged_maps[i])
          else:
            self.input_pubs[i].publish(self.local_maps[i])
      else:
        print("Local map not received from some robot(s)")  
      self.rate.sleep()

      
def execute():
  rospy.init_node('decentraliser')
  robots = ['tb3_0', 'tb3_1', 'tb3_2']
  comm_range = 5.0
  rate = rospy.Rate(1.0)
  
  decentre0 = Decentralise(robots[0], robots, comm_range, rate)
  decentre1 = Decentralise(robots[1], robots, comm_range, rate)
  decentre2 = Decentralise(robots[2], robots, comm_range, rate)
  decentre0.setName('decentre0')
  decentre1.setName('decentre1')
  decentre2.setName('decentre2')
  decentre0.daemon = True
  decentre1.daemon = True
  decentre2.daemon = True
  
  decentre0.start()
  decentre1.start()
  decentre2.start()
  
  decentre0.join()
  decentre1.join()
  decentre2.join() 
    

if __name__ == '__main__':
  try:
    execute()
    while not rospy.is_shutdown():
      rospy.Rate(1.0).sleep()
    
  except rospy.ROSInterruptException:
    pass
