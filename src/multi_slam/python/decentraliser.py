#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy
import random

# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
#from std_msgs.msg import String

class Intercept(object):
  
  def __init__(self, topic_in, topic_out):
    rospy.Subscriber(topic_in, OccupancyGrid, self.callback)
    self.pub = rospy.Publisher(topic_out, OccupancyGrid, queue_size=10)
    self.commEnabled = False
    
  def callback(self, msg):
    self.pub.publish(msg)
    
def run():
  rospy.init_node('decentraliser')
  intercept1 = Intercept('tb3_0/map_temp', 'tb3_0/map')
  
  sleep = rospy.Rate(5000)
  while not rospy.is_shutdown():
    print("running...")
    sleep.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
