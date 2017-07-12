#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np


pos = []

def odom_callback(odom):
    global pos
    position = odom.pose.pose.position
    pos.append([position.x, position.y])

    if len(pos) == 5000:
        a = np.array(pos)
        plt.scatter(a[:,0], a[:,1])
        plt.show()

if __name__ == '__main__': 
  try:
    
    rospy.init_node('odom_listener')
  
    odom_topic = 'odom'
    
    rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
