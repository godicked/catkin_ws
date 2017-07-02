#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import Path
import matplotlib.pyplot as plt


def path_callback(path):
    posx = []
    posy = []

    for pose in path.poses:
        posx.append(pose.pose.position.x * 10)
        posy.append(pose.pose.position.y * 10)
    
    plt.axis([-100,100, -100, 100])
    plt.plot(posx, posy, 'bo')
    plt.show()


if __name__ == '__main__': 
  try:
    
    rospy.init_node('curvature_plot')
    path_topic = '/smooth_path'

    rospy.Subscriber(path_topic, Path, path_callback, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
