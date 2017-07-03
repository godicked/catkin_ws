#!/usr/bin/env python

import rospy, math
import numpy as np
from nav_msgs.msg import Path
import matplotlib.pyplot as plt


def path_callback(path):
    posx = []
    posy = []
    pos = []

    for pose in path.poses:
        posx.append(pose.pose.position.x * 10)
        posy.append(pose.pose.position.y * 10)
        pos.append([pose.pose.position.x, pose.pose.position.y])
    a = np.array(pos)
    plt.scatter(a[:,0], a[:,1])
    plt.show()
    # plt.axis([-100,100, -100, 100])
    # plt.plot(posx, posy, 'bo')
    # plt.show()


if __name__ == '__main__': 
  try:
    
    rospy.init_node('curvature_plot')
    path_topic = '/smooth_path'

    rospy.Subscriber(path_topic, Path, path_callback, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
