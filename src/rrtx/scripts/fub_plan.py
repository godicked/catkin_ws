#!/usr/bin/env python

import rospy, math
import numpy as np
from nav_msgs.msg import Path
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Twist

car_speed = 1.0 # m/s
car_acceleration = 1.0 # m/s2


def distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y

    return math.sqrt(dx*dx + dy*dy)

def convert_to_fub_trajectory(path, traj):

    dist = 0

    step_size = 4

    for i in range(0, len(path)-step_size, step_size):
        p1 = path[i]
        p2 = path[i+step_size]

        d = distance(p1.pose.position, p2.pose.position)
        dist += d

        point = TrajectoryPoint()
        point.pose = p2.pose
        point.velocity.linear.x = car_speed
        point.time_from_start = dist / car_speed

        traj.trajectory.append(point)

    return (traj, dist)


def path_callback(path):
    plan = Trajectory()
    plan.header.frame_id = "map"
    plan.child_frame_id = "map"

    (plan, dist) = convert_to_fub_trajectory(path.poses, plan)

    print len(plan.trajectory)
    print dist


if __name__ == '__main__': 
  try:
    
    rospy.init_node('fub_plan')
    path_topic = '/rrtx_node/smooth_path'
    rospy.Subscriber(path_topic, Path, path_callback, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass