#!/usr/bin/env python

import rospy, math
import numpy as np
from nav_msgs.msg import Path
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Twist
import tf

car_speed = 1.0 # m/s
car_acceleration = 1.0 # m/s2

path_pub = None

listener = tf.TransformListener()

def transformPoses(poses):
    tf_poses = []
    listener.waitForTransform("map", "odom", rospy.Time.now(), rospy.Duration(1.0))

    for p in poses:
        tf_poses.append(listener.transformPose("odom", p))
    
    return tf_poses

def distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y

    # print 'p1 x:', p1.x, 'y:', p1.y
    # print 'p2 x:', p2.x, 'y:', p2.y
    # print '----------------'

    return math.sqrt(dx*dx + dy*dy)

def convert_to_fub_trajectory(path, traj):

    dist = 0
    # first_point = TrajectoryPoint()
    # first_point.pose = path[0].pose
    # first_point.pose.position.x = 0
    # first_point.pose.position.y = 0
    # first_point.pose.position.z = 0
    # first_point.velocity.linear.x = car_speed
    # first_point.time_from_start = rospy.Duration(0.0)
    # first_point.acceleration.linear.x = 0.0
    # traj.trajectory.append(first_point)

    for i in range(0, len(path)-1):
        # print 'i', i
        point1 = path[i]
        point2 = path[i+1]

        # print 'p1 x:', point1.pose.position.x, 'y:', point1.pose.position.y
        # print 'p2 x:', point2.pose.position.x, 'y:', point2.pose.position.y
        # print '----------------'
        
        d = distance(point1.pose.position, point2.pose.position)

        if d == 0:
            continue

        dist += d
        # print 'd', d

        point = TrajectoryPoint()
        point.pose = point2.pose
        point.pose.position.z = 0

        point.velocity.linear.x = car_speed
        point.time_from_start = rospy.Duration(dist / car_speed)
        point.acceleration.linear.x = 0

        traj.trajectory.append(point)

    return (traj, dist)


def path_callback(path):
    plan = Trajectory()
    plan.header.frame_id = "/odom"
    plan.child_frame_id = "/base_link"
    plan.header.stamp = rospy.Time.now()

    path.poses = transformPoses(path.poses)
    (plan, dist) = convert_to_fub_trajectory(path.poses, plan)

    print 'send path'
    path_pub.publish(plan)


if __name__ == '__main__': 
  try:
    
    rospy.init_node('fub_plan')
    path_topic = '/rrtx_node/smooth_path'
    rospy.Subscriber(path_topic, Path, path_callback, queue_size=1)
    path_pub = rospy.Publisher('/model_car/trajectory', Trajectory, queue_size=10)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass