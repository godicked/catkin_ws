#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int32
#from ackermann_msgs.msg import AckermannDriveStamped


def vel_to_rpm(vel):
  return (vel * 60) / ( 2 * 3.14 * 5.5) # speed * 60 sec / 2*pi*radius

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global twist_cmd_topic
  global speed_cmd_topic
  global frame_id
  global speed_pub
  global steer_pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  speed = Int32()
  speed.data = vel_to_rpm(v)
  speed_pub.publish(speed)
  rospy.loginfo("send speed %d", speed.data)

  rospy.loginfo("send steer %.2f", steering)
  steer = Int16()
  steer.data = steering 
  steer_pub.publish(steer)
  
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_manual_control')
  
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
    speed_cmd_topic = rospy.get_param('~speed_cmd_topic', '/manual_control/speed')
    steer_cmd_topic = rospy.get_param('steer_cmd_topic', 'manual_control/steering2')
    wheelbase = rospy.get_param('~wheelbase', 0.3)
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    #pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    speed_pub = rospy.Publisher(speed_cmd_topic, Int32, queue_size=1)
    steer_pub = rospy.Publisher(steer_cmd_topic, Int16, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_manual_control' started.\nListening to %s, publishing to %s and %s. Wheelbase: %f", twist_cmd_topic, speed_cmd_topic, steer_cmd_topic, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
