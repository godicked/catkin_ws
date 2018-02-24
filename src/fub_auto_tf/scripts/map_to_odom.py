#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
import tf
import math


# last gps position / prediction
last_gps = None
# last odom
last_odom = None
#	x, y, yaw
offset = [0, 0, 0]

init = False

br = tf.TransformBroadcaster()

#	-pi to pi
def normalizeAngle(angle):
	a = angle
	while(a > math.pi):
		a -= math.pi * 2
	while(a < -math.pi):
		a += math.pi * 2
	return a

#	Offset between two position
def computeOffset(gps, odom):
	#	offset on X and Y axis

	print 'time o', odom.header.stamp.secs
	print 'time g', gps.header.stamp.secs


	x = gps.pose.pose.position.x - odom.pose.pose.position.x
	y = gps.pose.pose.position.y - odom.pose.pose.position.y

	q1 = gps.pose.pose.orientation
	q2 = odom.pose.pose.orientation

	#	roll, pitch, yaw
	(r, p, gpsYaw) = tf.transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
	(r, p, odomYaw) = tf.transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])

	print 'Yaw:', math.degrees(gpsYaw), '(GPS)'
	print 'Yaw:', math.degrees(odomYaw), '(ODOM)'

	#	yaw offset
	yaw = normalizeAngle(odomYaw - gpsYaw)

	print 'Diff:', math.degrees(yaw)

	return [x, y, yaw]
	# return [0, 0, 0]


#	Update offset and publish Transform from map to odom.
def publishTransform():
	global last_gps, last_odom, offset, init, yaw

	if last_gps != None and last_odom != None:
		offset = computeOffset(last_gps, last_odom)
		# print 'offset', offset
		# if init == False:
		# 	yaw = offset[2]
		# 	init = True
		# offset[2] = yaw
	
	br.sendTransform( (offset[0], offset[1], 0),
		tf.transformations.quaternion_from_euler(0, 0, 0, axes='rzyx'),
		last_odom.header.stamp,
		"odom",
		"podom")

	br.sendTransform( (0, 0, 0),
		tf.transformations.quaternion_from_euler(-offset[2], 0, 0, axes='rzyx'),
		last_odom.header.stamp,
		"podom",
		"map")



def gpsCallback(odom):
	global last_gps
	last_gps = odom
	publishTransform()

def odomCallback(odom):
	global last_odom
	last_odom = odom
	publishTransform()



def main(args):
	rospy.init_node('map_to_odom_tf', anonymous=True)
	sub = rospy.Subscriber('/visual_gps/odom', Odometry, gpsCallback, queue_size=1)
	odom_sub = rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=1)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)