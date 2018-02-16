#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
import tf


# last gps position / prediction
last_gps = None
# last odom
last_odom = None
#	x, y, yaw
offset = [0, 0, 0]

br = tf.TransformBroadcaster()

#	-pi to pi
def normalizeAngle(angle):
	a = angle
	if(a > math.pi):
		a -= math.pi * 2
	if(a < -math.pi):
		a += math.pi * 2
	return a

#	Offset between two position
def computeOffset(gps, odom):
	#	offset on X and Y axis
	x = odom.pose.pose.position.x - position.pose.pose.position.x
	y = odom.pose.pose.position.y - position.pose.pose.position.y

	q1 = gps.pose.pose.orientation
	q2 = odom.pose.pose.orientation

	#	roll, pitch, yaw
	(r, p, y1) = tf.transformations.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
	(r, p, y2) = tf.transformations.euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])

	#	yaw offset
	yaw = normalizeAngle(y2 - y1)

	return [x, y, yaw]

#	Update offset and publish Transform from map to odom.
def publishTransform():
	global last_gps, last_odom, offset

	if last_gps != None and last_odom != None:
		offset = computeOffset(last_gps, last_odom)
	
	br.sendTransform( (offset[0], offset[1], 0),
		tf.transformations.quaternion_from_euler(math.pi, 0, offset[2]),
		data.header.stamp,
		"odom",
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
    rospy.init_node('kalman_filter', anonymous=True)

	gps_sub = rospy.Subscriber('/visual_gps', Odometry, gpsCallback, queue_size=10)
	odom_sub = rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)