#!/usr/bin/env python
import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import math

def normalizeAngle(angle):
	while(angle > math.pi):
		angle -= math.pi * 2
	while(angle < -math.pi):
		angle += math.pi * 2
	return angle

def dtime(odom_last, odom_now):
	return (odom_now.header.stamp - odom_last.header.stamp).nsecs / 10.0**9

def getVelocity(odom_last, odom_now):

	# to seconds
	dt = dtime(odom_last, odom_now)

	p1 = odom_last.pose.pose.position
	p2 = odom_now.pose.pose.position

	t1 = odom_last.pose.pose.orientation
	t2 = odom_now.pose.pose.orientation

	(r, p, y1) = tf.transformations.euler_from_quaternion([t1.x, t1.y, t1.z, t1.w])
	(r, p, y2) = tf.transformations.euler_from_quaternion([t2.x, t2.y, t2.z, t2.w])

	dx = (p2.x - p1.x)
	dy = (p2.y - p1.y)

	dth = (y2 - y1)

	dth = normalizeAngle(dth)
	
	# while(dth > math.pi):
	# 	dth -= math.pi * 2
	# while(dth < -math.pi):
	# 	dth += math.pi * 2

	v = math.sqrt(dx*dx + dy*dy) / dt
	vth = dth / dt

	# print 'dt', dt

	# inverse angle because odom is reversed
	return (v, -vth)



class kalman_filter:
	def __init__(self):
		self.gps_pub = rospy.Publisher("/kalman/odom", Odometry, queue_size=100)
		self.bridge = CvBridge()
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
		self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gpsCallback, queue_size=1)

		self.state = None
		self.updateTimestamp = None

		# state and covariance
		self.X = None
		self.P = None

		self.init = False
		self.gps = None
		self.odom = None
		self.vels = None
		self.time = None
		self.pos = None

	def odomCallback(self, data):
		if self.X == None:
			self.odom = data
			return
		self.vels = getVelocity(self.odom, data)
		

		# processing noise
		ra = 0.1
		rth = 0.1
		
		dt = dtime(self.odom, data)

		# error scaled to delta_t
		Q = dt * np.matrix([
			[ra, 0.0, 0.0],
			[0.0, ra, 0.0],
			[0.0, 0.0, rth]])

		self.predict(Q)

		self.odom = data

		self.publish(self.X)

	def gpsCallback(self, data):
		self.gps = data
		if self.X == None:
			# print 'init'
			self.X = self.getState(data)
			self.P = np.zeros((3,3), dtype=float)
			self.updateTimestamp = rospy.Time.now()
			return
		# gps observation

		z = self.getState(data)
		# print 'z', z

		# processing noise
		ra = 0.1
		rth = 0.1

		# dt = (rospy.Time.now() - self.updateTimestamp).nsecs / 10.0**9

		# error scaled to delta_t
		R = np.matrix([
			[ra, 0.0, 0.0],
			[0.0, ra, 0.0],
			[0.0, 0.0, rth]])

		H = np.identity(3, dtype=float)

		self.updateState(z, R, H)

		self.publish(self.X)
	
	def getState(self, odom):
		pos = odom.pose.pose.position
		ori = odom.pose.pose.orientation

		q = [ori.x, ori.y, ori.z, ori.w]
		(r, p, yaw) = tf.transformations.euler_from_quaternion(q)

		x = pos.x
		y = pos.y
		th = normalizeAngle(yaw)
		return np.matrix([x, y, th]).T

	def updateState(self, z, R, H):
		# innovation
		y = z - H * self.X

		# covariance innovation
		S = R + H * self.P * H.T
		# print 'S', S
		# print 'inv(S)', H.T * np.linalg.inv(S)

		# optimal gain 
		K = self.P * H.T * np.linalg.inv(S)

		# Updated state estimate
		self.X = self.X + K * y

		# Update covariance
		self.P = self.P - K * H * self.P

		# set timestamp
		self.updateTimestamp = rospy.Time.now()

		# self.X[2,0] = normalizeAngle(self.X[2,0])

	def predict(self, Q):
		(v, vth) = self.vels
		t = (rospy.Time.now() - self.updateTimestamp).nsecs / 10.0**9

		x = self.X[0,0]
		y = self.X[1,0]
		th = self.X[2,0]

		x1 = x + v * math.cos(th) * t
		y1 = y + v * math.sin(th) * t
		th1 = normalizeAngle(th + vth * t)

		# while(th1 > math.pi):
		# 	th1 -= math.pi * 2
		# while(th1 < -math.pi):
		# 	th1 += math.pi * 2

		# print 'th1', math.degrees(th1), 'th', math.degrees(th), 'vth', math.degrees(vth* t) 

		F = np.matrix([
			[x1/x, 0, 0],
			[0, y1/y, 0],
			[0, 0, th1/th]
		])

		self.X = F * self.X
		self.P = F * self.P * F.T + Q

		# set timestamp
		self.updateTimestamp = rospy.Time.now()
		# self.X[2,0] = normalizeAngle(self.X[2,0])

	def publish(self, X):
		# sending odometry 
		# print'X', self.X 
		odom = Odometry()
		odom.header.frame_id  = 'map'
		odom.header.seq = self.odom.header.seq
		odom.pose.pose.position.x = self.X[0,0]
		odom.pose.pose.position.y = self.X[1,0]
		odom.twist.twist.linear.x = self.odom.twist.twist.linear.x

		odom.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, self.X[2,0] ) )

		self.gps_pub.publish( odom )


def main(args):
    rospy.init_node('kalman_filter', anonymous=True)
    ic = kalman_filter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
