#!/usr/bin/env python2

import rospy
from balloon_detector import BalloonDetector
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import tf



import math

br = tf.TransformBroadcaster()


speed = 0

def odomCb(odom):
    global speed
    speed = odom.twist.twist.linear.x

class ImageHandler:
    def __init__(self, img_pub=False):
        if img_pub:
            self.image_pub_marked = rospy.Publisher("/assignment6/image_marked_ang", Image, queue_size=200, latch=True)
        else:
            self.image_pub_marked = None
        self.odom_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size=200)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, odomCb, queue_size=1)
        self.detector = BalloonDetector()

        pose_covar = PoseWithCovariance(Pose(Point(0, 0, 0), Quaternion()), None)
        self.odom = Odometry(Header(frame_id='map'), 'odom', pose_covar, None)

    def callback(self, data):
        t_start = rospy.Time.now()
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        xy = self.detector.calculate_best_position(img)

        if self.image_pub_marked is not None:
            self.detector.draw_markers(img)
            self.image_pub_marked.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        # Don't publish a pose if location can't be reliably determined
        if xy is None:
            print("No location found")
            return

        angle = self.detector.calculate_angle()

        header = self.odom.header
        header.seq = data.header.seq
        header.stamp = data.header.stamp

        pose = self.odom.pose.pose
        pos = pose.position
        pos.x, pos.y = xy

        quaternion = pose.orientation
        quaternion.z, quaternion.w = math.sin(angle / 2), math.cos(angle / 2)

        q = quaternion

        self.odom.twist.twist.linear.x = speed
        # self.odom.child_frame_id = "/base_link"
        self.odom_pub.publish(self.odom)

        # br.sendTransform( (pos.x, pos.y, 0),
        #               [q.x, q.y, q.z, q.w],
        #               rospy.Time.now(),
        #               "/base_link",
        #               "/odom")

        print('%-30s angle: %6.1f img_age: %5.3f calc: %5.3f' %
              (xy, np.rad2deg(angle), to_secs(t_start - data.header.stamp), to_secs(rospy.Time.now() - t_start)))


def to_secs(duration):
    """
    :type duration: rospy.Duration
    """
    return duration.secs + 1e-9 * duration.nsecs


def main():
    rospy.init_node('assignment6_angle_localisation')
    ImageHandler()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()
