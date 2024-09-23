#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3,Point, Quaternion
import csv

class Transform:
    def __init__(self):
        # publisher
        self.vicon_pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)
        # Subscriber
        rospy.Subscriber('/vicon/uvify/uvify', TransformStamped, self.vicon_callback)
        self.position = Point()
        self.orientation = Quaternion()

    def vicon_callback(self, msg):
        self.position.x = msg.transform.translation.x
        self.position.y = msg.transform.translation.y
        self.position.z = msg.transform.translation.z
        self.orientation = msg.transform.rotation
        temp_msg = PoseStamped()
        temp_msg.pose.position = self.position
        temp_msg.pose.orientation = self.orientation
        temp_msg.header.stamp = msg.header.stamp
        temp_msg.header.frame_id = "odom"
        self.vicon_pub.publish(temp_msg)

    def process(self):
        pass

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('pose_extract', anonymous=True, disable_signals=True)
    
    mission = Transform()

    rate = rospy.Rate(20.0)

    try:

        while not rospy.is_shutdown():
            mission.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass