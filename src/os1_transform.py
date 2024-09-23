#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

def os1_Cb(msg):
    temp = msg
    temp.header.frame_id = "os1_sensor"
    print(1)
    pub.publish(temp)


if __name__ == "__main__":
    rospy.init_node("publisher_node")
    sub = rospy.Subscriber('/penguin/os1_cloud_node/points', PointCloud2, os1_Cb)
    pub = rospy.Publisher("/new/os1_cloud_node/points", PointCloud2, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()