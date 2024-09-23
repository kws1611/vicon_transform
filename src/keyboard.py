#!/usr/bin/env python3.8

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from time import sleep
import readchar

class LF_Sim:
    def __init__(self):
        self.goal_point_pub = rospy.Publisher('/LF/goal_point', PoseStamped, queue_size=10)

        self.start_pos = Point(0,0,10)
        self.cur_pos = Point(0,0,10)
        self.img = np.zeros((400, 400, 3), dtype = np.uint8)
        self.origin = (200,200)
        self.radius = 15
        self.color = (255,0,0)
        self.thickness = -1
        self.cur_yaw = 0

    def pose_publish(self, Point, angle):
        temp_msg = PoseStamped()
        temp_msg.pose.position = Point
        q_w = np.cos(angle/2)
        q_z = np.sin(angle/2)
        temp_msg.pose.orientation.w = q_w
        temp_msg.pose.orientation.x = 0
        temp_msg.pose.orientation.y = 0
        temp_msg.pose.orientation.z = q_z
        self.goal_point_pub.publish(temp_msg)

    def process(self):
        self.img = np.zeros((400, 400, 3), dtype = np.uint8)
        key = readchar.readkey()
        key = key.lower()
        if key in('wsadqe'):
            if 'w' == key:
                self.cur_pos.y += 1

            elif 's' == key:
                self.cur_pos.y -= 1

            elif 'a' == key:
                self.cur_pos.x -= 1

            elif 'd' == key:
                self.cur_pos.x += 1

            elif 'q' == key:
                self.cur_yaw += np.pi/2
                print("check", self.cur_yaw)

            elif 'e' == key:
                self.cur_yaw -= np.pi/2
            
            else:
                pass

        else:
            pass
        #self.goal_point_pub.publish(self.cur_pos)
        self.pose_publish(self.cur_pos, self.cur_yaw)
        self.pose_publish(self.cur_pos, self.cur_yaw)
        self.pose_publish(self.cur_pos, self.cur_yaw)
        self.pose_publish(self.cur_pos, self.cur_yaw)
        #rospy.loginfo_throttle(1, 'LF Sim position <%.1f, %.1f, %.1f>'%(self.cur_pos.x,self.cur_pos.y,self.cur_pos.z))
        coordinate = (self.origin[0]+ 10*self.cur_pos.x, self.origin[1]- 10*self.cur_pos.y)
        aero_temp = (np.cos(-self.cur_yaw)*10, np.sin(-self.cur_yaw)*10)
        print(aero_temp)
        start_point = (int(coordinate[0] - aero_temp[0]) , int(coordinate[1] - aero_temp[1]))
        end_point = (int(coordinate[0] + aero_temp[0]) , int(coordinate[1] + aero_temp[1]))
        print('LF Sim position <%.1f, %.1f, %.1f>'%(self.cur_pos.x,self.cur_pos.y,self.cur_pos.z))
        #image = cv2.circle(self.img, coordinate, self.radius, self.color, self.thickness) 
        image = cv2.arrowedLine(self.img, start_point, end_point, (0,0,255), 5, tipLength = 0.5)
        cv2.imshow('black image', image)
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('main', anonymous=True, disable_signals=True)
    
    mission = LF_Sim()

    rate = rospy.Rate(100.0)

    try:
        while not rospy.is_shutdown():
            mission.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
