#!/usr/bin/env python3.8

import rospy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool, Int32, Float32
from math import sin, cos, sqrt, pi
import numpy as np

class Mission:
    def __init__(self):
        self.current_state = State()
        self.global_home_position = GeoPoint()
        self.current_local_position = Point()
        
        self.earth_radius = None
        self.home_set = False
        self.goal_point = Point(0,0,10)
        self.goal_orientation = Quaternion(0,0,0,1)

        self.local_home_position = Point(0, 0, 0)
        
        # Waypoints and Obstacle point
        #self.local_home_position = None
        self.wpTakeoff = None

        self.wp1 = None
        self.wp2 = None
        self.wp3 = None

        self.obstacle = None

        self.mission_start = False
        # Publisher
        self.local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

        # Subscriber
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.LocalPositionCb)
        rospy.Subscriber('/LF/goal_point', PoseStamped, self.LfsimCb)

        # Service_client
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def LfsimCb(self, msg):
        self.goal_point = msg.pose.position
        # self.goal_point.y = msg.y
        # self.goal_point.z = msg.z
        self.goal_orientation = msg.pose.orientation

    
    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")

        rospy.loginfo("FCU connected")

    def check_home_setting(self):
        while not self.home_set:
            rospy.loginfo_throttle(1, "Wait Home position Setting")
        rospy.loginfo("Home position is setted")


    def setMode(self, mode):
        # Custom Mode List: 
        # http://wiki.ros.org/mavros/CustomModes

        rate = rospy.Rate(0.5)
        while True:
            self.PubLocalPosition(self.local_home_position)

            if self.current_state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
            
            else:
                break

            rate.sleep()
                
    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)

            else:
                break

            rate.sleep()

    def setWayPoints(self):
        self.local_home_position = Point(0, 0, 0)

        self.wpTakeoff = Point(0, 0, rospy.get_param('/takeoff_alt', 30.0))

        self.wp1 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp1'))
        self.wp2 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp2'))
        self.wp3 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp3'))
        
        rospy.loginfo('Set Waypoint')


    def stateCb(self, msg):
        prev_state = self.current_state

        self.current_state = msg
        
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)

        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
        
    def homeCb(self, msg):
        self.home_set = True

        self.global_home_position.latitude = msg.geo.latitude
        self.global_home_position.longitude = msg.geo.longitude

    def LocalPositionCb(self, msg):
        self.current_local_position.x = msg.pose.position.x
        self.current_local_position.y = msg.pose.position.y
        self.current_local_position.z = msg.pose.position.z

    def PubLocalPosition(self, point):
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z

        self.local_pose_pub.publish(pose)
    
    def PubLocalPose(self, point, orientation):
        temp_pose = PoseStamped()

        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.pose.position= point
        temp_pose.pose.orientation = orientation

        self.local_pose_pub.publish(temp_pose)

    def PublishVelocity(self, vel):
        self.velocity_pub.publish(vel)

    def ConvertLocalPoint(self, global_point):
        local_point = Point()

        home_lat = self.global_home_position.latitude
        home_lon = self.global_home_position.longitude
        
        local_point.x = self.earth_radius * cos(home_lat * pi / 180) * (global_point[1] - home_lon) * pi / 180
        local_point.y = self.earth_radius * (global_point[0] - home_lat) * pi / 180
        local_point.z = global_point[2]

        return local_point

    def waypoint_reach_check(self, wp, reach_range):

        x = self.current_local_position.x
        y = self.current_local_position.y
        z = self.current_local_position.z

        d = sqrt((wp[1].x - x)**2 + (wp[1].y - y)**2 + (wp[1].z - z)**2)

        if d < reach_range :
            return True
        else:
            rospy.loginfo_throttle(1, '%s Process <Distance: %.1f>'%(wp[0], d))
            return False

    def calculate_velocity(self, wp):
        velocity = Twist()

        xy_position = np.array((self.current_local_position.x, self.current_local_position.y))
        xy_target = np.array((wp.x, wp.y))

        z_position = self.current_local_position.z
        z_target = wp.z

        # XY control
        obstacle = np.array((self.obstacle.x, self.obstacle.y))
        R  = self.obstacle.z
        d1, d2 = (10.0, R + 20.0)
        XY_max_vel = 5.0
        
        k1 = XY_max_vel / d1
        k2 = 2*XY_max_vel * (R**2) * R*d2/(d2 - R)
        
        distance_goal = np.linalg.norm((xy_target - xy_position))
        distance_obstacle = np.linalg.norm((obstacle - xy_position))

        # XY control, Attractive term
        if distance_goal <= d1:
            gradient = k1 * (xy_position - xy_target)
        else:
            gradient = k1*d1/distance_goal * (xy_position - xy_target)

        # XY control, Repulsive term
        if distance_obstacle <= d2:
            gradient += k2*(1/d2 - 1/distance_obstacle)/(distance_obstacle**3)*(xy_position - obstacle)

        velocity.linear.x = -gradient[0]
        velocity.linear.y = -gradient[1]

        # Z control
        k3 = 2

        velocity.linear.z = k3 * (z_target - z_position)

        return velocity

    def process(self):
        self.PubLocalPose(self.goal_point, self.goal_orientation)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('main', anonymous=True, disable_signals=True)
    
    flight = Mission()

    rate = rospy.Rate(20.0)

    try:
        flight.check_FCU_connection()
        rospy.sleep(1)

        flight.check_home_setting()
        #flight.setEarthRadius() 
        rospy.sleep(1)
        
        #flight.setWayPoints()
        #flight.setObstaclePoints()
        rospy.sleep(1)
        
        flight.setMode("OFFBOARD")
        # rospy.sleep(1)

        flight.setArm()

        while not rospy.is_shutdown():
            flight.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass