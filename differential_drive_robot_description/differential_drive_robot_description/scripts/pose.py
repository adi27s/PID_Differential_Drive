#!/usr/bin/env python3

import time
import queue
import rospy
from math import atan2,sqrt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Add more waypoints by uncommenting the below coordinates
x_goals = [3.99109,-8.70703, 7.6093, -22.159, -13.9462]          #, 9.84763, 3.99109, -12.8382, 1.36524, -16.1508]
y_goals = [-8.03451, 7.84897, 3.72627, 4.11626, 0.671191]       # , -5.33862, -8.03451, -7.41725, 6.50833, 7.66259]

class MyNode:
    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        self.sub = rospy.Subscriber('/odom',Odometry,self.callback)
        self.lidar = rospy.Subscriber('/rrbot/laser/scan', LaserScan, self.callbackLidar)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.val=LaserScan()
        self.vel=Twist()
        self.rate = rospy.Rate(10) # Call 10 times in one second
        self.index,self.poseX,self.poseY,self.theta,self.e_x,self.e_y=0,0.0,0.0,0.0,0.0,0.0

    def callbackLidar(self, data):
        self.val = data

    # Check for obstacle from -30 deg to +30 deg in front of the robot (Array data from onboard Lidar)
    def get_60_degree_range(self):
        priority_queue=queue.PriorityQueue()
        for i in range(329, 391):
            priority_queue.put(self.val.ranges[i])
        return priority_queue.get()
    
    # Position and orientation data
    def callback(self,data):
        self.poseX = data.pose.pose.position.x
        self.poseY = data.pose.pose.position.y
        x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        _, _, self.theta = euler_from_quaternion([x, y, z, w])
        self.e_x = x_goals[self.index]-self.poseX
        self.e_y = y_goals[self.index]-self.poseY
        
    # Calculation of Euclidean Distance from current position to the destination coordinates
    def distance(self):
        return sqrt(pow(self.e_x, 2)+pow(self.e_y, 2))

    # Linear Speed constrain
    def constraint_linear_speed(self,speed):
        if speed>1.0:
            return 1.0
        elif speed<0.0:
            return 0.0
        else:
            return speed
        
    # Angular Speed constrain
    def constraint_angular_speed(self,speed):
        if speed > 0.3:
            return 0.3
        elif speed < -0.3:
            return -0.3
        else:
            return speed

    # Removes the ambiguity of turning ACW/CW while traversing    
    def angle_calculation_on_the_run(self,angle):
        if (abs(angle)<3.14):
            return angle
        elif angle>3.14:
            return angle-6.28
        elif angle<3.14:
            return 6.28-angle
    
    # MAIN FUNCTION
    def move(self):
        global x_goals,y_goals
        kp_l,kp_r=0.1,0.8
        kd_r=0.7
        ki_r=0.01
        while not rospy.is_shutdown():
            #--------------------------angular motion---------------------------------
            self.rate.sleep()
            theta = atan2(self.e_y,self.e_x)
            while abs(theta-self.theta) > 0.05 and not rospy.is_shutdown():
                self.vel.angular.z=kp_r*(theta-self.theta)+0.0
                self.pub.publish(self.vel)
                self.rate.sleep()
            self.vel.angular.z=0
            self.pub.publish(self.vel)
            print("Angle Moved")
            time.sleep(2)

            #--------------------------linear motion and path correction--------------------------------
            theta_error_PID = [theta-self.theta]
            while (self.distance()) > 0.3 and (self.get_60_degree_range() > 0.8) and not rospy.is_shutdown():
                self.vel.linear.x = self.constraint_linear_speed(kp_l*self.distance())
                current_theta_error = self.angle_calculation_on_the_run(atan2(self.e_y, self.e_x)-self.theta)
                self.vel.angular.z = self.constraint_angular_speed(kp_r*(current_theta_error)+ki_r*(sum(theta_error_PID))+kd_r*(theta_error_PID[-1]))
                theta_error_PID.append(current_theta_error)
                self.pub.publish(self.vel)
                self.rate.sleep()

            self.vel.linear.x=0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
            print(f"Destination {self.index+1} Reached Successfully")
            self.index += 1
            if (self.index==len(x_goals)):
                print("All goals reached")
                break

if __name__=='__main__':
    try:
        node = MyNode()
        time.sleep(2) # Initialization Time
        node.move()
    except rospy.ROSInterruptException:
        pass