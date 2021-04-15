#!/usr/bin/env python2
import rospy
import math
import time
from nav_msgs.msg import Odometry 
from tf import transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point
class control(self):
    def __init__(self):
#to define variable to use throughout the program
        self.start = Point()
        self.angle =0.0
        self.ctr =0
        self.distance_accuracy =1
        self.angle_accuracy = (math.pi)/180
        self.k_distance = 0.5
        self.k_angle = 0.5
        self.final_angle
        self.final_pos[3] =(20,20,0)
        self.twist_msg = Twist()
#to make a node, subscriber and publisher    
        rospy.init_node("p_controller",anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        subscriber = rospy.Subscriber('/odom',Odometry,self.callback_value)
        self.rate = rospy.Rate(100)
#to check how to move using counter variable
        while not rospy.is_shutdown():
            if self.ctr == 0:
                self.change_angle()
            elif self.ctr ==1:
                self.change_postion()
            elif self.ctr ==2:
                self.final()
            else:
                rospy.logerr("Error")
                pass 
            rate.sleep()
#to change the angle 
    def change_angle(self):
#to find the error and use the condition
        self.final_angle = math.atan(self.final_pos[1]-self.start.y/self.final_pos[0]-self.start.x)
        self.ange_error = self.final_angle-angle
        if math.fabs(self.ange_error)> self.angle_accuracy:
            self.twist_msg.angular.z = self.ange_error*(self.k_angle)
            self.pub.publish(twist_msg)
        else:
            self.ctr =1
# to change the position
    def change_postion(self):
#to find the error and use the condition
        self.distance_error = math.sqrt((math.pow(self.final_pos[0]-self.start.x,2))+(math.pow(self.final_pos[1]-self.start.y,2)))
        self.final_angle = math.atan(self.final_pos[1]-self.start.y/self.final_pos[0]-self.start.x)
        self.ange_error = self.final_angle-angle
        if(self.distance_error>self.distance_accuracy):
            self.twist_msg.linear.x = 0.5*self.distance_error
            self.pub.publish(self.twist_msg)
        else:
            self.ctr=2
        if math.fabs(self.ange_error)> self.angle_accuracy:
            self.twist_msg.angular.z = self.ange_error*(self.k_angle)
            self.pub.publish(self.twist_msg)
        else:
            self.ctr =1
#to stop when final position reached
    def final(self):
        self.twist_msg.linear.x=0
        self.twist_msg.angular.y=0
        self.pub.publish(self.twist_msg)
#to get the current position
    def callback_value(self):
        self.start = msg.pose.pose.position
        self.quat =(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        self.euler = transformations.euler_from_quaternion(quat)
        self.angle = euler[2]
if __name__ == '__main__':
        control()
        rospy.spin()
   
