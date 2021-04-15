import rospy
import math
import time
from nav_msgs.msg import Odometry 
from tf import transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point
class control(self):
    start = Point()
    angle =0.0
    ctr =0
    distance_accuracy =1
    angle_accuracy = (math.pi)/180
    k_distance = 0.5
    k_angle = 0.5
    final_angle
    final_pos[3] =(20,20,0)
    twist_msg = Twist()
    def main(self):
        rospy.init_node("p_controller",anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.subscriber = rospy.Subscriber('/odom',Odometry,callback_value)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if ctr == 0:
                change_angle()
            elif ctr ==1:
                change_postion()
            elif ctr ==2:
                final()
            else:
                rospy.logerr("Error")
                pass 
            rate.sleep()
    def change_angle():
        final_angle = math.atan(final_pos[1]-start.y/final_pos[0]-start.x)
        ange_error = final_angle-angle
        if math.fabs(ange_error)> angle_accuracy:
            twist_msg.angular.z = ange_error*(k_angle)
            pub.publish(twist_msg)
        else:
            ctr =1
    def change_postion():
        distance_error = math.sqrt((math.pow(final_pos[0]-start.x,2))+(math.pow(final_pos[1]-start.y,2)))
        final_angle = math.atan(final_pos[1]-start.y/final_pos[0]-start.x)
        ange_error = final_angle-angle
        if(distance_error>distance_accuracy):
            twist_msg.linear.x = 0.5*distance_error
            pub.publish(twist_msg)
        else:
            ctr=2
        if math.fabs(ange_error)> angle_accuracy:
            twist_msg.angular.z = ange_error*(k_angle)
            pub.publish(twist_msg)
        else:
            ctr =1
    def final():
        twist_msg.linear.x=0
        twist_msg.angular.y=0
        pub.publish(twist_msg)
    def callback_value():
        start = msg.pose.pose.position
        quat =(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        euler = transformations.euler_from_quaternion(quat)
        angle = euler[2]
if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
