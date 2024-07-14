#!/usr/bin/env python3  
import rospy  
from nav_msgs.msg import Odometry  
import tf  
from geometry_msgs.msg import Quaternion, Point, Pose  
from math import cos, sin  
from geometry_msgs.msg import Twist  

class PubOdom:

    def __init__(self): 
        # 初始化ROS节点  
        rospy.init_node('odom_publisher', anonymous=True) 

        # 创建一个Twist消息  
        self.twist = Twist()  
        self.twist.linear.x = 0 # 设置线速度  
        self.twist.angular.z = 0 # 设置角速度

        self.ts = 0.05
        self.last_time = rospy.Time.now()
        self.last_time = self.last_time.secs + self.last_time.nsecs * 1e-9
        self.rate = rospy.Rate(20) # 20hz 
        # 设置机器人初始位置
        self.init_x = 2
        self.init_y = 2
        self.init_theta = 0 

        self.pub = rospy.Publisher('/odom', Odometry, queue_size=2) 

    def callback(self,data): 
        rospy.loginfo(rospy.get_caller_id() + "I heard %f %f", data.linear.x, data.angular.z)  
        self.twist.linear.x = data.linear.x
        self.twist.angular.z = data.angular.z
    
    def listener():  
        # 初始化ROS节点  
        rospy.init_node('cmd_subscriber', anonymous=True)  

    def quaternion_from_euler(self,roll, pitch, yaw):  
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)  
        return Quaternion(*q)  

    def publish_odom(self):  
        rospy.init_node('odom_publisher', anonymous=True)   
        while not rospy.is_shutdown(): 
            time_now = rospy.Time.now()
            current_time = time_now.secs + time_now.nsecs * 1e-9
            # 订阅'/cmd_vel'话题，并将回调函数设置为self.callback  
            rospy.Subscriber("/cmd_vel", Twist, self.callback) 
            # 更新位置和姿态
            self.ts = current_time-self.last_time
            self.last_time = current_time
            self.init_x =  self.init_x+self.ts*self.twist.linear.x*cos(self.init_theta)
            self.init_y =  self.init_y+self.ts*self.twist.linear.x*sin(self.init_theta)
            self.init_theta =  self.init_theta + self.ts*self.twist.angular.z
            # 发布机器人odom坐标
            odom = Odometry()  
            odom.header.stamp = time_now 
            odom.header.frame_id = "odom"  
            odom.child_frame_id = "base_link"
              
            # 设置位置和姿态  
            odom.pose.pose = Pose(Point(x=self.init_x, y=self.init_y, z=0.0),  
                                self.quaternion_from_euler(0, 0, self.init_theta))  

            # 设置速度和角速度  
            odom.twist.twist.linear.x = self.twist.linear.x 
            odom.twist.twist.angular.z = self.twist.angular.z
            self.pub.publish(odom)  

            # 重置
            self.twist.linear.x = 0 # 设置线速度  
            self.twist.angular.z = 0 # 设置角速度

            self.rate.sleep()  

if __name__ == '__main__':  
    try:  
        pubodom = PubOdom()
        pubodom.publish_odom()
    except rospy.ROSInterruptException:  
        pass