#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import PoseStamped,Quaternion,Twist
from geometry_msgs.msg import TransformStamped  
from tf2_ros import TransformBroadcaster    
from nav_msgs.msg import Odometry  
import tf.transformations  
import numpy as np
import math


class CmdVelListener:  

    def __init__(self):  

        rospy.init_node('cmd_vel_listener', anonymous=True)  

        self.initial_x = 10  # 初始化变量  
        self.initial_y = 10 # 初始化变量 
        self.initial_theta = 0  # 初始化变量 
        self.simtime = 0.2
        self.lasttime = rospy.Time.now() 
        self.lasttime = self.lasttime.secs + self.lasttime.nsecs * 1e-9
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.y = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)  
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.broadcaster = TransformBroadcaster() 

    def cmd_vel_callback(self, msg):  
        rospy.loginfo(rospy.get_caller_id() + "I heard %f m/s linear velocity and %f rad/s angular velocity!",  
                    msg.linear.x, msg.angular.z) 
        self.twist = msg   

    def set_point_pose(self):  
        rate = rospy.Rate(10)  # 设置发布频率为1Hz   
        while not rospy.is_shutdown():  
            timenow = rospy.Time.now() 
            # 发布base_footprint到odom的坐标系
            transform1 = TransformStamped() 
            transform1.header.stamp = timenow
            transform1.header.frame_id = "map"  # 父坐标系ID 
            transform1.child_frame_id = "odom"   # 子坐标系ID  
            transform1.transform.translation.x = 0.0  
            transform1.transform.translation.y = 0.0  
            transform1.transform.translation.z = 0.0 
            transform1.transform.rotation.w = 1.0  
            transform1.transform.rotation.x = 0.0  
            transform1.transform.rotation.y = 0.0  
            transform1.transform.rotation.z = 0.0 

            # # 获取机器人的位置和姿态
            # mujoco.mj_step(m1,data1)
            
            # robot_pos = data1.body('waist').xpos
            # robot_quat = data1.body('waist').xquat         

            # 发布dom到base_footprint的坐标系
            transform2 = TransformStamped()
            transform2.header.stamp = rospy.Time.now() 
            transform2.header.frame_id = "odom"  # 父坐标系ID 
            transform2.child_frame_id = "base_footprint"   # 子坐标系ID

            timenow1 = timenow.secs + timenow.nsecs * 1e-9
            self.simtime = timenow1 - self.lasttime
            self.lasttime = timenow1
            
            self.initial_x = self.initial_x + self.simtime*self.twist.linear.x*np.cos(self.initial_theta)
            self.initial_y = self.initial_y + self.simtime*self.twist.linear.x*np.sin(self.initial_theta)

            transform2.transform.translation.z = 0.0 
            transform2.transform.translation.x = self.initial_x
            transform2.transform.translation.y = self.initial_y
            # 设置旋转（这里使用单位四元数，表示没有旋转）
            self.initial_theta = self.initial_theta+self.simtime*self.twist.angular.z
            quaternion = tf.transformations .quaternion_from_euler(0, 0, self.initial_theta)   
            transform2.transform.rotation.w = quaternion[3] 
            transform2.transform.rotation.x = quaternion[0]  
            transform2.transform.rotation.y = quaternion[1]  
            transform2.transform.rotation.z = quaternion[2]

            self.broadcaster.sendTransform(transform1)
            self.broadcaster.sendTransform(transform2)

            # 创建一个Odometry消息  
            odom = Odometry()  
            odom.header.stamp = timenow
            odom.header.frame_id = "odom"  
            # odom.child_frame_id = "base_footprint"  
            # 设置位置  
            odom.pose.pose.position.x = self.initial_x 
            odom.pose.pose.position.y = self.initial_y
            odom.pose.pose.position.z = 0.0 
            # 设置姿态（例如，没有旋转）  
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]
            # 填充速度信息（如果需要）  
            odom.twist.twist.linear.x = self.twist.linear.x  
            odom.twist.twist.linear.y = 0.0  
            odom.twist.twist.angular.z = self.twist.angular.z  

            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.y = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0
            # 发布里程计信息  
            self.odom_pub.publish(odom)

            rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)

            
            rate.sleep()  # 等待下一个发布周期 

    def spin(self):  

        rospy.spin()  

  

if __name__ == '__main__':  

    listener = CmdVelListener()  

    listener.set_point_pose()