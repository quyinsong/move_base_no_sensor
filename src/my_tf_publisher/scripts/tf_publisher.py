#!/usr/bin/env python3  
import rospy  
import tf  
import tf2_ros  
from geometry_msgs.msg import TransformStamped  
from nav_msgs.msg import Odometry 
from math import radians, cos, sin 
from geometry_msgs.msg import Quaternion, Point, Pose 

class PubTf:

    def __init__(self): 
        # 初始化ROS节点  
        rospy.init_node('tf_publisher', anonymous=True) 

        self.rate = rospy.Rate(20.0)  # 10hz

        self.br = tf2_ros.TransformBroadcaster()
        # 设置机器人初始位置
        self.init_x = 2
        self.init_y = 2
        self.init_theta = 0 
        # 发布机器人odom坐标
        self.odom = Odometry()  
        self.odom.header.stamp = rospy.Time.now()  
        self.odom.header.frame_id = "odom"  
        self.odom.child_frame_id = "base_link"
        # 设置位置和姿态  
        self.odom.pose.pose = Pose(Point(x=self.init_x, y=self.init_y, z=0.0),  
                            self.quaternion_from_euler(0, 0, radians(self.init_theta)))  

        # 设置速度和角速度  
        self.odom.twist.twist.linear.x = 0
        self.odom.twist.twist.angular.z = 0

    def callback(self,data): 
        self.odom.pose.pose = data.pose.pose 

    def quaternion_from_euler(self,roll, pitch, yaw):  
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)  
        return Quaternion(*q)  

    def publish_tf(self):  
        while not rospy.is_shutdown():  
            time_now = rospy.Time.now() 
            # 订阅'/cmd_vel'话题，并将回调函数设置为self.callback  
            rospy.Subscriber("/odom", Odometry, self.callback) 
            # 假设我们发布从'/world'到'/child'的变换  
            trans1 = TransformStamped()  
            trans1.header.stamp = time_now
            trans1.header.frame_id = "map"  
            trans1.child_frame_id = "odom"  
            trans1.transform.translation.x = 0.0  
            trans1.transform.translation.y = 0.0  
            trans1.transform.translation.z = 0.0  
            trans1.transform.rotation.x = 0.0  # 对应于四元数[0, 0, 0, 1]（即没有旋转） 
            trans1.transform.rotation.y = 0.0  # 对应于四元数[0, 0, 0, 1]（即没有旋转） 
            trans1.transform.rotation.z = 0.0  # 对应于四元数[0, 0, 0, 1]（即没有旋转） 
            trans1.transform.rotation.w = 1.0  # 对应于四元数[0, 0, 0, 1]（即没有旋转）  
            # 假设我们发布从'/world'到'/child'的变换  
            trans2 = TransformStamped()  
            trans2.header.stamp = time_now 
            trans2.header.frame_id = "odom"  
            trans2.child_frame_id = "base_link"  
            trans2.transform.translation.x = self.odom.pose.pose.position.x 
            trans2.transform.translation.y = self.odom.pose.pose.position.y  
            trans2.transform.translation.z = 0.0  
            trans2.transform.rotation.x = self.odom.pose.pose.orientation.x
            trans2.transform.rotation.y = self.odom.pose.pose.orientation.y
            trans2.transform.rotation.z = self.odom.pose.pose.orientation.z
            trans2.transform.rotation.w = self.odom.pose.pose.orientation.w

            self.br.sendTransform(trans1)  
            self.br.sendTransform(trans2)  

            self.rate.sleep()  
  
if __name__ == '__main__':  
    try:  
        pub_tf = PubTf()
        pub_tf.publish_tf()
    except rospy.ROSInterruptException:  
        pass