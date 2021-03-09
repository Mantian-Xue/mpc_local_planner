#!/usr/bin/env python
import rospy
from mpc_local_planner_msgs.msg import StateFeedback
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf2_ros
import tf_conversions


class FakeBase():
    '''
        Simulated ackermann base
    '''
    def __init__(self):
        rospy.Subscriber("/nav_cmd_vel",Twist, self.cmd_vel_callback)
        self.pub = rospy.Publisher("/move_base/MpcLocalPlannerROS/state_feedback", StateFeedback,queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.odom_pub = rospy.Publisher("/odom",Odometry,queue_size=1)  
    
        # params
        self.lf = 0.4
        self.lr = 0.4 
        self.tau = 0.7

        # control
        # linear_x, steering_angle 
        self.u = np.array([0.0,0.0])

        # system states
        # x,y,theta,steering
        self.x = np.array([0.0,0.0,0.0,0.0])
        self.dx = np.array([0.0,0.0,0.0,0.0])

        # time
        self.last_time = rospy.Time.now()
    
    def publish_states(self):
        current_time = rospy.Time.now()
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        q = tf_conversions.transformations.quaternion_from_euler(0,0,self.x[2])

        br_static = tf2_ros.StaticTransformBroadcaster()
        t_static = TransformStamped()
        t_static.header.stamp = current_time
        t_static.header.frame_id = "map"
        t_static.child_frame_id = "odom"
        t_static.transform.translation.x = 2.0
        t_static.transform.translation.y = 2.0
        t_static.transform.translation.z = 0.0
        t_static.transform.rotation.x = 0.0
        t_static.transform.rotation.y = 0.0
        t_static.transform.rotation.z = 0.0
        t_static.transform.rotation.w = 1.0
        br_static.sendTransform(t_static)

        # odom
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x[0]
        odom_msg.pose.pose.position.y = self.x[1]
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom_msg)

        # tf
        t.transform.translation.x = self.x[0]
        t.transform.translation.y = self.x[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        
        # mpc states
        msg = StateFeedback()
        msg.header.stamp = current_time
        msg.state = list(self.x)
        self.pub.publish(msg)
        print("states: {}".format(self.x))

    def update(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.dynamics()
        self.x = self.x + self.dx * dt
        self.last_time = current_time
        self.publish_states()

    def dynamics(self):
        beta = math.atan(self.lr/(self.lr + self.lf) * math.tan(self.x[3]))
        self.dx[0] = self.u[0] * math.cos(self.x[2] + beta)
        self.dx[1] = self.u[0] * math.sin(self.x[2] + beta)
        self.dx[2] = self.u[0] * math.sin(beta) / self.lr
        self.dx[3] = - (self.x[3] - self.u[1]) / self.tau
        
    def cmd_vel_callback(self,cmd_vel):
        self.u[0] = cmd_vel.linear.x
        self.u[1] = cmd_vel.angular.z
        print("cmd: {}".format(self.u))
    
if __name__ == "__main__":
    rospy.init_node("base_feedback_node")
    fakebase = FakeBase()
    loop_rate = rospy.timer.Rate(100) 
    while not rospy.is_shutdown():
        fakebase.update()
        loop_rate.sleep()
        
