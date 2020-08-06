#!/usr/bin/env python
# coding=utf-8
# /gtec/toa/ranging  /imu/data


import rospy
import quaternion
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Vector3
from gtec_msgs.msg import Ranging
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from shfaf import shfaf

pub_filter = rospy.Publisher('filter_position', Odometry, queue_size=1)
pub_uwb = rospy.Publisher('uwb_position', Odometry, queue_size=1)
pub_heading = rospy.Publisher('filter_heading', Float32 , queue_size=1)
pub_heading_true = rospy.Publisher('true_heading', Float32 , queue_size=1)
pub_error = rospy.Publisher('filter_error', Float32 , queue_size=1)
pub_bias = rospy.Publisher('filter_bias', Vector3 , queue_size=1)
init = 1
count = 0

def imu_callback(data):

    if init == 0:

        w = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        a = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
        filter.prediction(a,w,t)
        x = filter.x_
        q = filter.q_
        # rospy.loginfo(filter.posOld)
        odom = Odometry()
        odom.pose.pose.position.x = x[0]
        odom.pose.pose.position.y = x[1]
        odom.pose.pose.position.z = x[2]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x[3]
        odom.twist.twist.linear.y = x[4]
        odom.twist.twist.linear.z = x[5]
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = 0
        pub_filter.publish(odom)

        bias = Vector3()
        bias.x = filter.x_[6,0]
        bias.y = filter.x_[7,0]
        bias.z = filter.x_[8,0]
        pub_bias.publish(bias)

        heading = quaternion.as_euler_angles(quaternion.from_float_array(q))
        pub_heading.publish(heading[2])

        x_uwb = filter.posOld
        vel_uwb = filter.velOld
        odom_uwb = Odometry()
        odom_uwb.pose.pose.position.x = x_uwb[0]
        odom_uwb.pose.pose.position.y = x_uwb[1]
        odom_uwb.pose.pose.position.z = x_uwb[2]
        odom_uwb.twist.twist.linear.x = vel_uwb[0]
        odom_uwb.twist.twist.linear.y = vel_uwb[1]
        odom_uwb.twist.twist.linear.z = vel_uwb[2]
        odom_uwb.twist.twist.angular.x = 0
        odom_uwb.twist.twist.angular.y = 0
        odom_uwb.twist.twist.angular.z = 0
        pub_uwb.publish(odom_uwb)



def uwb_callback(data):
    if init == 2:
        i = data.anchorId
        r = data.range*0.001
        filter.range[i] = r
        filter.uwbCeck[i] = 1
        if sum(filter.uwbCeck) == filter.nAnchors:
            filter.correction()
            filter.uwbCeck = np.zeros(4)

def activation_callback(data):
    global filter
    filter.mode = data.data
    # if data.data == 3:
    #     filter.x_[6:9, :] = 0


def model_callback(data):
    global init
    global filter
    global count
    count = count + 1
    pos_true = np.zeros(3)
    if init == 1:
        q = np.zeros(4)
        x = np.zeros(12)
        x[0] = data.pose[5].position.x
        x[1] = data.pose[5].position.y
        x[3] = data.twist[5].linear.x
        x[4] = data.twist[5].linear.y
        x[6:9] = 0.0
        q[0] = data.pose[5].orientation.w
        q[1] = 0
        q[2] = 0
        q[3] = data.pose[5].orientation.z
        filter = shfaf(R=None, Q=None, P=None, x=x, q=q, window_width=100, a=0.9784)
        filter.nAnchors = 4
        filter.anchorPos = np.array([[-1, -1],
                                     [9, -1],
                                     [-1, 9],
                                     [9, 9],
                                     ])
        rospy.loginfo('Filter initialized at x: ')
        rospy.loginfo(filter.x)
        rospy.loginfo('q: ')
        rospy.loginfo(filter.q)
        init = 0
    true_q = np.array([data.pose[5].orientation.w, 0, 0, data.pose[5].orientation.z])
    heading_true = quaternion.as_euler_angles(quaternion.from_float_array(true_q))
    pub_heading_true.publish(heading_true[2])
    if count % 200 == 0 and init==0:
        out = np.greater(np.random.rand(), 0.95).astype(int)
        pos_outlier = out * (np.random.rand(3)-0.5)*4
        pos_true[0] = data.pose[5].position.x
        pos_true[1] = data.pose[5].position.y
        pos_true[2] = data.pose[5].position.z
        uwb_noise = np.random.normal(0,0.1,3)
        filter.posOld = pos_true + uwb_noise + pos_outlier
        filter.velOld[0] = data.twist[5].linear.x + np.random.normal(0.0, 0.1) + uwb_noise[0]*5 + pos_outlier[0]*5
        filter.velOld[1] = data.twist[5].linear.y + np.random.normal(0.0, 0.1) + uwb_noise[1]*5  + pos_outlier[1]*5
        filter.velOld[2] = data.twist[5].linear.z + np.random.normal(0.0, 0.1) + uwb_noise[2]*5  + pos_outlier[2]*5
        error = np.linalg.norm(pos_true - filter.x_[:3,0])
        pub_error.publish(error)
        if filter.mode != 3:
            filter.correction()
        count = 0






def node():

    rospy.init_node('estimator', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/gtec/toa/ranging", Ranging, uwb_callback)
    rospy.Subscriber("/filter_mode", Int8, activation_callback)
    rospy.Subscriber("/gazebo/model_states", LinkStates, model_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    node()

