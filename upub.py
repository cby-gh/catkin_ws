#! /usr/bin/env python
import sys 
sys.path.append("/home/cby/underactuated/") 

import math
import numpy as np
import matplotlib.pyplot as plt
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.all import (DiagramBuilder, DirectCollocation, MultibodyPlant,
                         MultibodyPositionToGeometryPose, Parser,
                         PiecewisePolynomial, PlanarSceneGraphVisualizer,
                         SceneGraph, Simulator, Solve, TrajectorySource)
from underactuated import FindResource


import rospy
from gazebo_msgs.msg import LinkStates
from cartpole.msg import catpolexMsg
from cartpole.msg import catpoleuMsg
from cartpole.msg import catpolexMsgx0x1
from cartpole.msg import catpoleuMsgu0u1
from cartpole.msg import catpolexMsgx0
from cartpole.msg import catpoleuMsgu0
from cartpole.msg import catpolexMsgx1
from cartpole.msg import catpoleuMsgu1
from sensor_msgs.msg import JointState


rospy.init_node("get_joint_input")
#import tf

def callback(msg):
	# print(msg.pose[2].orientation)
	#(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	#print(p)
	#print(msg.pose[2].position.x)
	#print('常量 PI 的值近似为： {}。'.format(math.pi))
    cartpoleu0_msg = catpoleuMsgu0()
    cartpoleu1_msg = catpoleuMsgu1()
    cartpolex0_msg = catpolexMsgx0()
    cartpolex1_msg = catpolexMsgx1()
    global i
    global sum_u0
    global sum_u1
    global sum_x0
    global sum_x1
    """for i in range(10):
	    current_x[i] = msg.pose[2].position.x
		sum_x = sum_x +  current_x[i]
		print(current_x[i])
	averagex_10=sum_x/10"""
    curretu0=msg.effort[0]
    curretu1=msg.effort[1]
    curretx0=msg.position[0]
    curretx1=msg.position[1]
    print('current u is :{}'.format(curretu0))
    if i<9:
        i=i+1
        sum_u0 = sum_u0+curretu0
        sum_u1 = sum_u1+curretu1
        sum_x0 = sum_x0+curretx0
        sum_x1= sum_x1+curretx1
    elif i==9:
        i=0
        averageu0_10=(sum_u0+curretu0)/10
        averageu1_10=(sum_u1+curretu1)/10
        averagex0_10=(sum_x0+curretx0)/10
        averagex1_10=(sum_x1+curretx1)/10
        sum_u0=0
        sum_u1=0
        sum_x0=0
        sum_x1=0
        print('average current joint input u is :{}'.format(averageu0_10))#this module we change frequency 1000HZ->100HZ
        cartpoleu0_msg.current_u0 = averageu0_10
        cartpoleu1_msg.current_u1 = averageu1_10
        cartpolex0_msg.current_x0 = averagex0_10
        cartpolex1_msg.current_x1 = averagex1_10
        u0_pub.publish(cartpoleu0_msg)
        u1_pub.publish(cartpoleu1_msg)
        x0_pub.publish(cartpolex0_msg)
        x1_pub.publish(cartpolex1_msg)
	#rospy.sleep(1.)

if __name__ == '__main__':
    #rospy.init_node('xpublisher', anonymous=True)
    i=0
    sum_u0=0
    sum_u0=0
    sum_u1=0
    sum_x0=0
    sum_x1=0
    #sensor_msgs/JointState
    u0_pub = rospy.Publisher('/currentu0', catpoleuMsgu0, queue_size=100)
    u1_pub = rospy.Publisher('/currentu1', catpoleuMsgu1, queue_size=100)
    x0_pub = rospy.Publisher('/currentx0', catpolexMsgx0, queue_size=100)
    x1_pub = rospy.Publisher('/currentx1', catpolexMsgx1, queue_size=100)
    sub = rospy.Subscriber("/joint_states",JointState,callback, queue_size=1, buff_size=100)#1000HZ
    rospy.spin()