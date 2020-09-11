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
from sensor_msgs.msg import JointState


rospy.init_node("get_joint_input")
#import tf

def callback(msg):
	# print(msg.pose[2].orientation)
	#(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	#print(p)
	#print(msg.pose[2].position.x)
	#print('常量 PI 的值近似为： {}。'.format(math.pi))
    cartpole_msg = catpoleuMsg()
    global i
    global sum_u
    """for i in range(10):
	    current_x[i] = msg.pose[2].position.x
		sum_x = sum_x +  current_x[i]
		print(current_x[i])
	averagex_10=sum_x/10"""
    curretu=msg.effort[0]
    print('current u is :{}'.format(curretu))
    if i<9:
        i=i+1
        sum_u = sum_u+curretu
    elif i==9:
        i=0
        averageu_10=(sum_u+curretu)/10
        sum_u=0
        print('average current joint input u is :{}'.format(averageu_10))#1000HZ->100HZ
        cartpole_msg.current_u = averageu_10
        u_pub.publish(cartpole_msg)
	#rospy.sleep(1.)

if __name__ == '__main__':
    #rospy.init_node('xpublisher', anonymous=True)
    i=0
    sum_u=0
    #sensor_msgs/JointState
    u_pub = rospy.Publisher('/currentu', catpoleuMsg, queue_size=100)
    sub = rospy.Subscriber("/joint_states",JointState,callback, queue_size=1, buff_size=100)#1000HZ
    rospy.spin()