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
from gazebo_msgs.msg import LinkState
from cartpole.msg import catpolexMsg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
#global fig, ax
"""def plotphase_portrait():
    fig, ax = plt.subplots()
    u_trajectory = dircol.ReconstructInputTrajectory(result)
    times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
    u_lookup = np.vectorize(u_trajectory.value)
    u_values = u_lookup(times)
    ax.plot(times, u_values)
    ax.set_xlabel("time (seconds)")
    ax.set_ylabel("force (Newtons)")
    ax.set_title(' Direct collocation for Cartpole ')
    plt.show()
#x_knots = np.hstack([
#    x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                             #  x_trajectory.end_time(), 100)
#])
    x_trajectory = dircol.ReconstructStateTrajectory(result)
    x_knots = np.hstack([
        x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                   x_trajectory.end_time(), 100)
    ])
    print(x_trajectory.start_time())
    print(x_trajectory.end_time())
    
    fig, ax = plt.subplots(2,1,figsize=(8,8))
    plt.subplots_adjust(wspace =0, hspace =0.5)
    #plt.tight_layout(3)#adjust total space
    ax[0].set_title('state of direct collocation for Cartpole')
    ax[0].plot(x_knots[0, :], x_knots[2, :], linewidth=2, color='b', linestyle='-')
    ax[0].set_xlabel("state_dot(theta1_dot and t|heta2_dot)")
    ax[0].set_ylabel("state(theta1 and theta2)");
    ax[0].plot(x_knots[1, :], x_knots[3, :],color='r',linewidth=2,linestyle='--')
    ax[0].legend(('theta1&theta1dot','theta2&theta2dot'));
    ax[1].set_title('input u(t) of direct collocation for Cartpole')
    ax[1].plot(times,u_values, 'g')
    ax[1].legend(('input u(t)'))
    ax[1].set_xlabel("time")
    ax[1].set_ylabel("u(t)")
    plt.show()"""
global times

def drake_trajectory_generation(file_name):
    print(file_name)
    Parser(plant).AddModelFromFile(file_name)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    global dircol 
    dircol= DirectCollocation(
        plant,
        context,
        num_time_samples=11,
        minimum_timestep=0.1,
        maximum_timestep=0.4,
        input_port_index=plant.get_actuation_input_port().get_index())
    dircol.AddEqualTimeIntervalsConstraints()
    initial_state = (0., 0., 0., 0.)
    dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                dircol.initial_state())
    final_state = (0., math.pi, 0., 0.)
    dircol.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())
    R = 10  # Cost on input "effort".weight
    u = dircol.input()
    dircol.AddRunningCost(R * u[0]**2)
    # Add a final cost equal to the total duration.
    dircol.AddFinalCost(dircol.time())
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        [0., 4.], np.column_stack((initial_state, final_state)))  # yapf: disable
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)
    global result
    global u_values
    result = Solve(dircol)
    assert result.is_success()
    #plotphase_portrait()
    fig1, ax1 = plt.subplots()

    u_trajectory = dircol.ReconstructInputTrajectory(result)
    times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 400)
    u_lookup = np.vectorize(u_trajectory.value)
	#now we have ndarray of u_values with 400 points for 4 seconds w/ 100hz pub frequency
    u_values = u_lookup(times)

    ax1.plot(times, u_values)
    ax1.set_xlabel("time (seconds)")
    ax1.set_ylabel("force (Newtons)")
    ax1.set_title(' Direct collocation for Cartpole ')
    print('here2')
    plt.show()
    print('here3')
    #x_knots = np.hstack([
    #    x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                 #  x_trajectory.end_time(), 100)
    #])
    x_trajectory = dircol.ReconstructStateTrajectory(result)
    x_knots = np.hstack([
        x_trajectory.value(t) for t in np.linspace(x_trajectory.start_time(),
                                                   x_trajectory.end_time(), 400)
    ])
    print(x_trajectory.start_time())
    print(x_trajectory.end_time())
   

    fig, ax = plt.subplots(2,1,figsize=(8,8))
    plt.subplots_adjust(wspace =0, hspace =0.5)
    #plt.tight_layout(3)#adjust total space
    ax[0].set_title('state of direct collocation for Cartpole')
    ax[0].plot(x_knots[0, :], x_knots[2, :], linewidth=2, color='b', linestyle='-')
    ax[0].set_xlabel("state_dot(theta1_dot and t|heta2_dot)")
    ax[0].set_ylabel("state(theta1 and theta2)");
    ax[0].plot(x_knots[1, :], x_knots[3, :],color='r',linewidth=2,linestyle='--')
    ax[0].legend(('theta1&theta1dot','theta2&theta2dot'));
    ax[1].set_title('input u(t) of direct collocation for Cartpole')
    ax[1].plot(times,u_values, 'g')
    ax[1].legend(('input u(t)'))
    ax[1].set_xlabel("time")
    ax[1].set_ylabel("u(t)")
    print('here4')
    plt.show()
    print('here5')
    return x_knots[0, :]#u_values
def currentCallback(msg):
    #rospy.loginfo("Subcribe Person Info: name:%s  age:%d  sex:%d", 
    #		 msg.name, msg.age, msg.sex)
    #here we need LQR to tracking the trajectory
	
    rospy.loginfo("we need LQR to tracking")
    j=0
    j=j+1

def drake_state_publisher(x_cmd):
    rate = rospy.Rate(100)
    times=0
    xmsg=LinkState()
	
    while not rospy.is_shutdown():
        xmsg.link_name='cartpole::link_chassis'
#        hello_str = "hello world %s" % rospy.get_time()
        buff=Pose
        position_buff=Point
        orientation_buff=Quaternion
        position_buff.x=x_cmd[times]
        position_buff.y=0
        position_buff.z=0
        orientation_buff.x=0
        orientation_buff.y=0
        orientation_buff.z=0
        orientation_buff.w=1
        buff.position=position_buff
        buff.orientation=orientation_buff
        #xmsg.pose = buffpose.position.x
        #xmsg.twist = [1,0,0,0,0,0]
        xmsg.pose.position.x = x_cmd[times]
        xmsg.pose.position.z = 2
        drake_state_pub.publish(xmsg)
        rospy.loginfo(x_cmd[times])
        drake_state_pub_echo.publish(x_cmd[times])
        times=times+1
#'current x state is :{}'.format(statex)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drake_control', anonymous=True)
    
    #fig, ax = plt.subplots(2,1,figsize=(8,8))
    i=0
    global x_cmd
    
    sum_x=0
    #x_pub = rospy.Publisher('/curretx', catpolexMsg, queue_size=100)
    #drake_cmd_pub = rospy.Publisher('/chassis_world_effort_controller/command', Float64, queue_size=100)
    drake_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=100)
    drake_state_pub_echo = rospy.Publisher('/drake_state_pub_echo', Float64, queue_size=100)
	#/chassis_world_effort_controller/command
   # sub = rospy.Subscriber("/curretx",catpolexMsg,currentCallback, queue_size=1, buff_size=100)#1000HZ
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    file_name = FindResource("/home/cby/catkin_ws/src/cartpole/urdf/cartpole2.urdf")
    x_cmd=drake_trajectory_generation(file_name)#x_cmd with 400points  effort input
    rospy.loginfo("trajectory complete!! sending")
    drake_state_publisher(x_cmd)
	
    #rospy.spin()