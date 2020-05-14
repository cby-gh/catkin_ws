#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
rospy.init_node("get_link_state")
import tf
def callback(msg):
	# print(msg.pose[2].orientation)
	(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])
	print(p)

sub = rospy.Subscriber("/gazebo/link_states",LinkStates,callback)
rospy.spin()
