#!/usr/bin/env python3.8
import rospy
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState

rospy.init_node('rotate_table')
rate = rospy.Rate(hz=10)

table_velocity = - 0.11

while not rospy.is_shutdown():
    state = LinkState()
    state.link_name = 'Rot_table::link_1'
    state.reference_frame = 'Rot_table::link_0'

    state.twist.angular.z = table_velocity

    set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    result = set_state(state)
    rate.sleep()
