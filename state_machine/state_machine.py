#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
import numpy as np
from modern_robotics import (
    MatrixExp6,
    VecTose3
)

class StateMachineNode(object):
    

    def __init__(self):
        rospy.init_node("state_machine_node")

        # Defining the constants required for an opened and closed gripper
        gripperClose = Float32()
        gripperClose.data = 1500

        gripperOpen = Float32()
        gripperOpen.data = 2000

        # Instantiating all the required publishers and subscribers associated with this node
        self.joint_state_sub = rospy.Subscriber("/desired_joint_states", data_class=JointState, callback=self.joint_state_callback)
        self.pose_pub = rospy.Publisher("/desired_pose", data_class=JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper", Float32, queue_size=10)
        rate = rospy.Rate(10)
