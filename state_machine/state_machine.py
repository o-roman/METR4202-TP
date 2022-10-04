#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import JointState
import numpy as np
from modern_robotics import (
    MatrixExp6,
    VecTose3
)

# Define robot workspace


class StateMachineNode(object):
    

    def __init__(self):
        rospy.init_node("state_machine_node")

        # Defining state variables that determine whether or not state can be changed
        # current pose is forward kinematics on the reported joint angles (unless there is a way to report where the end effector is directly)
        currentPose = FK(self.jointStateSub)
        state = ""
        obstructed = False

        # Defining the constants required for an opened and closed gripper
        gripperClose = Float32()
        gripperClose.data = 1500

        gripperOpen = Float32()
        gripperOpen.data = 2000


        tagPos = Pose()
        currentJoint = JointState()

        # Instantiating all the required publishers and subscribers associated with this node
        self.desJointStateSub = rospy.Subscriber("/desired_joint_states", data_class=JointState, callback=self.des_joint_state_callback)
        self.jointStateSub = rospy.Subscriber("/joint_states", data_class=JointState, callback=self.joint_state_callback)
        self.tagPose = rospy.Subscriber("/tag_pose", data_class=Pose, callback=self.tag_pose_callback)
        self.posePub = rospy.Publisher("/desired_pose", data_class=JointState, queue_size=10)
        self.gripperPub = rospy.Publisher("/gripper", Float32, queue_size=10)
        rate = rospy.Rate(10)


    def tag_pose_callback(pose: Pose):
        print("tag pose recieved")
        tagPos = pose
        # TODO: Set up a dictionary with keys being the aruco tags and values being the pose position to easily update values of each of the cubes


    def des_joint_state_callback():
        print("desired joint state recieved")


    def joint_state_callback(joints: JointState):
        currentJoint = joints
        print("current joint state recieved")


    def open_gripper():
        self.gripperPub.publish(gripperOpen)


    def close_gripper():
        self.gripperPub.publish(gripperClose)


# Main robot loop
def main():
    stateMachine = StateMachineNode() 
    while(1):
        match state:
            case "waiting":
                # If robot is in home position then
                stateMachine.posePub(tag_pos)
                state = "grabbing"

            case "grabbing":


            case "dropping":


            case _:
                print("Invalid state")
        rospy.sleep(5)
    




if __name__ == "__main__":
    main()