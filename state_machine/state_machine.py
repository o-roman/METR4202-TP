#!/usr/bin/env python3

import rospy
import logging
from std_msgs.msg import Header
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

# Define global variables
# TODO: Check that these zones are defined correctly
# Red bin is zone 1, green bin is zone 2, blue bin is zone 3 and yellow bin is zone 4
binDict = {"red":[150 -50 200], "green":[50 -150 200], "blue":[-50 -150 200], "yellow":[-150 -50 200]}
# Make a reasonable spot that won't hit the environment
homePose = Pose()


class StateMachineNode:

    def __init__(self):
        rospy.init_node("state_machine_node")

        # Defining state variables that determine whether or not state can be changed
        
        self.currentState = "waiting"
        self.currentEndEffector = Pose()
        self.cubeDict = {}
               

        # Defining the constants required for an opened and closed gripper
        self.gripperClose , self.gripperOpen = Float32()
        self.gripperClose.data = 1500
        self.gripperOpen.data = 2000

        # Instantiating all the required publishers and subscribers associated with this node
        self.tagPoseSub = rospy.Subscriber("/tag_pose", data_class=Pose, callback=self.tag_pose_callback)
        self.endEffectorPoseSub = rospy.Subscriber("/end_effector_pose", data_class=Pose, callback=self.end_effector_pose_callback)
        self.desiredPosePub = rospy.Publisher("/desired_pose", data_class=JointState, queue_size=10)
        self.gripperPub = rospy.Publisher("/gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)

        # Instantiate the position of the motors
        self.desiredPosePub.publish(Pose(position=homePose))
    
    def end_effector_pose_callback(self, pose: Pose):
        self.currentEndEffector = pose.position
        logging.info("End effector pose received")


    def tag_pose_callback(self, pose: Pose):
        # Add the scanned cube to the cube dictionary
        cube = TagCube(pose)
        self.cubeDict(cube.tag) = cube
        logging.info("Tag pose recieved") 


    def open_gripper(self):
        self.gripperPub.publish(self.gripperOpen)


    def close_gripper(self):
        self.gripperPub.publish(self.gripperClose)


    # Detects whether the end effector position is within the accepted tolerance of +-5% of the desired pose
    def end_effector_arrived(self, pose):
        
        if(self.currentEndEffector < 1.05 * pose.position) and (self.currentEndEffector > 0.95 * pose.position):
            return True
        else:
            return False

    
    # Have part of the decision algorithm be a function that picks the minimum distance between a block and its respective bin
    # TODO:Have the other portion of the algorithm decide whether a block can be picked up
    def pick_cube(self):
        self.cubeDict.sort()


# Class describing a cube object
# TODO: Finish class object, define colour, position, id
class TagCube:

    def __init__(self):
        # TODO: Change passes lol
        self.id = pass
        self.colour = pass
        self.position = pass

    # Overloading comparison functions to make sorting easier
    def __lt__(self, other):
        return self.getDistance() < other.getDistance()

    def __le__(self, other):
        return self.getDistance() <= other.getDistance()
    
    def __gt__(self, other):
        return self.getDistance() > other.getDistance()
    
    def __ge__(self, other):
        return self.getDistance() >= other.getDistance()
    
    def __eq__(self, other):
        return self.getDistance() == other.getDistance()
    
    def __ne__(self, other):
        return self.getDistance() != other.getDistance()

    def getId(self):
        return self.id

    def getColour(self):
        return self.colour

    def getPosition(self):
        return self.position

    # Distance between 2 cubes here is defined as the difference between their distances to their bins, not their conventional distance
    def getBinDistance(self):
        return abs(self.getPosition - binDict[self.getColour()])
        

# Main robot loop
def main():

    stateMachine = StateMachineNode()
    
    while(True):
        match stateMachine.state:
            case "waiting":
                # If robot is in home position then publish the desired location to move
                if(stateMachine.currentEndEffector == homePose):
                    # This line pops the first key value pair off the dictionary which corresponds to the one with minimised distance
                    cube = stateMachine.cubeDict.pop(list(stateMachine.cubeDict)[0])
                    
                    # Form ROS msg for the desired pose
                    desiredPoseMsg = Pose(header=Header(stamp=rospy.Time.now()))
                    
                    # TODO: Check to see this cube assignment works okay, cube should be [x y z]
                    desiredPoseMsg.position = cube
                    stateMachine.posePub(desiredPoseMsg)
                else:
                    # If the robot is waiting but its not in its home config that means its gotten stuck somehow
                    logging.warning("Robob stugg :(")

            case "grabbing":
                if(stateMachine.end_effector_arrived()):
                    stateMachine.close_gripper()
                    stateMachine.currentState = "dropping"
                else:
                    stateMachine.currentState = "obstructed"

            case "dropping":
                if(stateMachine.end_effector_arrived()):
                    stateMachine.open_gripper()
                    stateMachine.posePub(homePose)
                    stateMachine.currentState = "waiting"
                else:
                    stateMachine.currentState = "obstructed"
        
            case "obstructed":
                stateMachine.posePub(homePose)
                stateMachine.currentState = "waiting"
            
            case _:
                print("Invalid state")

        rospy.sleep(5)
    

if __name__ == "__main__":
    main()