#!/usr/bin/env python3

from signal import SIG_DFL
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
bin_dict = {"red":[150 -50 200], "green":[50 -150 200], "blue":[-50 -150 200], "yellow":[-150 -50 200]}
# Make a reasonable spot that won't hit the environment
home_pose = Pose()


class StateMachineNode:

    def __init__(self):
        rospy.init_node("state_machine_node")

        # Defining state variables that determine whether or not state can be changed
        
        self.current_state = "waiting"
        self.current_end_effector = Pose()
        self.cube_dict = {}
               

        # Defining the constants required for an opened and closed gripper
        self.gripper_close , self.gripper_open = Float32()
        self.gripper_close.data = 1500
        self.gripper_open.data = 2000

        # Instantiating all the required publishers and subscribers associated with this node
        self.tag_pose_sub = rospy.Subscriber("/tag_pose", data_class=Pose, callback=self.tag_pose_callback)
        self.end_effector_pose_sub  = rospy.Subscriber("/end_effector_pose", data_class=Pose, callback=self.end_effector_pose_callback)
        self.desired_pose_pub = rospy.Publisher("/desired_pose", data_class=JointState, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)

        # Instantiate the position of the motors
        self.desired_pose_pub.publish(Pose(position=home_pose))
    
    def end_effector_pose_callback(self, pose: Pose):
        self.current_end_effector = pose.position
        logging.info("End effector pose received")


    def tag_pose_callback(self, pose: Pose):
        # Add the scanned cube to the cube dictionary
        cube = TagCube(pose)
        self.cube_dict[cube.get_id()] = cube
        logging.info("Tag pose recieved") 


    def open_gripper(self):
        self.gripper_pub.publish(self.gripper_open)


    def close_gripper(self):
        self.gripper_pub.publish(self.gripper_close)


    # Detects whether the end effector position is within the accepted tolerance of +-5% of the desired pose
    def end_effector_arrived(self, pose):
        
        if(self.current_end_effector < 1.05 * pose.position) and (self.current_end_effector > 0.95 * pose.position):
            return True
        else:
            return False

    
    # Have part of the decision algorithm be a function that picks the minimum distance between a block and its respective bin
    # TODO:Have the other portion of the algorithm decide whether a block can be picked up
    def pick_cube(self):
        self.cube_dict.sort()


# Class describing a cube object
class TagCube:

    def __init__(self):
        # FIXME: Change passes lol
        self.id = pass
        self.colour = pass
        self.position = pass

    # Overloading comparison functions to make sorting easier
    def __lt__(self, other):
        return self.get_bin_distance() < other.get_bin_distance()

    def __le__(self, other):
        return self.get_bin_distance() <= other.get_bin_distance()
    
    def __gt__(self, other):
        return self.get_bin_distance() > other.get_bin_distance()
    
    def __ge__(self, other):
        return self.get_bin_distance() >= other.get_bin_distance()
    
    def __eq__(self, other):
        return self.get_bin_distance() == other.get_bin_distance()
    
    def __ne__(self, other):
        return self.get_bin_distance() != other.get_bin_distance()

    def get_id(self):
        return self.id

    def get_colour(self):
        return self.colour

    def get_position(self):
        return self.position

    # Distance between 2 cubes here is defined as the difference between their distances to their bins, not their conventional distance
    def get_bin_distance(self):
        return abs(self.get_position - bin_dict[self.get_colour()])

    def is_beside
        

# Main robot loop
def main():

    state_machine = StateMachineNode()
    
    while(True):
        match state_machine.state:
            case "waiting":
                # If robot is in home position then publish the desired location to move
                if(state_machine.current_end_effector == home_pose):
                    # This line pops the first key value pair off the dictionary which corresponds to the one with minimised distance
                    cube = state_machine.cube_dict.pop(list(state_machine.cube_dict)[0])
                    
                    # Form ROS msg for the desired pose
                    desiredPoseMsg = Pose(header=Header(stamp=rospy.Time.now()))
                    
                    # Check to see this cube assignment works okay, cube should be [x y z]
                    desiredPoseMsg.position = cube
                    state_machine.posePub(desiredPoseMsg)
                else:
                    # If the robot is waiting but its not in its home config that means its gotten stuck somehow
                    logging.warning("Robob stugg :(")

            case "grabbing":
                if(state_machine.end_effector_arrived()):
                    state_machine.close_gripper()
                    state_machine.current_state = "dropping"
                else:
                    state_machine.current_state = "obstructed"

            case "dropping":
                if(state_machine.end_effector_arrived()):
                    state_machine.open_gripper()
                    state_machine.posePub(home_pose)
                    state_machine.current_state = "waiting"
                else:
                    state_machine.current_state = "obstructed"
        
            case "obstructed":
                state_machine.posePub(home_pose)
                state_machine.current_state = "waiting"
            
            case _:
                print("Invalid state")

        rospy.sleep(5)
    

if __name__ == "__main__":
    main()