#!/usr/bin/env python3

from signal import SIG_DFL
import rospy
import logging
from std_msgs.msg import Header
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

# TODO: Make it so that the state machine will try to pick up blocks when the conveyor has stopped, also think about how 
#       to make it so that it tries to pick up a block while its moving for Task 3b. Maybe just always make the state machine
#       try to grab a block while moving? Will probably need to predict where the cube will be in advance (calculate the 
#       circular trajectory and wait above a spot until the cube reaches it).


# TODO: Check that these zones are defined correctly
# Red bin is zone 1, green bin is zone 2, blue bin is zone 3 and yellow bin is zone 4
bin_dict = {"red":[150, -50, 200], "green":[50, -150, 200], "blue":[-50, -150, 200], "yellow":[-150, -50, 200]}
# TODO: Make a reasonable spot that won't hit the environment, decide with team
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
    # TODO: Decide if the +-5% is appropriate
    def end_effector_arrived(self, pose):
        
        if(self.current_end_effector < 1.05 * pose.position) and (self.current_end_effector > 0.95 * pose.position):
            return True
        else:
            return False

    
    # Function that picks the cube that is reachable and that is currently the closest to transport to a bin. If there are no
    # reachable cubes then return 'None'
    def pick_cube(self):
        self.cube_dict.sort()
        for cube in self.cube_dict:
            if not cube.is_obstructed():
                return cube.position
        return None



# Class describing a cube object
class TagCube:

    def __init__(self, msg):
        # FIXME: Make sure this msg type is set up
        self.id = msg.id
        self.colour = msg.colour
        self.position = msg.position

    # TODO: Consider changing this to be position differences between two cubes and changing bin distance to something else
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

    # Determines wheter a particular block is obstructed by any other blocks around it
    def is_obstructed(self):
        # TODO: Make sure the loop through the dictionary doesn't flag itself as it will always return True
        for cube in self.cube_dict:
            if (cube.id != self.id):
            # If two cubes' positions (mid points) are closer than 40 in the x direction (direction in line with the gripper) then consider them right next to each other 
                if (self.position[0] - cube.position[0] <= 40):
                    return True
        # If the for loop gets all the way through without triggering True then the block is not obstructed by any other blocks around it
        return False


# Main robot loop
def main():

    state_machine = StateMachineNode()
    
    while(True):
        if state_machine.state == "waiting":
            # If robot is in home position then publish the desired location to move
            if(state_machine.current_end_effector == home_pose and t < 10):
                # This line pops the first key value pair off the dictionary which corresponds to the one with minimised distance
                cube = state_machine.cube_dict.pop(list(state_machine.cube_dict)[0])
                
                # Form ROS msg for the desired pose
                desiredPoseMsg = Pose(header=Header(stamp=rospy.Time.now()))
                
                # Check to see this cube assignment works okay, cube should be [x y z]
                desiredPoseMsg.position = cube.get_position()
                state_machine.posePub(desiredPoseMsg)
            else:
                # If the robot is waiting but its not in its home config that means its gotten stuck somehow
                logging.warning("Robob stugg :(")

        elif state_machine.state == "grabbing":
            # Add a check here for if its been past a certain time threshold to move the state to obstructed
            if(state_machine.end_effector_arrived(desiredPoseMsg.position)):
                state_machine.close_gripper()
                #TODO: Might need a way to check if the thing got obstructed
                state_machine.posePub(home_pose)
                if(state_machine.current_end_effector != home_pose and t > 10)
                    state_machine.current_state = "obstructed"
                else:
                    # Form ROS msg for the desired pose
                    desiredPoseMsg = Pose(header=Header(stamp=rospy.Time.now()))
                    
                    # Check to see this cube assignment works okay, cube should be [x y z]
                    desiredPoseMsg.position = bin_dict[cube.get_colour()]
                    state_machine.posePub(desiredPoseMsg)
                    state_machine.current_state = "dropping"

            else:
                state_machine.current_state = "obstructed"

        elif state_machine.state == "dropping":
            if(state_machine.end_effector_arrived() and t < 10):
                state_machine.open_gripper()
                state_machine.posePub(home_pose)
                state_machine.current_state = "waiting"
            else:
                state_machine.current_state = "obstructed"
    
        elif state_machine.state == "obstructed":
            state_machine.posePub(home_pose)
            state_machine.current_state = "waiting"
        
        else:
            print("Invalid state")

        rospy.sleep(10)
    

if __name__ == "__main__":
    main()