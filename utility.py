#!/usr/bin/env python3

import rospy
import logging
from std_msgs.msg import Header
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

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

