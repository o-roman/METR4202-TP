#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import random

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransform
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Pose

import numpy as np



def read_file():
    fname = '/home/metr4202-team16/catkin_ws/src/metr4202_w7_prac/scripts/test2.txt'
    with open(fname, 'r+', encoding='utf-8') as f:
        s = [i[:-1].split(',') for i in f.readlines()]
    transer_data = np.zeros(shape=(1701, 7))

    for i in range(len(transer_data)):
        for j in range(7):
            transer_data[i][j] = float(s[i][j])
    for i in range(len(transer_data)):
        transer_data[i][0] = transer_data[i][0]-1.507       
    return transer_data 
def read_file_2():
    fname = '/home/metr4202-team16/catkin_ws/src/metr4202_w7_prac/scripts/test.txt'
    with open(fname, 'r+', encoding='utf-8') as f:
        s = [i[:-1].split(',') for i in f.readlines()]
    transer_data = np.zeros(shape=(1793, 7))

    for i in range(len(transer_data)):
        for j in range(7):
            transer_data[i][j] = float(s[i][j])
    for i in range(len(transer_data)-1):
        transer_data[i][0] = transer_data[i][0]-1.507  


    #print(transer_data)
    return transer_data
def searching_file(x,y,z):
    joint_array = [0,0,0,0]
    flag =0
    x_int=int(x)
    x_float_part = x-x_int
    if x_float_part <0.25:
        x_out = x_int
    if x_float_part>=0.25 and x_float_part<=0.75:
        x_out = x_int+0.5
    if x_float_part>0.75:
        x_out = x_int +1

    y_int = int(y)
    y_float_part = y - y_int
    if y_float_part < 0.25:
        y_out = y_int
    if y_float_part >= 0.25 and y_float_part <= 0.75:
        y_out = y_int + 0.5
    if x_float_part > 0.75:
        y_out = y_int + 1

    z_out = 1.5
    coordinate = [x_out,y_out,z_out]
    joint_coordi_info = read_file()
    if x_out == 0 and y_out ==0 and z_out == 0:
        joint_array=[0,0,0,0]
        return joint_array

    for i in range(len(joint_coordi_info)):
       
        if joint_coordi_info[i][4] == x_out and joint_coordi_info[i][5] == y_out and joint_coordi_info[i][6] == z_out:
            joint0 = joint_coordi_info[i][0]
            joint1 = joint_coordi_info[i][1]
            joint2 = joint_coordi_info[i][2]
            joint3 = joint_coordi_info[i][3]
            joint_array = [joint0,joint1,joint2,joint3]
            flag = 1
                #print(joint_coordi_info[i])
    if flag ==0:
        joint_coordi_info = read_file_2()
        for i in range(len(joint_coordi_info)):
           
            if joint_coordi_info[i][4] == x_out and joint_coordi_info[i][5] == y_out and joint_coordi_info[i][6] == z_out:
                joint0 = joint_coordi_info[i][0]
                joint1 = joint_coordi_info[i][1]
                joint2 = joint_coordi_info[i][2]
                joint3 = joint_coordi_info[i][3]
                joint_array = [joint0,joint1,joint2,joint3]
                flag = 1

    print(joint_array)
    print(coordinate)
    return joint_array




# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(fidTrans) -> JointState:
    global pub
    #print(str(fidTrans))
    #print(fidTrans.transforms[0].transform.translation)
    
    fidTrans.transforms[0].transform.translation.x = fidTrans.transforms[0].transform.translation.x*100
    fidTrans.transforms[0].transform.translation.y = fidTrans.transforms[0].transform.translation.y*100+22.8
    fidTrans.transforms[0].transform.translation.z = 2
    
    
    #print(fidTrans.transforms[0].transform.translation)
    # TODO: Have fun :)
    #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(dummy_joint_states(fidTrans.transforms[0].transform.translation))


# Funny code
def dummy_joint_states(position) -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current timemy_joint_states(pose.position))
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )

    joint_array = searching_file(position.x,position.y,position.z)
    
    # Funny code   
    msg.position = [
        -joint_array[0],
        joint_array[1],
        joint_array[2],
        -joint_array[3]

    ]
    # Funny code
    
    '''msg.position = [
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5)
    ]'''
    return msg


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )
    
    # Create subscriber
    sub = rospy.Subscriber(
        #'desired_pose', # Topic name
        '/fiducial_transforms',
        FiducialTransformArray, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('inversekinematic')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
