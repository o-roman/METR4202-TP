#!/usr/bin/env python3
from re import T
import numpy as np
import rospy
# import logging
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import *

import sys
sys.path.insert(1, '/home/metr4202/catkin_ws/src/metr4202_tp/inverse_kinematics/msg')
import JointStateArray.msg 

def IKinSpace(x,y,z):
    L1 = 100
    L2 = 117.5
    L3 = 95
    L4 = 110

    j0 = np.arctan2(y,x)
    L=x/np.cos(j0)
    thetalist = np.empty((1,4))
    for theta2 in range(0,90):
        j1 = theta2 / 180 * np.pi
     
        Lc = L2 * np.cos(j1)
        Ls = L2 * np.sin(j1)
      
        beta = np.arctan2(L-Ls,L1+Lc)
        L5 = np.sqrt((L-Ls)**2+(L1+Lc-z)**2)
        ac1 = (L3**2+L4**2-L5**2)/(2*L3*L4)
        ac2 = (L3**2+L5**2-L4**2)/(2*L3*L5)
        '''
        print(ac1)
        print(ac2)
        '''

        if -1<ac1<1 and -1<ac2<1:
            j3 = np.pi-np.arccos(ac1)
            j2 = np.pi-j1-beta-np.arccos(ac2)
            # A=L*np.cos(j0)
            # B=L*np.sin(j0)
            # C=L1+L2*np.cos(j1)+L3*np.cos(j1+j2)+L4*np.cos(j1+j2+j3)
            thetalist = np.append(thetalist,[[j0,j1,j2,j3]],axis=0)
            # print(j0,j1,j2,j3,A,B,C)
        else:
            pass
        '''
        while -1<ac1<1 and -1<ac2<1:
            j3 = np.pi-np.arccos(ac1)
            j2 = np.pi-j1-beta-np.arccos(ac2)
            A=L*np.cos(j0)
            B=L*np.sin(j0)
            C=L1+L2*np.cos(j1)+L3*np.cos(j1+j2)+L4*np.cos(j1+j2+j3)

            if A==x and B==y and C==z:
                print(theta1, theta2, theta3, theta4)
            else:
                pass
        '''
       
    return thetalist



class InverseKinematicsNode(object):
    def __init__(self):
        rospy.init_node('inverse_kinematics_node')
        self.desired_pose_sub = rospy.Subscriber('/desired_pose', data_class=Pose, callback=self.desired_pose_callback)
        self.raw_theta_end_pub = rospy.Publisher('/raw_theta_end',data_class=JointStateArray, queue_size = 10)
        
    def desired_pose_callback(self, msg:Pose):
        # raw_theta_end_msg = JointState()
        raw_theta_end_msg = JointState(header = Header(), name=["joint_0","joint_1","joint_2","joint_3"], position=IKinSpace(msg.position.x, msg.position.y, msg.position.z),velocity=[],effort=[])
        # raw_theta_end_msg = IKinSpace(msg.position.x, msg.position.y, msg.position.z)
        # print(msg.position.x, msg.position.y, msg.position.z)
        # print(raw_theta_end_msg)

        # N = len(raw_theta_end_msg.position)

        # for i in range(N):
        # raw_theta_end_msg = JointState(header = Header(), name=["joint_0","joint_1","joint_2","joint_3"], position=raw_theta_end_msg[i],velocity=[],effort=[])
        self.raw_theta_end_pub.publish(raw_theta_end_msg)

    def run(self):
        rospy.sleep(2)
        rospy.spin()

if __name__ == "__main__":
    try:
        ik_node = InverseKinematicsNode()
        ik_node.run()
    except rospy.ROSInterruptException as e:
        pass
