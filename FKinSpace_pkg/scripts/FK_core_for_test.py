#!/usr/bin/env python3
from re import T
import numpy as np
from std_msgs.msg import (Float32, Header)
import rospy
from modern_robotics import TransToRp
#from sensor_msgs 

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransform
from fiducial_msgs.msg import FiducialTransformArray



def FKs(thetalist):
    M = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,422.5],
                  [0,0,0,1]])
    Slist = np.array([[0,0,1,0,0,0],
                      [0,-1,0,100,0,0],
                      [0,-1,0,217.5,0,0],
                      [0,-1,0,312.5,0,0]])
    I = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    
    theta0 = thetalist[0]
    theta1 = thetalist[1]
    theta2 = thetalist[2]
    theta3 = thetalist[3]
    
    w1 = Slist[0,0:3]
    w2 = Slist[1,0:3]
    w3 = Slist[2,0:3]
    w4 = Slist[3,0:3]
    v1 = Slist[0,3:6]
    v2 = Slist[1,3:6]
    v3 = Slist[2,3:6]
    v4 = Slist[3,3:6]
    
    w1_so3 = np.array([[0,-w1[2],w1[1]],
                       [w1[2],0,-w1[0]],
                       [-w1[1],w1[0],0]])
    w2_so3 = np.array([[0,-w2[2],w2[1]],
                       [w2[2],0,-w2[0]],
                       [-w2[1],w2[0],0]])
    w3_so3 = np.array([[0,-w3[2],w3[1]],
                       [w3[2],0,-w3[0]],
                       [-w3[1],w3[0],0]])
    w4_so3 = np.array([[0,-w4[2],w4[1]],
                       [w4[2],0,-w4[0]],
                       [-w4[1],w4[0],0]])
    # v1_vec = np.array([[v1[0]],
    #                    [v1[1]],
    #                    [v1[2]]])
    # v2_vec = np.array([[v2[0]],
    #                    [v2[1]],
    #                    [v2[2]]])
    # v3_vec = np.array([[v3[0]],
    #                    [v3[1]],
    #                    [v3[2]]])
    # v4_vec = np.array([[v4[0]],
    #                    [v4[1]],
    #                    [v4[2]]])

    R1 = I + np.sin(theta0) * w1_so3 + (1-np.cos(theta0)) * (np.dot(w1_so3,w1_so3))
    R2 = I + np.sin(theta1) * w2_so3 + (1-np.cos(theta1)) * (np.dot(w2_so3,w2_so3))
    R3 = I + np.sin(theta2) * w3_so3 + (1-np.cos(theta2)) * (np.dot(w3_so3,w3_so3))
    R4 = I + np.sin(theta3) * w4_so3 + (1-np.cos(theta3)) * (np.dot(w4_so3,w4_so3))
    G1 = I * theta0 + (1-np.cos(theta0)) * w1_so3 + (theta0 - np.sin(theta0)) * (np.dot(w1_so3,w1_so3))
    G2 = I * theta1 + (1-np.cos(theta1)) * w2_so3 + (theta1 - np.sin(theta1)) * (np.dot(w2_so3,w2_so3))
    G3 = I * theta2 + (1-np.cos(theta2)) * w3_so3 + (theta2 - np.sin(theta2)) * (np.dot(w3_so3,w3_so3))
    G4 = I * theta3 + (1-np.cos(theta3)) * w4_so3 + (theta3 - np.sin(theta3)) * (np.dot(w4_so3,w4_so3))
    G1v = np.dot(G1,v1)
    G2v = np.dot(G2,v2)
    G3v = np.dot(G3,v3)
    G4v = np.dot(G4,v4)
    # G1v = np.array(np.dot(G1,v1_vec)).T
    # G2v = np.array(np.dot(G2,v2_vec)).T
    # G3v = np.array(np.dot(G3,v3_vec)).T
    # G4v = np.array(np.dot(G4,v4_vec)).T

    T01 = np.insert(np.insert(R1,3,values=G1v,axis=1),3,values=np.array([0,0,0,1]),axis=0)
    T12 = np.insert(np.insert(R2,3,values=G2v,axis=1),3,values=np.array([0,0,0,1]),axis=0)
    T23 = np.insert(np.insert(R3,3,values=G3v,axis=1),3,values=np.array([0,0,0,1]),axis=0)
    T34 = np.insert(np.insert(R4,3,values=G4v,axis=1),3,values=np.array([0,0,0,1]),axis=0)
    Tsb = np.dot(T01,np.dot(T12,np.dot(T23,np.dot(T34,M))))

    
    return Tsb

def callback(msg):

    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['Tsb']
    Tsb = FKs(msg.position)
    hello_str.position = TransToRp(Tsb)[1]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    print(hello_str)

def main():

    global pub
    # Create publisher
    pub = rospy.Publisher('end_effector_pose',Pose,queue_size=10)
    rospy.init_node('fk')
    sub = rospy.Subscriber(
      '/desired_joint_states',
       JointState, callback # Callback function (required)
    )

   
    #rospy.init_node('inverse_kinematics',anonymous=True)
    rate = rospy.Rate(10)
  

    # Initialise node with any node name
    #rospy.init_node('metr4202_w7_prac')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()
