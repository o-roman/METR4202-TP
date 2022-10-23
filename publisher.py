#!/usr/bin/env python3
import random
import numpy as np

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransform
from fiducial_msgs.msg import FiducialTransformArray
def IKinSpace(x,y,z):
    L1 = 100
    L2 = 117.5
    L3 = 95
    L4 = 102

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

def callback(fidTrans)-> JointState:
    
    global pub
    # TODO: Have fun :)
    x = fidTrans.transforms[0].transform.translation.y*1000
    y= -fidTrans.transforms[0].transform.translation.x*1000
    x=x+250
    z = 20
    
    print("x: ",x)
    print("y: ",y)
    print("z: ",z)
    thetalist = IKinSpace(x,y,z)
    
    msg_list = []
    '''''
    msg = JointState()
    
    for thetas in thetalist:
        msg.position = thetas
        msg_list.append(msg.position)
    data = ['', 'name', msg_list,0, 0]

    '''''
    
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['joint0', 'joint1', 'joint2', 'joint3']
    hello_str.position = thetalist
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    print(hello_str)
       
        


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher('raw_theta_end',JointState,queue_size=10)
    rospy.init_node('ik')
    sub = rospy.Subscriber(
      '/fiducial_transforms',
        FiducialTransformArray, 
        callback # Callback function (required)
    )

   
    #rospy.init_node('inverse_kinematics',anonymous=True)
    rate = rospy.Rate(10)
  

    # Initialise node with any node name
    #rospy.init_node('metr4202_w7_prac')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
