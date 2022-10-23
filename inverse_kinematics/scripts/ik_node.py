#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from inverse_kinematics import IKinSpace 

class InverseKinematicsNode:
    def __init__(self):
        rospy.init_node('inverse_kinematics_node')
        self.desired_pose_sub = rospy.Subscriber('/desired_pose', data_class=Pose, callback=self.desired_pose_callback)
        self.raw_theta_end_pub = rospy.Publisher('/raw_theta_end',data_class=JointState, queue_size = 10)
        
    def desired_pose_callback(self, msg:Pose):
        position = msg.position
        raw_theta_end_msg = JointState(name=["joint_0","joint_1","joint_2","joint_3"], position=IKinSpace(position.x, position.y, position.z),velocity=[],effort=[])
        print(msg.position.x, msg.position.y, msg.position.z)
        print(raw_theta_end_msg)

        self.raw_theta_end_pub.publish(raw_theta_end_msg)

    def run(self):
        rospy.sleep(2)
        rospy.spin()

if __name__ == "__main__":
    try:
        ik_node = InverseKinematicsNode()
        ik_node.run()
    except ROSInterruptException as e:
        pass