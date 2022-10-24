#!/usr/bin/env python3

import rospy
from camera_node.msg import TagPose
from fiducial_msgs.msg import FiducialArray

class TagNode(object):
    def __init__(self):
        rospy.init_node("camera_tag_node")
        self.tag_state_sub = rospy.Subscriber("/fiducial_vertices", data_class = FiducialArray, callback = self.tag_state_callback)
        self.cube_dict = {}
    def tag_state_callback(self, msg:FiducialArray):
        fiducial = msg.fiducials[0]
        print(fiducial.x0, fiducial.y0)

    def run(self):
        rospy.sleep(2)
        rospy.spin()

if __name__ == "__main__":
    try:
        tag_state_node = TagNode()
        tag_state_node.run()
    except rospy.ROSInterruptException as e:
        pass


