#!/usr/bin/env python

import rospy

from msg import TagPose

def tag_pose_pub():
    #topic: tag_pose
    pub = rospy.Publisher("tag_pose", TagPose, queue_size = 10)

try:
    point = 1
    msg = TagPose(id = 3, position = point, color = 'green')
    tag_pose_pub(msg)
except rospy.ROSInterruptException:
    pass


