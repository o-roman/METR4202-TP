#!/usr/bin/env python3

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
from fiducial_msgs.msgs import FiducialTransform, FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Vector3

img = None

class CameraViewer:

  def __init__(self, serial):
    self.bridge = CvBridge()
    self.serial = serial
    self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self.img_callback)
    self.fiducial_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.tag_callback)
    self.fiducial_vertices_sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.vertices_callback)
    self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)
    self.tag_pub = rospy.Publisher("/tag_pose", TagPose, queue_size=10)

#TODO: Tag message structure:
# point:tag_position
# id:tag_id
# string:colour

  def vertices_callback(self, data):
    global corner
    corner.id = data.fiducials[0].id
    corner_x_0 = data.fiducuals[0].x0
    corner_y_0 = data.fiducuals[0].y0
    corner = [corner_x_0, corner_y_0]

  def tag_callback(self, data):
    global tag
    tag_x = data.transform[0].translation.x
    tag_y = data.transform[0].translation.y
    tag_z = data.transform[0].translation.z
    tag = Point(x=tag_x, y=tag_y, z=tag_z)


  def img_callback(self,data):
    
    global img
    global color
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    bgr = img[round(corner[0] - 2.5, 1), round(corner[1], 1), :]
    color = ColorRGBA()
    color.r = bgr[2]
    color.g = bgr[1]
    color.b = bgr[0]
    viewer.color_pub.publish(color)


if __name__ == '__main__':  
  rospy.init_node('image_node', anonymous=True)
  viewer = CameraViewer('32702051')
  try:
    while not rospy.is_shutdown():
        if img is not None:
            cv2.imshow("Image", img)
            msg = TagPose(header=Header(stamp=rospy.Time.now()))
            msg.id = corner.id
            msg.color = color
            msg.position = tag
            tag_pub.publish(msg)
            cv2.waitKey(1)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()