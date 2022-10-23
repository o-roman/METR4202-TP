#!/usr/bin/env python

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
    #self.fiducial_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.tag_callback)
    self.fiducial_vertices_sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.vertices_callback)
    #self.geometry_sub = rospy.Subscriber()
    self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)
    self.tag_pub = rospy.Publisher("/tag_pose", TagPose, queue_size=10)

#TODO: Tag message structure:
# point:tag_position
# id:tag_id
# string:colour

  def vertices_callback(self, data):
    global cube
    cube.id = data.fiducials[0].id
    x_0 = data.fiducuals[0].x0
    y_0 = data.fiducuals[0].y0
    cube.corner = (x_0, y_0)



  #def tag_callback(self,data):
   # tag_point = data.transforms[0].transform.translation
    #FIXME: Point(tag_point) might break if it does do Point(x=tag_point.x, ...)
    #msg = TagPose(tag_id = data.transforms[0].fiducial_id, tag_position = Point(tag_point))

  def img_callback(self,data):
    
  
    global img
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    bgr = img[cube.corner[0], cube.corner[1], :]
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
            cv2.waitKey(1)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()