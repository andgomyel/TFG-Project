#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from visualization_msgs.msg import MarkerArray, Marker
from PyQt5.QtCore import QObject, pyqtSignal
from tf.transformations import euler_from_quaternion


# La clase se ha definido con herencia de QObject para poder llevarla a un thread aparte del principal
class camera(QObject):

  image_signal = pyqtSignal()
  update_signal = pyqtSignal()

  FOV = 77.0  # FOV de la cámara del tb3 modelado en turtlebot3_waffle.gazebo.xacro

  def __init__(self, id):
    super().__init__()
    
    self.ID = id

    self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)

    self.original_img_sub = rospy.Subscriber("/tb3_"+str(id)+"/camera/rgb/image_raw", Image, self.orig_callback)
    self.yolo_img_sub = rospy.Subscriber("/tb3_"+str(id)+"/darknet_ros/detection_image", Image, self.yolo_callback)
    
    self.n_objects_sub = rospy.Subscriber("/tb3_"+str(id)+"/darknet_ros/found_object", ObjectCount, self.n_obj_callback)
    self.detection_sub = rospy.Subscriber("/tb3_"+str(id)+"/darknet_ros/bounding_boxes", BoundingBoxes, self.detection_callback)
    
    self.listener = tf.TransformListener()

    self.original_img = None
    self.yolo_img = None

    self.obj_count = 0
    self.B_boxes = []
    self.detecting = False
    self.detected_objects = None

    self.markers = MarkerArray()
    self.marker_id = 0

    self.new_object = False

  def getCurrentPos(self):
        pos,rotation = self.listener.lookupTransform('map', "tb3_"+str(self.ID)+"/base_link",rospy.Time(0))
        position = round(pos[0],3), round(pos[1],3) # Tuple
        return [position, rotation]

  def n_obj_callback(self,data):
    self.obj_count = data.count

  def orig_callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image

    self.original_img = cv2.resize(image, (640,360)) 
    
    # self.yolo_img = self.yolo_detector.run_pipeline(self.img)

    self.image_signal.emit()

  def yolo_callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    image = cv_image

    self.yolo_img = cv2.resize(image, (640,360)) 

    self.process_detected()

  def detection_callback(self,data):

    self.detected_objects = data

    if self.obj_count != 0:
      for BB in self.detected_objects.bounding_boxes:
        if BB.Class == "suitcase" or BB.Class == "backpack":
          self.new_object = True
          self.detecting = True

        else:
          self.detecting = False

    else:
      self.detecting = False



  def process_detected(self):

    if self.detecting and self.obj_count == 0 and self.new_object and self.detected_objects.bounding_boxes is not None:  # Si venimos de detectar un objeto pero ya no
      self.detecting = False
      for BB in self.detected_objects.bounding_boxes:
        if BB.Class == "suitcase" or BB.Class == "backpack" and self.new_object:
          self.new_object = False
          self.process_frame(BB)
 
          self.marker_pub.publish(self.markers)
          self.marker_id += 1
          self.update_signal.emit()



  def process_frame(self,BB):
    x0 = BB.xmin
    x1 = BB.xmax

    
    centroidX = x0 + (x1-x0)/2
  
    offset_orientation = ((centroidX-960)/960) * (self.FOV/2)
    offset_orientation = np.deg2rad(-offset_orientation)

    pos,rot = self.getCurrentPos()
    R,P,Y = euler_from_quaternion(rot)


    if Y > 0:
      if Y > 90:
        Y = Y-90
        # print("yaw=" + str(Y) + ",off = " +str(offset_orientation)+", angle= " + str(Y+offset_orientation))
        dx = np.sin(Y+offset_orientation)
        dy = np.cos(Y+offset_orientation)
      else:
        # print("yaw=" + str(Y) + ",off = " +str(offset_orientation)+", angle= " + str(Y+offset_orientation))
        dx = np.cos(Y+offset_orientation)
        dy = np.sin(Y+offset_orientation)
    else:
      if Y < -90:
        Y = Y+90
        # print("yaw=" + str(Y) + ",off = " +str(offset_orientation)+", angle= " + str(Y+offset_orientation))
        dx = np.sin(Y+offset_orientation)
        dy = np.cos(Y+offset_orientation)
      else:
        # print("yaw=" + str(Y) + ",off = " +str(offset_orientation)+", angle= " + str(Y+offset_orientation))
        dx = np.cos(Y+offset_orientation)
        dy = np.sin(Y+offset_orientation)

    x_marker = pos[0] + dx
    y_marker = pos[1] + dy

    self.add_marker(x_marker, y_marker)


  def add_marker(self,x,y):
    marker = Marker()
    marker.header.frame_id = "tb3_"+ str(self.ID) + "/map"
    marker.header.stamp = rospy.Time.now()

    marker.type = 3
    marker.id = self.marker_id

    marker.scale.x = 0.18
    marker.scale.y = 0.18
    marker.scale.z = 0.18
    
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    self.markers.markers.append(marker)
