#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class stereo_align:
	def __init__(self):
		rospy.init_node('Stereo_Allignment',anonymous=True)
		self.cross_hair_color = rospy.get_param("~cross_hair_color")
		self.line_width = rospy.get_param("~line_width")
		self.line_length = rospy.get_param("~line_length")
		image_subL = message_filters.Subscriber('left_camera/image_raw', Image)
		image_subR = message_filters.Subscriber('right_camera/image_raw', Image)
		ts = message_filters.ApproximateTimeSynchronizer([image_subL, image_subR], 10, 0.1, allow_headerless=True)
		#ts = message_filters.TimeSynchronizer([image_subL, image_subR], 10)
		ts.registerCallback(self.img_callback)
		self.img_pub=rospy.Publisher("image_out",Image, queue_size=10)
		self.bridge = CvBridge()
		self.get_color()
		self.spin()

	def get_color(self):
		color_dict={'red':(0,0,255),'green':(0,255,0),'blue':(255,0,0)}
		try:
			self.line_color=color_dict[self.cross_hair_color]
		except KeyError as e:
			print(e)
			self.line_color=color_dict['red']
		rospy.loginfo('Set Cross Hair Color to ' +str(self.line_color))

	def img_callback(self,left,right):
		try:
			left_image = self.bridge.imgmsg_to_cv2(left, "bgr8")
			right_image = self.bridge.imgmsg_to_cv2(right, "bgr8")
		except CvBridgeError as e:
			print(e)
		h,w,c=left_image.shape
		center=[int(round(left_image.shape[1]/2)),int(round(left_image.shape[0]/2))]
		cv2.line(left_image,(center[0]-self.line_length,center[1]),(center[0]+self.line_length,center[1]),self.line_color,self.line_width)
		cv2.line(left_image,(center[0],center[1]-self.line_length),(center[0],center[1]+self.line_length),self.line_color,self.line_width)
		cv2.line(right_image,(center[0]-self.line_length,center[1]),(center[0]+self.line_length,center[1]),self.line_color,self.line_width)
		cv2.line(right_image,(center[0],center[1]-self.line_length),(center[0],center[1]+self.line_length),self.line_color,self.line_width)
		if (True): #zoom
			combined=np.empty((h,w*2,c),dtype=np.uint8)
			combined[:,:w,:]=left_image
			combined[:,w:,:]=right_image
		else:
			combined=np.empty((h/2,w,c),dtype=np.uint8)
			combined[:,:w/2,:]=left_image[h/4:3*h/4,w/4:3*w/4]
			combined[:,w/2:,:]=right_image[h/4:3*h/4,w/4:3*w/4]

		vis_img=self.bridge.cv2_to_imgmsg(combined,'bgr8')
		self.img_pub.publish(vis_img)

	def spin(self):
		while not rospy.is_shutdown():
			rospy.spin()

a=stereo_align()
