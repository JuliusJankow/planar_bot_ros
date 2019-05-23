#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

rospy.init_node('register')

#markerArray = MarkerArray()

while not rospy.is_shutdown():

   marker = Marker()
   
   radius = 0.1
   marker.header.frame_id = "base_link"
   marker.id = 0
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 2*radius
   marker.scale.y = 2*radius
   marker.scale.z = 2*radius
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 0.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = 1.0
   marker.pose.position.y = 0.6 
   marker.pose.position.z = 0.1 

   # Publish the Marker
   publisher.publish(marker)
   
   marker.id = 1
   marker.pose.position.x = 2.0
   marker.pose.position.y = 0.6 
   marker.pose.position.z = 0.1
   
   # Publish the Marker
   publisher.publish(marker)

   rospy.sleep(0.1)
