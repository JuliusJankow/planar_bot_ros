#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker

class SpherePlotter:
  def __init__(self, controller_name):
    self.active = False
    self.goal = []
    
    self.marker = Marker()
    self.marker.header.frame_id = "base_link"
    self.marker.type = self.marker.SPHERE
    self.marker.action = self.marker.ADD
    self.marker.color.a = 1.0
    self.marker.pose.orientation.w = 1.0 
    self.marker.pose.position.z = 0.1
    
    goal_topic = '/planar_bot/' + controller_name + '/goal'
    self.subscriber = rospy.Subscriber(goal_topic, Float64MultiArray, self.callback)
    self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
  def callback(self, goal_msg):
    self.goal = goal_msg.data;
    self.active = True
    
  def prepareMarker(self, marker_id, radius, x, y, color):
    self.marker.id = marker_id
    self.marker.scale.x = 2*radius
    self.marker.scale.y = 2*radius
    self.marker.scale.z = 2*radius
    self.marker.pose.position.x = x
    self.marker.pose.position.y = y
    if color == 'red':
      self.marker.color.r = 1.0
      self.marker.color.g = 0.0
      self.marker.color.b = 0.0
    elif color == 'green':
      self.marker.color.r = 0.0
      self.marker.color.g = 1.0
      self.marker.color.b = 0.0
    elif color == 'blue':
      self.marker.color.r = 0.0
      self.marker.color.g = 0.0
      self.marker.color.b = 1.0
    
  def push(self):
    self.prepareMarker(marker_id=0, radius=0.1, x=1.0, y=0.6, color='red')
    # Publish the Marker
    self.publisher.publish(self.marker)
    
    self.prepareMarker(marker_id=1, radius=0.1, x=2.0, y=0.6, color='red')
    # Publish the Marker
    self.publisher.publish(self.marker)
    
    if self.active == True:
      self.prepareMarker(marker_id=2, radius=0.1, x=self.goal[0], y=self.goal[1], color='green')
      # Publish the Marker
      self.publisher.publish(self.marker)

if __name__=="__main__":
  if len(sys.argv) < 2:
    args = ''
    for i in range(len(sys.argv)):
      args = args + sys.argv[i] + ' '
    print("Usage: plot_rviz_spheres.py <controller_name> ; Received: " + args)
  else:
    rospy.init_node('register', anonymous=True)

    sphere_plotter = SpherePlotter(controller_name=sys.argv[1])

    while not rospy.is_shutdown():
      sphere_plotter.push()
      rospy.sleep(0.2)
