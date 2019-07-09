#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SpherePlotter:
  def __init__(self, topic_name):
    self.active = False
    self.goal = []
    
    self.marker = Marker()
    self.marker.header.frame_id = "base_link"
    self.marker.action = self.marker.ADD
    self.marker.color.a = 1.0
    self.marker.pose.orientation.x = 0.0
    self.marker.pose.orientation.y = 0.0
    self.marker.pose.orientation.z = 0.0
    self.marker.pose.orientation.w = 1.0 
    self.marker.pose.position.z = 0.1
    self.marker.points.append(Point())
    self.marker.points.append(Point())
    self.marker.points[0].z = 0.0
    self.marker.points[1].z = 0.0
    
    #goal_topic = '/planar_bot/' + controller_name + '/goal'
    self.subscriber = rospy.Subscriber(topic_name, Float64MultiArray, self.callback)
    self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
  def callback(self, goal_msg):
    self.goal = goal_msg.data;
    self.active = True
    
  def prepareSphere(self, marker_id, radius, x, y, color):
    self.marker.id = marker_id
    self.marker.type = self.marker.SPHERE
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

  def prepareLine(self, marker_id, width, x1, y1, x2, y2, color):
    self.marker.id = marker_id
    self.marker.type = self.marker.LINE_STRIP
    self.marker.scale.x = width
    self.marker.pose.position.x = 0.0
    self.marker.pose.position.y = 0.0
    self.marker.points[0].x = x1
    self.marker.points[0].y = y1
    self.marker.points[1].x = x2
    self.marker.points[1].y = y2
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
    self.prepareSphere(marker_id=0, radius=0.1, x=1.0, y=0.6, color='red')
    # Publish the Marker
    self.publisher.publish(self.marker)
    
    self.prepareSphere(marker_id=1, radius=0.1, x=2.0, y=0.6, color='red')
    # Publish the Marker
    self.publisher.publish(self.marker)
    
    self.prepareLine(marker_id=2, width=0.1, x1=-3.0, y1=-0.4, x2=3.0, y2=-0.4, color='red')
    # Publish the Marker
    self.publisher.publish(self.marker)
    
    if self.active == True:
      self.prepareSphere(marker_id=3, radius=0.1, x=self.goal[0], y=self.goal[1], color='green')
      # Publish the Marker
      self.publisher.publish(self.marker)

if __name__=="__main__":
  if len(sys.argv) < 2:
    args = ''
    for i in range(len(sys.argv)):
      args = args + sys.argv[i] + ' '
    print("Usage: plot_rviz_spheres.py <Goal topic name> ; Received: " + args)
  else:
    rospy.init_node('register', anonymous=True)

    sphere_plotter = SpherePlotter(topic_name=sys.argv[1])

    while not rospy.is_shutdown():
      sphere_plotter.push()
      rospy.sleep(0.2)
