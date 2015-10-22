#! /usr/bin/env python

import rospy
# Messages
from visualization_msgs.msg import Marker
from baxter_core_msgs.msg import EndpointState
from std_msgs.msg import Bool
#Math
import numpy as np

class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''
    
    
class VirtualObstacles:
    
  def __init__(self):
      
    # Topics to interact
    slave_name = self.read_parameter('~slave_name', 'grips')
    self.collision_topic = '/%s/collision' % slave_name
    self.slave_state_topic = '/%s/endpoint_state' % slave_name
    #~ self.feedback_topic = '/%s/force_feedback' % master_name
    
    # Initial values      
    self.slave_pos = None
    self.frame_id = self.read_parameter('~reference_frame', 'world')
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.colors = TextColors()
    self.timer = None
    self.is_drawn = False
          
    # Setup Subscribers/Publishers
    self.collision_pub = rospy.Publisher(self.collision_topic, Bool)
    self.vis_pub = rospy.Publisher('/vis_obstacles', Marker)
    rospy.Subscriber(self.slave_state_topic, EndpointState, self.cb_slave_state)
    
    self.loginfo('Waiting for [%s] topic' % (self.slave_state_topic))
    while not rospy.is_shutdown():
      if (self.slave_pos == None):
        rospy.sleep(0.01)
      else:
        self.loginfo('Virtual Objects running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break
        
    # Draw obstacles
    rospy.sleep(0.5)
    self.draw_obstacle()
    
    # Start the timer that will publish the ik commands
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_collision)
    rospy.spin()
  
  
  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])    
      
  def shutdown_hook(self):
    # Stop the publisher timer
    self.timer.shutdown()
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
        rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    
  def inside_obstacle(self, point):
    if ((point[0]>0.5) and (point[0]<1)):
        return True
    else:
        return False
          
  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
        
  def publish_collision(self, event):
    try:
      self.collision_pub.publish(self.inside_obstacle(self.slave_pos))
    except rospy.exceptions.ROSException:
      pass
    
        
  def draw_obstacle(self):
    self.loginfo('Drawing obstacles')
    marker = Marker()
    marker.header.frame_id = self.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.id = 1;
    marker.type = marker.CUBE
    marker.ns = 'obstacle'
    marker.action = marker.ADD
    marker.pose.position.x = 0.6
    marker.pose.position.y = 0
    marker.pose.position.z = 2
    marker.scale.x = 0.2
    marker.scale.y = 4
    marker.scale.z = 4
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    #~ Publish
    self.vis_pub.publish(marker)
  

if __name__ == '__main__':
    rospy.init_node('virtual_obstacles', log_level=rospy.WARN)
    loader = VirtualObstacles()

