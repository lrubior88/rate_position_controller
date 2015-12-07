#! /usr/bin/env python

import rospy
# Messages
from visualization_msgs.msg import Marker
from baxter_core_msgs.msg import EndpointState
from std_msgs.msg import Bool
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent
from geometry_msgs.msg import Vector3

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
    master_name = self.read_parameter('~master_name', 'phantom')
    self.ext_forces_topic = '/%s/external_forces' % slave_name
    self.collision_topic = '/%s/collision' % slave_name
    self.slave_state_topic = '/%s/endpoint_state' % slave_name
    self.master_state_topic = '/%s/state' % master_name 
    
    
    # Initial values      
    self.slave_pos = None
    self.master_pos = None
    self.slave_rot = None
    self.master_rot = None
    self.frame_id = self.read_parameter('~reference_frame', 'world')
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.colors = TextColors()
    self.timer = None
    #~ self.is_drawn = False
    self.external_forces = np.zeros(3)
    self.k_prop = 30
          
    # Setup Subscribers/Publishers
    self.ext_forces_pub = rospy.Publisher(self.ext_forces_topic, OmniFeedback)
    self.collision_pub = rospy.Publisher(self.collision_topic, Bool)
    self.vis_pub = rospy.Publisher('/vis_obstacles', Marker)
    rospy.Subscriber(self.slave_state_topic, EndpointState, self.cb_slave_state)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    
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
    rospy.sleep(3.0)
    self.draw_obstacle()
    
    # Start the timer that will publish the ik commands
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_collision)
    rospy.spin()
  
  
  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    #~ dif_state_x= abs(msg.pose.position.x-self.state_before_x)
    #~ if (dif_state_x < 0.01):
      #~ state_msg.pose.position.x = self.state_before_x
    #~ else:
      #~ self.state_before_x = state_msg.pose.position.x
#~ 
    #~ dif_state_y= abs(state_msg.pose.position.y-self.state_before_y)
    #~ if (dif_state_y < 0.01):
      #~ state_msg.pose.position.y = self.state_before_y
    #~ else:
      #~ self.state_before_y = state_msg.pose.position.y
 #~ 
    #~ dif_state_z= abs(state_msg.pose.position.z-self.state_before_z)
    #~ if (dif_state_z < 0.01):
      #~ state_msg.pose.position.z = self.state_before_z
    #~ else:
      #~ self.state_before_z = state_msg.pose.position.z
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])    
  def cb_master_state(self, msg):
    self.master_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.master_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])   
      
  def shutdown_hook(self):
    # Stop the publisher timer
    self.timer.shutdown()
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
        rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    
  def inside_obstacle(self, point):
    if ((point[0]>0.5)):
        self.external_forces[0] = self.k_prop * (0.5 - point[0])
        return True
    else:
        self.external_forces = np.zeros(3)
        return False
        
  def send_forces(self):
    feedback_msg = OmniFeedback()
    feedback_msg.force = Vector3(*self.external_forces)
    feedback_msg.position = Vector3(*self.slave_pos)
    self.ext_forces_pub.publish(feedback_msg) 
          
  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
        
  def publish_collision(self, event):
    #~ self.draw_obstacle()
    try:
      self.collision_pub.publish(self.inside_obstacle(self.slave_pos))
      self.send_forces()
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

