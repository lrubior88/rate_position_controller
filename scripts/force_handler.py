#! /usr/bin/env python

import rospy
# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent
from geometry_msgs.msg import Vector3, Point, PoseStamped, Quaternion, Wrench, Transform, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
# State Machine
import smach
import smach_ros
from smach import CBState
# Math
from math import pi, exp, sin, sqrt
import numpy as np
import tf.transformations as tr
# Quaternions tools
import PyKDL
import time


class Force_handler:

  def __init__(self):
    # Read all the parameters from the parameter server
    # Topics to interact
    
    
    master_name = self.read_parameter('~master_name', 'phantom')
    slave_name = self.read_parameter('~slave_name', 'grips')

    self.master_state_topic = '/%s/state' % master_name
    self.feedback_topic = '/%s/force_feedback' % master_name
    self.sm_control_topic = '/sm_control'
    self.ext_forces_topic = '/%s/external_forces' % slave_name
    
    # Force feedback parameters
    self.k_center = self.read_parameter('~k_center', 0.1)
    self.b_center = self.read_parameter('~b_center', 0.003)
    self.k_rate = self.read_parameter('~k_rate', 0.05)
    self.b_rate = self.read_parameter('~b_rate', 0.003)
    
    
    # Position parameters
    #~ self.hysteresis = self.read_parameter('~hysteresis', 3.0)
    #~ self.pivot_dist = self.read_parameter('~pivot_dist', 5.0)
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    #~ self.position_ratio = self.read_parameter('~position_ratio', 250)
    #~ self.axes_rotation = self.read_parameter('~axes_rotation', [0, 0, 0])
    #~ self.angle_rotation = self.read_parameter('~angle_rotation', 1.570796)
    self.position_axes = [0, 1, 2]
    self.position_sign = np.array([1.0, 1.0, 1.0])
    self.axes_mapping = self.read_parameter('~axes_mapping', ['x', 'y' ,'z'])
#~ 
    #~ rospy.logwarn('axes_mapping[0] -> %s' % self.axes_mapping[0])
    #~ rospy.logwarn('axes_mapping[1] -> %s' % self.axes_mapping[1])
    #~ rospy.logwarn('axes_mapping[2] -> %s' % self.axes_mapping[2])

    if len(self.axes_mapping) != 3:
      rospy.logwarn('The invalid number of values in [axes_mapping]. Received 3, expected %d' % len(self.axes_mapping))
    for i, axis in enumerate(self.axes_mapping):
      axis = axis.lower()
      if '-' == axis[0]:
        axis = axis[1:]
        self.position_sign[i] = -1.0
      if axis not in ('x','y','z'):
        rospy.logwarn('Invalid axis %s given in [axes_mapping]' % axis)
      self.position_axes[i] = ['x','y','z'].index(axis)
    
    # Vibration parameters
    self.vib_a = self.read_parameter('~vibration/a', 2.0)             # Amplitude (mm)
    self.vib_c = self.read_parameter('~vibration/c', 5.0)             # Damping
    self.vib_freq = self.read_parameter('~vibration/frequency', 30.0) # Frequency (Hz)
    self.vib_time = self.read_parameter('~vibration/duration', 0.3)   # Duration (s)
    self.vib_start_time = 0.0

    # Setup Subscribers/Publishers
    self.feedback_pub = rospy.Publisher(self.feedback_topic, OmniFeedback)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    rospy.Subscriber(self.sm_control_topic, Float64, self.cb_sm_control) 
    rospy.Subscriber(self.ext_forces_topic, OmniFeedback, self.cb_ext_forces)
    # Initial values
    self.ext_forces = np.zeros(3)
    self.force_feedback = np.zeros(3)
    self.sm_control = 1.0
    self.k_prop = 2
    self.master_real_pos = np.zeros(3)
    self.master_pos = np.zeros(3)
    self.center_pos = np.zeros(3)
    self.master_vel = np.zeros(3)
    #Timer for publish forces
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_force)

    rospy.spin()

  def cb_ext_forces(self, msg):
    self.ext_forces = np.array([msg.force.x, msg.force.y, msg.force.z])
    
  def cb_sm_control(self, msg):
      self.sm_control = msg.data

  def cb_master_state(self, msg):
    self.master_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.master_vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    self.master_dir = self.normalize_vector(self.master_vel)
    
  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result    
    
  def publish_force(self, event):
   try:
     self.send_feedback()
   except rospy.exceptions.ROSException:
     pass
   
  def send_feedback(self):

     if (self.sm_control == 1.0):
        self.force_feedback = self.ext_forces
     elif (self.sm_control == 2.0):
        self.force_feedback = (self.k_rate * self.master_pos + self.b_rate * self.master_vel) * -1.0
     elif (self.sm_control == 3.0):
        t = rospy.get_time() - self.vib_start_time
        amplitude = -self.vib_a*exp(-self.vib_c*t)*sin(2*pi*self.vib_freq*t);
        self.force_feedback = amplitude * self.master_dir
     elif (self.sm_control == 4.0):
        self.force_feedback = (self.k_center * self.master_pos + self.b_center * self.master_vel) * -1.0
     else :
        self.force_feedback = np.zeros(3)

     feedback_msg = OmniFeedback()
     #~ force = self.change_force_axes(self.force_feedback)
     force = self.force_feedback
     pos = self.change_axes(self.center_pos)
     feedback_msg.force = Vector3(*force)
     feedback_msg.position = Vector3(*pos)
     self.feedback_pub.publish(feedback_msg) 
    
  def change_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[idx]
    return result

  def change_force_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[i] #~ ??
    return result
    
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


if __name__ == '__main__':
    rospy.init_node('Force_handler', log_level=rospy.WARN)
    loader = Force_handler()
    
