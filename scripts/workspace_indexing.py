#! /usr/bin/env python
"""
Notes
-----
Calculations are carried out with numpy.float64 precision.

This Python implementation is not optimized for speed.

Angles are in radians unless specified otherwise.

Quaternions ix+jy+kz+w are represented as [x, y, z, w].
"""
import rospy
# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent
from geometry_msgs.msg import Vector3, Quaternion, Transform, PoseStamped, Point, Wrench
from visualization_msgs.msg import Marker
# State Machine
import smach
import smach_ros
from smach import CBState
# Math
from math import pi, exp, sin, sqrt
import numpy as np
import tf.transformations as tr

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

class RatePositionController:
  STATES = ['GO_TO_CENTER', 'POSITION_CONTROL', 'VIBRATORY_PHASE', 'RATE_CONTROL', 'RATE_COLLISION']
  def __init__(self):
    # Create a SMACH state machine
    self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with self.sm:
      # Add states to the state machine
      smach.StateMachine.add('GO_TO_CENTER', CBState(self.go_to_center, cb_args=[self]), 
                             transitions={'lock':'GO_TO_CENTER', 'succeeded':'POSITION_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('POSITION_CONTROL', CBState(self.position_control, cb_args=[self]), 
                             transitions={'stay':'POSITION_CONTROL', 'leave':'GO_TO_CENTER', 'aborted': 'aborted'})
    
    # Read all the parameters from the parameter server
    self.pp_gain = float(self.read_parameter('~pp_gain', 0.0))
    # Topics to interact
    master_name = self.read_parameter('~master_name', 'phantom')
    slave_name = self.read_parameter('~slave_name', 'grips')
    self.master_state_topic = '/%s/state' % master_name
    self.feedback_topic = '/%s/force_feedback' % master_name
    self.slave_state_topic = '/%s/state' % slave_name
    self.ik_mc_topic = '/%s/raw_ik_command' % slave_name
    # Indexing parameters
    self.locked = False
    self.center_pos = np.array([0, 0, 0])
    # Position parameters
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.position_ratio = self.read_parameter('~position_ratio', 250)
    self.position_axes = [0, 1, 2]
    self.position_sign = np.array([1.0, 1.0, 1.0])
    self.axes_mapping = self.read_parameter('~axes_mapping', ['x', 'y' ,'z'])
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
    # Initial values
    self.frame_id = self.read_parameter('~frame_id', 'world')
    self.colors = TextColors()
    self.master_pos = None
    self.master_rot = np.array([0, 0, 0, 1])
    self.master_vel = np.zeros(3)
    self.master_dir = np.zeros(3)
    self.slave_pos = None
    self.command_pos = None
    self.slave_rot = np.array([0, 0, 0, 1])
    self.timer = None
    self.force_feedback = np.zeros(3)
    self.pos_force_feedback = np.zeros(3)
    # Synch
    self.slave_synch_pos = np.zeros(3)
    self.slave_synch_rot = np.array([0, 0, 0, 1])
    
    # Setup Subscribers/Publishers
    self.feedback_pub = rospy.Publisher(self.feedback_topic, OmniFeedback)
    self.ik_mc_pub = rospy.Publisher(self.ik_mc_topic, PoseStamped)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    rospy.Subscriber(self.slave_state_topic, EndpointState, self.cb_slave_state)
    rospy.Subscriber('/takktile/force_feedback', Wrench, self.feedback_cb)
    
    self.loginfo('Waiting for [%s] and [%s] topics' % (self.master_state_topic, self.slave_state_topic))
    while not rospy.is_shutdown():
      if (self.slave_pos == None) or (self.master_pos == None):
        rospy.sleep(0.01)
      else:
        self.loginfo('Rate position controller running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break
    
    # Make sure the first command sent to the slave is equal to its current position6D
    self.command_pos = np.array(self.slave_pos)
    self.command_rot = np.array(self.slave_rot)
    
    # Start the timer that will publish the ik commands
    self.command_timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_command)
    self.rate = rospy.Rate(self.publish_frequency)
    
    self.loginfo('State machine state: GO_TO_CENTER')
    
  @smach.cb_interface(outcomes=['lock', 'succeeded', 'aborted'])
  def go_to_center(user_data, self):
    if not self.locked:
      try:
        self.rate.sleep()
      except rospy.ROSInterruptException:
        pass
      return 'lock'
    else:
      self.force_feedback = np.zeros(3)
      self.slave_synch_pos = np.array(self.slave_pos)
      self.command_pos = np.array(self.slave_pos)
      self.center_pos = np.array(self.master_real_pos)
      self.loginfo('State machine transitioning: GO_TO_CENTER:succeeded-->POSITION_CONTROL')
      return 'succeeded'
  
  @smach.cb_interface(outcomes=['stay', 'leave', 'aborted'])
  def position_control(user_data, self):
    if self.locked:
      self.command_pos = self.slave_synch_pos + self.master_pos / self.position_ratio
      self.force_feedback = self.pos_force_feedback
      try:
        self.rate.sleep()
      except rospy.ROSInterruptException:
        pass
      return 'stay'
    else:
      self.force_feedback = np.zeros(3)
      self.command_pos = np.array(self.slave_pos)
      self.loginfo('State machine transitioning: POSITION_CONTROL:leave-->RATE_CONTROL')
      return 'leave'
  
  def execute(self):
    self.sm.execute()
      
  def shutdown_hook(self):
    # Stop timers
    self.command_timer.shutdown()
    # Stop the state machine
    self.sm.request_preempt()
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
  
  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
    #~ rospy.loginfo(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
    
  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result
  
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
      result[i] = array[idx] * sign[i]
    return result
  
  def send_feedback(self):
    feedback_msg = OmniFeedback()
    force = self.change_force_axes(self.force_feedback)
    pos = self.change_axes(self.center_pos)
    feedback_msg.force = Vector3(*force)
    feedback_msg.position = Vector3(*pos)
    self.feedback_pub.publish(feedback_msg)
  
  # DO NOT print to the console within this function
  def cb_master_state(self, msg):
    self.master_real_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) - self.center_pos
    vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    self.master_pos = self.change_axes(pos)
    self.master_vel = self.change_axes(vel)
    self.master_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    self.master_dir = self.normalize_vector(self.master_vel)
    self.locked = msg.locked
  
  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    # Implement here the position-position control
    if self.master_pos != None and self.command_pos != None:
      self.pos_force_feedback = self.pp_gain * (self.slave_pos - self.command_pos)
  
  def feedback_cb(self, msg):
    self.pos_force_feedback = np.array([msg.force.x, msg.force.y, msg.force.z])
  
  def publish_command(self, event):
    position, orientation = self.command_pos, self.command_rot
    ik_mc_msg = PoseStamped()
    ik_mc_msg.header.frame_id = self.frame_id
    ik_mc_msg.header.stamp = rospy.Time.now()
    ik_mc_msg.pose.position = Point(*position)
    ik_mc_msg.pose.orientation = Quaternion(*orientation)
    try:
      self.ik_mc_pub.publish(ik_mc_msg)
      self.send_feedback()
    except rospy.exceptions.ROSException:
      pass
    
if __name__ == '__main__':
  rospy.init_node('workspace_indexing', log_level=rospy.WARN)
  try:
    controller = RatePositionController()
    controller.execute()
  except rospy.ROSInterruptException:
      pass
