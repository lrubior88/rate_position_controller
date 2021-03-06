#! /usr/bin/env python
"""
Notes
-----
Calculations are carried out with numpy.float64 precision.

This Python implementation is not optimized for speed.

Angles are in radians unless specified otherwise.

Quaternions ix+jy+kz+w are represented as [x, y, z, w].
"""
import rospy, os, math
# Messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent
from geometry_msgs.msg import Vector3, Point, PoseStamped, Quaternion, Wrench, Transform, PoseStamped
from visualization_msgs.msg import Marker
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

GREY_BUTTON = 0
WHITE_BUTTON = 1
GRIPPER_RATIO = 10

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

class RatePositionButtonController:
  STATES = ['GO_TO_CENTER_POS', 'POSITION_CONTROL', 'GO_TO_CENTER_RATE', 'RATE_CONTROL', 'RATE_COLLISION']
  def __init__(self):
    # Create a SMACH state machine
    self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    with self.sm:
      # Add states to the state machine
      smach.StateMachine.add('GO_TO_CENTER_POS', CBState(self.go_to_center_pos, cb_args=[self]), 
                             transitions={'lock': 'GO_TO_CENTER_POS', 'succeeded': 'POSITION_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('POSITION_CONTROL', CBState(self.position_control, cb_args=[self]), 
                             transitions={'stay': 'POSITION_CONTROL', 'leave': 'GO_TO_CENTER_RATE', 'aborted': 'aborted'})
      smach.StateMachine.add('GO_TO_CENTER_RATE', CBState(self.go_to_center_rate, cb_args=[self]),
                             transitions={'lock': 'GO_TO_CENTER_RATE', 'succeeded': 'RATE_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_CONTROL', CBState(self.rate_control, cb_args=[self]), 
                             transitions={'stay': 'RATE_CONTROL', 'leave': 'GO_TO_CENTER_POS', 'collision': 'RATE_COLLISION', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_COLLISION', CBState(self.rate_collision, cb_args=[self]),
                             transitions={'succeeded': 'GO_TO_CENTER_RATE', 'aborted': 'aborted'})
    
    # Read all the parameters from the parameter server
    self.pp_gain = float(self.read_parameter('~pp_gain', 0.0))
    # Topics to interact
    master_name = self.read_parameter('~master_name', 'phantom')
    slave_name = self.read_parameter('~slave_name', 'grips')
    self.master_state_topic = '/%s/state' % master_name
    self.feedback_topic = '/%s/force_feedback' % master_name
    self.slave_state_topic = '/%s/state' % slave_name
    self.ik_mc_topic = '/%s/ik_command' % slave_name
    self.gripper_topic = '/%s/GRIP/command' % slave_name
    self.button_topic = '/%s/button' % master_name

    # Force feedback parameters
    self.k_center = self.read_parameter('~k_center', 0.1)
    self.b_center = self.read_parameter('~b_center', 0.003)
    self.k_rate = self.read_parameter('~k_rate', 0.05)
    self.b_rate = self.read_parameter('~b_rate', 0.003)
    
    # Position parameters
    self.hysteresis = self.read_parameter('~hysteresis', 3.0)
    self.pivot_dist = self.read_parameter('~pivot_dist', 5.0)
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.position_ratio = self.read_parameter('~position_ratio', 250)
    self.axes_rotation = self.read_parameter('~axes_rotation', [0, 0, 0])
    self.angle_rotation = self.read_parameter('~angle_rotation', 1.570796)
    self.position_axes = [0, 1, 2]
    self.position_sign = np.array([1.0, 1.0, 1.0])
    self.axes_mapping = self.read_parameter('~axes_mapping', ['x', 'y' ,'z'])
    
    rospy.logwarn('axes_mapping[0] -> %s' % self.axes_mapping[0])
    rospy.logwarn('axes_mapping[1] -> %s' % self.axes_mapping[1])
    rospy.logwarn('axes_mapping[2] -> %s' % self.axes_mapping[2])
    
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
    
    # Rate parameters
    self.rate_pivot = np.zeros(3)
    self.rate_gain = self.read_parameter('~rate_gain', 1.0) 
    
    # Initial values
    self.center_pos = np.array([0, 0, 0])
    self.frame_id = self.read_parameter('~reference_frame', 'world')
    self.colors = TextColors()
    self.gripper_cmd = 0.0
    self.master_pos = None
    self.master_rot = np.array([0, 0, 0, 1])
    self.master_vel = np.zeros(3)
    self.master_dir = np.zeros(3)
    self.slave_pos = None
    self.slave_rot = np.array([0, 0, 0, 1])
    self.timer = None
    self.force_feedback = np.zeros(3)
    #~ self.roll_angle = 0
    #~ self.q0 = PyKDL.Rotation.Quaternion(-math.sqrt(2)/2.0, 0.0, 0.0, math.sqrt(2)/2.0)
    
    # Synch
    self.slave_synch_pos = np.zeros(3)
    self.slave_synch_rot = np.array([0, 0, 0, 1])
    self.master_synch_rot = np.array([0, 0, 0, 1])
    
    # Button
    self.prev_buttons = [0] * 2
    self.buttons = [False] * 2
    self.buttons[WHITE_BUTTON] = True
    
    # Setup Subscribers/Publishers
    #~ self.aux_pose_pub = rospy.Publisher("/aux_pose", PoseStamped)
    self.feedback_pub = rospy.Publisher(self.feedback_topic, OmniFeedback)
    self.ik_mc_pub = rospy.Publisher(self.ik_mc_topic, PoseStamped)
    self.gripper_pub = rospy.Publisher(self.gripper_topic, Float64)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    rospy.Subscriber(self.slave_state_topic, EndpointState, self.cb_slave_state)
    rospy.Subscriber(self.button_topic, OmniButtonEvent, self.buttons_cb)
    
    self.loginfo('Waiting for [%s] and [%s] topics' % (self.master_state_topic, self.slave_state_topic))
    while not rospy.is_shutdown():
      if (self.slave_pos == None) or (self.master_pos == None):
        rospy.sleep(0.01)
      else:
        self.loginfo('Rate position button controller running')
        # Register rospy shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        break
    
    # Make sure the first command sent to the slave is equal to its current position6D
    self.command_pos = np.array(self.slave_pos)
    self.command_rot = np.array(self.slave_rot)
    
    # Start the timer that will publish the ik commands
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_command)
    
    self.loginfo('State machine state: GO_TO_CENTER_POS')
    
  @smach.cb_interface(outcomes=['lock', 'succeeded', 'aborted'])
  def go_to_center_pos(user_data, self):
    if not np.allclose(np.zeros(3), self.master_pos, atol=self.hysteresis):
      self.force_feedback = (self.k_center * self.master_pos + self.b_center * self.master_vel) * -1.0
      return 'lock'
    else:
      self.force_feedback = np.zeros(3)
      self.slave_synch_pos = np.array(self.slave_pos)
      self.slave_synch_rot = np.array(self.slave_rot)
      self.master_synch_rot = np.array(self.master_rot)
      self.command_pos = np.array(self.slave_pos)
      self.command_rot = np.array(self.slave_rot)
      self.loginfo('State machine transitioning: GO_TO_CENTER_POS:succeeded-->POSITION_CONTROL')
      return 'succeeded'
  
  @smach.cb_interface(outcomes=['stay', 'leave', 'aborted'])
  def position_control(user_data, self):
    if self.buttons[WHITE_BUTTON]:
      self.command_pos = self.slave_synch_pos + self.master_pos / self.position_ratio
      self.command_rot = np.array(self.master_rot)
      return 'stay'
    else:
      self.force_feedback = np.zeros(3)
      self.command_pos = np.array(self.slave_pos)
      self.command_rot = np.array(self.slave_rot)
      self.loginfo('State machine transitioning: POSITION_CONTROL:leave-->GO_TO_CENTER_RATE')
      return 'leave'
    
  @smach.cb_interface(outcomes=['lock', 'succeeded', 'aborted'])
  def go_to_center_rate(user_data, self):
    if not np.allclose(np.zeros(3), self.master_pos, atol=self.hysteresis):
      self.force_feedback = (self.k_center * self.master_pos + self.b_center * self.master_vel) * -1.0
      return 'lock'
    else:
      self.force_feedback = np.zeros(3)
      self.slave_synch_pos = np.array(self.slave_pos)
      self.slave_synch_rot = np.array(self.slave_rot)
      self.master_synch_rot = np.array(self.master_rot)
      self.command_pos = np.array(self.slave_pos)
      self.command_rot = np.array(self.slave_rot)
      self.loginfo('State machine transitioning: GO_TO_CENTER_RATE:succeeded-->RATE_CONTROL')
      return 'succeeded'
  
  @smach.cb_interface(outcomes=['stay', 'leave', 'collision', 'aborted'])
  def rate_control(user_data, self):
    if not self.buttons[WHITE_BUTTON]:
      # Send the force feedback to the master
      self.force_feedback = (self.k_rate * self.master_pos + self.b_rate * self.master_vel) * -1.0
      # Send the rate command to the slave
      distance = sqrt(np.sum((self.master_pos - self.rate_pivot) ** 2)) / self.position_ratio * 70
      self.command_pos += (self.rate_gain * distance * self.normalize_vector(self.master_pos)) / self.position_ratio
      self.command_rot = np.array(self.master_rot)
      return 'stay'
    else:
      self.command_pos = np.array(self.slave_pos)
      self.command_rot = np.array(self.slave_rot)
      self.force_feedback = np.zeros(3)
      self.loginfo('State machine transitioning: RATE_CONTROL:leave-->GO_TO_CENTER')
      return 'leave'
    
  @smach.cb_interface(outcomes=['succeeded', 'aborted'])
  def rate_collision(user_data, self):
    return 'succeeded'
  
  def execute(self):
    self.sm.execute()
      
  def shutdown_hook(self):
    # Stop the state machine
    self.sm.request_preempt()
    # Stop the publisher timer
    self.timer.shutdown()   
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
  
  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
    
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
      result[i] = array[idx] * sign[i] #~ ??
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
    
    # Rotate tu use the same axes orientation between master and slave
    real_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    q = tr.quaternion_about_axis(self.angle_rotation, self.axes_rotation)
    self.master_rot = tr.quaternion_multiply(real_rot, q)
    self.master_dir = self.normalize_vector(self.master_vel)
    
    # Discomment for debugging
    #~ aux_msg = PoseStamped()
    #~ aux_msg.header.frame_id = self.frame_id
    #~ aux_msg.header.stamp = rospy.Time.now()
    #~ aux_msg.pose.position = Point(*self.master_pos)
    #~ aux_msg.pose.position.x /= 1000.0;
    #~ aux_msg.pose.position.y /= 1000.0;
    #~ aux_msg.pose.position.z /= 1000.0;
    #~ aux_msg.pose.orientation = Quaternion(*self.master_rot)
    #~ self.aux_pose_pub.publish(aux_msg) 
  
  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    
  def buttons_cb(self, msg):
    button_states = [msg.grey_button, msg.white_button]
    # Check that any button was pressed / released
    for i, previous in enumerate(self.prev_buttons):
      if (previous != button_states[i]) and button_states[i] == 1:
        self.buttons[i] = not self.buttons[i]
    # Open or close the gripper
    if self.buttons[GREY_BUTTON]:   # Close
      self.gripper_cmd = GRIPPER_RATIO
    else:                           # Open
      self.gripper_cmd = -GRIPPER_RATIO
    self.gripper_pub.publish(Float64(self.gripper_cmd))
        
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
  rospy.init_node('rate_position_button_controller', log_level=rospy.WARN)
  try:
    controller = RatePositionButtonController()
    controller.execute()
  except rospy.exceptions.ROSInterruptException:
      pass
