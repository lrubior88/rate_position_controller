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

GREY_BUTTON = 0
WHITE_BUTTON = 1


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
                             transitions={'lock': 'GO_TO_CENTER', 'succeeded_pos': 'POSITION_CONTROL','succeeded_rate': 'RATE_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('POSITION_CONTROL', CBState(self.position_control, cb_args=[self]),
                             transitions={'stay': 'POSITION_CONTROL', 'leave': 'GO_TO_CENTER', 'aborted': 'aborted'})
      #~ smach.StateMachine.add('VIBRATORY_PHASE', CBState(self.vibratory_phase, cb_args=[self]),
                             #~ transitions={'vibrate': 'VIBRATORY_PHASE', 'succeeded': 'RATE_CONTROL', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_CONTROL', CBState(self.rate_control, cb_args=[self]),
                             transitions={'stay': 'RATE_CONTROL', 'leave': 'GO_TO_CENTER', 'collision': 'RATE_COLLISION', 'aborted': 'aborted'})
      smach.StateMachine.add('RATE_COLLISION', CBState(self.rate_collision, cb_args=[self]),
                             transitions={'succeeded': 'GO_TO_CENTER', 'aborted': 'aborted'})

    # Read all the parameters from the parameter server
    # Topics to interact
    master_name = self.read_parameter('~master_name', 'phantom')
    slave_name = self.read_parameter('~slave_name', 'grips')
    self.master_state_topic = '/%s/state' % master_name
    #~ self.feedback_topic = '/%s/force_feedback' % master_name ##
    self.slave_state_topic = '/%s/state' % slave_name
    self.ik_mc_topic = '/%s/ik_command' % slave_name
    self.gripper_topic = '/%s/GRIP/command' % slave_name
    self.button_topic = '/%s/button' % master_name
    self.slave_collision_topic = '/%s/collision' % slave_name
    self.sm_control_topic = '/sm_control'
    self.reset_signal = '/reset'
    #~ self.ext_forces_topic = '/%s/external_forces' % slave_name ##

    # Workspace definition
    self.units = self.read_parameter('~units', 'mm')
    width = self.read_parameter('~workspace/width', 140.0)
    height = self.read_parameter('~workspace/height', 100.0)
    depth = self.read_parameter('~workspace/depth', 55.0)
    self.center_pos = self.read_parameter('~workspace/center', [0, 0 ,0])
    self.workspace = np.array([width, depth, height])
    self.hysteresis = self.read_parameter('~hysteresis', 3.0)
    self.pivot_dist = self.read_parameter('~pivot_dist', 5.0)

    # Force feedback parameters
    self.k_center = self.read_parameter('~k_center', 0.1)
    self.b_center = self.read_parameter('~b_center', 0.003)
    self.k_rate = self.read_parameter('~k_rate', 0.05)
    self.b_rate = self.read_parameter('~b_rate', 0.003)

    # Position parameters
    self.hysteresis = self.read_parameter('~hysteresis', 1.0)
    self.pivot_dist = self.read_parameter('~pivot_dist', 5.0)
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    self.position_ratio = self.read_parameter('~position_ratio', 250)
    self.axes_rotation_1 = self.read_parameter('~axes_rotation_1', [0, 0, 0])
    self.angle_rotation_1 = self.read_parameter('~angle_rotation_1',0.0)
    self.axes_rotation_2 = self.read_parameter('~axes_rotation_2', [0, 0, 0])
    self.angle_rotation_2 = self.read_parameter('~angle_rotation_2', 0.0)
    self.axes_rotation_3 = self.read_parameter('~axes_rotation_3', [0, 0, 0])
    self.angle_rotation_3 = self.read_parameter('~angle_rotation_3', 0.0)
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
    self.slave_collision = False
    self.timer = None
    #~ self.force_feedback = np.zeros(3) ##
    #~ self.ext_forces = np.zeros(3) ##
    self.gripper_value = 0.5
    self.button_states =np.array([0, 0])
    self.time_counter = time.clock()
    self.change_mode = False
    self.pos_control = True
    self.reset_value = 0.0
    self.block_button = 0.0
    self.bef_state_white = 0.0
    self.bef_state_grey = 0.0

    # Synch
    self.slave_synch_pos = np.zeros(3)
    self.slave_synch_rot = np.array([0, 0, 0, 1])
    self.master_synch_rot = np.array([0, 0, 0, 1])

    # Button
    self.prev_buttons = [0] * 2
    self.buttons = [False] * 2
    self.buttons[WHITE_BUTTON] = True

    # Setup Subscribers/Publishers
    #~ self.feedback_pub = rospy.Publisher(self.feedback_topic, OmniFeedback)
    self.sm_control_pub = rospy.Publisher(self.sm_control_topic, Float64)
    self.ik_mc_pub = rospy.Publisher(self.ik_mc_topic, PoseStamped)
    self.gripper_pub = rospy.Publisher(self.gripper_topic, Float64)
    self.reset_pub = rospy.Publisher(self.reset_signal, Float64)
    self.vis_pub = rospy.Publisher('visualization_marker', Marker)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    rospy.Subscriber(self.slave_state_topic, PoseStamped, self.cb_slave_state)
    rospy.Subscriber(self.slave_collision_topic, Bool, self.cb_slave_collision)
    #~ rospy.Subscriber(self.ext_forces_topic, OmniFeedback, self.cb_ext_forces)  ##
    rospy.Subscriber(self.button_topic, OmniButtonEvent, self.buttons_cb)


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
    self.loginfo('Publisher frequency: [%f]' % self.publish_frequency)
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_command)
    self.loginfo('State machine state: GO_TO_CENTER')

  @smach.cb_interface(outcomes=['lock', 'succeeded_rate','succeeded_pos', 'aborted'])
  def go_to_center(user_data, self):
    if not np.allclose(np.zeros(3), self.master_pos, atol=self.hysteresis):
      #~ self.force_feedback = (self.k_center * self.master_pos + self.b_center * self.master_vel) * -1.0
      self.sm_control_pub.publish(4.0)
      return 'lock'
    else:
      if self.pos_control:
          self.sm_control_pub.publish(1.0)
          #~ self.force_feedback = np.zeros(3)
          self.slave_synch_pos = np.array(self.slave_pos)
          self.slave_synch_rot = np.array(self.slave_rot)
          self.master_synch_rot = np.array(self.master_rot)
          self.command_pos = np.array(self.slave_pos)
          self.command_rot = np.array(self.slave_rot)
          self.draw_position_region(self.slave_synch_pos)
          self.loginfo('State machine transitioning: GO_TO_CENTER:succeeded-->POSITION_CONTROL')
          return 'succeeded_pos'
      elif not self.pos_control:
          self.sm_control_pub.publish(1.0)
          #~ self.force_feedback = np.zeros(3)
          self.slave_synch_pos = np.array(self.slave_pos)
          self.slave_synch_rot = np.array(self.slave_rot)
          self.master_synch_rot = np.array(self.master_rot)
          self.command_pos = np.array(self.slave_pos)
          self.command_rot = np.array(self.slave_rot)
          self.draw_position_region(self.slave_synch_pos)
          self.loginfo('State machine transitioning: GO_TO_CENTER:succeeded-->RATE_CONTROL')
          return 'succeeded_rate'

  @smach.cb_interface(outcomes=['stay', 'leave', 'aborted'])
  def position_control(user_data, self):
    if not self.change_mode:
      #~ if reset_value:
          #~ self.slave_synch_pos = np.array(self.slave_pos)
          #~ self.slave_synch_rot = np.array(self.slave_rot)
          #~ self.master_synch_rot = np.array(self.master_rot)

      self.command_pos = self.slave_synch_pos + self.master_pos / self.position_ratio
      self.command_rot = np.array(self.master_rot)
      self.sm_control_pub.publish(1.0)
      #~ self.force_feedback = self.ext_forces  ##
      return 'stay'
    else:
      self.sm_control_pub.publish(3.0)
      #~ self.force_feedback = np.zeros(3)
      self.command_pos = np.array(self.slave_pos)
      self.command_rot = np.array(self.slave_rot)
      self.vib_start_time = rospy.get_time()
      self.change_mode = False
      self.pos_control = False
      self.loginfo('State machine transitioning: POSITION_CONTROL:leave-->RATE_CONTROL')
      return 'leave'

  @smach.cb_interface(outcomes=['vibrate', 'succeeded', 'aborted'])
  def vibratory_phase(user_data, self):
    if rospy.get_time() < self.vib_start_time + self.vib_time:
      self.sm_control_pub.publish(3.0)
      #~ t = rospy.get_time() - self.vib_start_time
      #~ amplitude = -self.vib_a*exp(-self.vib_c*t)*sin(2*pi*self.vib_freq*t);
      #~ self.force_feedback = amplitude * self.master_dir
      return 'vibrate'
    else:
      self.sm_control_pub.publish(2.0)
      # The pivot point should be inside the position area but it's better when we use the center
      #~ self.rate_pivot = self.master_pos - self.pivot_dist * self.normalize_vector(self.master_pos)
      #~ self.force_feedback = np.zeros(3)
      self.rate_pivot = np.array(self.master_pos)
      self.loginfo('State machine transitioning: VIBRATORY_PHASE:succeeded-->RATE_CONTROL')
      return 'succeeded'

  @smach.cb_interface(outcomes=['stay', 'leave', 'collision', 'aborted'])
  def rate_control(user_data, self):
    if not self.slave_collision:
        if not self.change_mode:
          #~ if reset_value:
            #~ self.slave_synch_pos = np.array(self.slave_pos)
            #~ self.slave_synch_rot = np.array(self.slave_rot)
            #~ self.master_synch_rot = np.array(self.master_rot)

          # Send the force feedback to the master
          #~ self.force_feedback = (self.k_rate * self.master_pos + self.b_rate * self.master_vel) * -1.0 ##
          # Send the rate command to the slave
          distance = sqrt(np.sum((self.master_pos - self.rate_pivot) ** 2)) / self.position_ratio
          self.command_pos += (self.rate_gain * distance * self.normalize_vector(self.master_pos)) / self.position_ratio
          self.command_rot = np.array(self.master_synch_rot)
          self.sm_control_pub.publish(2.0)

          return 'stay'
        else :
          self.command_pos = np.array(self.slave_pos)
          self.command_rot = np.array(self.slave_rot)
          #~ self.force_feedback = np.zeros(3)
          self.sm_control_pub.publish(4.0)
          self.loginfo('State machine transitioning: RATE_CONTROL:leave-->GO_TO_CENTER')
          self.change_mode = False
          self.pos_control = True
          return 'leave'
    else:
        self.command_pos = np.array(self.slave_pos)
        self.command_rot = np.array(self.slave_rot)
        self.sm_control_pub.publish(0.0)
        #~ self.force_feedback = np.zeros(3)
        self.loginfo('State machine transitioning: RATE_CONTROL:collision-->RATE_COLLISION')
        return 'collision'

  @smach.cb_interface(outcomes=['succeeded', 'aborted'])
  def rate_collision(user_data, self):
    self.loginfo('State machine transitioning: RATE_COLLISION:succeeded-->GO_TO_CENTER')
    self.change_mode = False
    self.pos_control = True
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

  def inside_workspace(self, point):
    # The workspace as an ellipsoid: http://en.wikipedia.org/wiki/Ellipsoid
    return np.sum(np.divide(point**2, self.workspace**2)) < 1

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

  #~ def send_feedback(self):
#~
    #~ feedback_msg = OmniFeedback()
    #~ force = self.change_force_axes(self.force_feedback)
    #~ pos = self.change_axes(self.center_pos)
    #~ feedback_msg.force = Vector3(*force)
    #~ feedback_msg.position = Vector3(*pos)
    #~ self.feedback_pub.publish(feedback_msg)



  # DO NOT print to the console within this function
  def cb_master_state(self, msg):
    self.master_real_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) - self.center_pos
    vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    self.master_pos = self.change_axes(pos)
    self.master_vel = self.change_axes(vel)

    # Rotate tu use the same axes orientation between master and slave
    real_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    q_1 = tr.quaternion_about_axis(self.angle_rotation_1, self.axes_rotation_1)
    aux_rot = tr.quaternion_multiply(q_1, real_rot)
    q_2 = tr.quaternion_about_axis(self.angle_rotation_2, self.axes_rotation_2)
    aux_rot_2 = tr.quaternion_multiply(q_2, aux_rot)
    q_3 = tr.quaternion_about_axis(self.angle_rotation_3, self.axes_rotation_3)
    self.master_rot = tr.quaternion_multiply(q_3, aux_rot_2)

    #Normalize velocitity
    self.master_dir = self.normalize_vector(self.master_vel)


  def cb_slave_state(self, msg):
    self.slave_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    self.slave_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

  def cb_slave_collision(self, msg):
    self.slave_collision = msg.data

  #~ def cb_ext_forces(self, msg):   ##
    #~ self.ext_forces = np.array([msg.force.x, msg.force.y, msg.force.z]) ##

  def buttons_cb(self, msg):

    self.button_states = [msg.grey_button, msg.white_button]

    #~ if  (self.button_states[GREY_BUTTON] and self.button_states[WHITE_BUTTON]):
      #~ self.change_mode = True
    #~ else:
      #~ self.change_mode = False
#~
    #~ if  (self.button_states[GREY_BUTTON]):
        #~ pass
        #~ uzjuzk
      #~
    #~ elif (self.button_states[WHITE_BUTTON]):
      #~ self.reset_value = not self.reset_value

      #------------------------------

      #TRANSITION BLOCK
    if  (self.button_states[GREY_BUTTON] and self.button_states[WHITE_BUTTON]):
      self.block_button= not self.block_button
      self.change_mode = False
      self.loginfo('TRANSITION')
      
      
    #Modo emergencia _ DEAD MEN
    if (self.button_states[WHITE_BUTTON]):
      self.reset_value = 1.0
      self.loginfo('dead men ')
    else:
      self.reset_value = 0.0

    if( self.bef_state_white == 0.0 and self.bef_state_grey == 0.0):
          #BLOCK0 = reset & change_mode
          
        if (self.button_states[GREY_BUTTON] and self.block_button==0.0 and (not self.button_states[WHITE_BUTTON])):
          self.change_mode = True
          self.loginfo('CHANGE MODE')
        else:
          self.change_mode = False
          self.loginfo('False Change mode')

        #~ if (self.button_states[WHITE_BUTTON] and self.block_button == 0.0 and (not self.button_states[GREY_BUTTON])):
          #~ self.reset_value = not self.reset_value
          #~ self.reset_value = 1.0
          #~ self.loginfo('RESET')
          #~ 
        #~ if (not self.button_states[WHITE_BUTTON]):
          #~ self.reset_value = not self.reset_value
          #~ self.reset_value = 0.0
          
         #BLOCK1 = open & close grip
        if(self.block_button==1):
           if (self.button_states[GREY_BUTTON]==1.0 and self.button_states[WHITE_BUTTON]==0.0 and (not self.button_states[WHITE_BUTTON])):   # Close
             self.gripper_value += 0.0001
             self.loginfo('CLOSE')
           if self.gripper_value > 1.0:
             self.gripper_value = 1.0
           elif (self.button_states[GREY_BUTTON]==0.0 and self.button_states[WHITE_BUTTON]==1.0 and (not self.button_states[GREY_BUTTON])):     # Open
             self.gripper_value -= 0.0001
             self.loginfo('COPEN')
           if self.gripper_value < 0.0:
             self.gripper_value = 0.0

    self.bef_state_white = self.button_states[WHITE_BUTTON]
    self.bef_state_grey = self.button_states[GREY_BUTTON]


      #------------------------------

    self.reset_pub.publish(Float64(self.reset_value))

  def publish_command(self, event):


    position, orientation = self.command_pos, self.command_rot
    ik_mc_msg = PoseStamped()
    ik_mc_msg.header.frame_id = self.frame_id
    ik_mc_msg.header.stamp = rospy.Time.now()
    ik_mc_msg.pose.position = Point(*position)
    ik_mc_msg.pose.orientation = Quaternion(*orientation)

    #Button selection
    #~ if (self.button_states[GREY_BUTTON]==1 and self.button_states[WHITE_BUTTON]==0):   # Close
      #~ self.gripper_value += 0.0001
      #~ if self.gripper_value > 1.0:
          #~ self.gripper_value = 1.0
    #~ elif (self.button_states[GREY_BUTTON]==0 and self.button_states[WHITE_BUTTON]==1 ):     # Open
      #~ self.gripper_value -= 0.0001
      #~ if self.gripper_value < 0.0:
          #~ self.gripper_value = 0.0

    try:
      self.gripper_pub.publish(Float64(self.gripper_value))
      self.ik_mc_pub.publish(ik_mc_msg)
      #~ t1 = time.time()

      #~ t2 =time.time()
    except rospy.exceptions.ROSException:
      pass

    #~ dif_time = t2-t1
    #~ self.loginfo('Diference of time %0.3f ms' % (dif_time * 1000.0))
    #~ self.time_counter = time.clock()



  def draw_position_region(self, center_pos):
    marker = Marker()
    marker.header.frame_id = self.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.id = 0;
    marker.type = marker.SPHERE
    marker.ns = 'position_region'
    marker.action = marker.ADD
    marker.pose.position.x = center_pos[0]
    marker.pose.position.y = center_pos[1]
    marker.pose.position.z = center_pos[2]
    #~ Workspace ellipsoid: self.workspace
    marker.scale.x = 0.2 * self.workspace[0]/self.position_ratio
    marker.scale.y = 0.2 * self.workspace[1]/self.position_ratio
    marker.scale.z = 0.2 * self.workspace[2]/self.position_ratio
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.2
    #~ Publish
    self.vis_pub.publish(marker)


if __name__ == '__main__':
  rospy.init_node('rate_position_controller', log_level=rospy.WARN)
  try:
    controller = RatePositionController()
    controller.execute()
  except rospy.exceptions.ROSInterruptException:
      pass
