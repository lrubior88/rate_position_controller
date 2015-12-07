#! /usr/bin/env python
"""
1: Position
2: Rate
3: Vibration    X -> Transitions
4: Go to center X -> Transitions
5: Collision    X -> Transitions

El nodo strategies permite cambiar de modo, y en funcion de este,
se enviara el estado(position o rate) y las funciones que se deben hacer
cuando se pulsen los diferntes botones del phantom.

"""
import rospy
# Messages
from std_msgs.msg import Float64
from omni_msgs.msg import OmniState, OmniButtonEvent
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool

# Math
import numpy as np
from math import pi, exp, sin, sqrt
import tf.transformations as tr

GREY_BUTTON = 0
WHITE_BUTTON = 1

class Strategies:

  def __init__(self):

    # Topics to interact
    master_name = self.read_parameter('~master_name', 'phantom')
    slave_name = self.read_parameter('~slave_name', 'grips')
    self.mode_name = self.read_parameter('~mode_name', 'c')

    self.gripper_topic = '/%s/GRIP/command' % slave_name
    self.button_topic = '/%s/button' % master_name
    self.sm_control_topic = '/rate_pos/sm_control'
    self.master_state_topic = '/%s/state' % master_name
    self.sm_desired_topic = '/rate_pos/sm_desired'
    self.slave_collision_topic = '/%s/collision' % slave_name
    self.reset_signal = '/reset'

    # Setup Subscribers/Publishers
    self.sm_desired_pub = rospy.Publisher(self.sm_desired_topic, Float64)
    self.gripper_pub = rospy.Publisher(self.gripper_topic, Float64)
    self.reset_pub = rospy.Publisher(self.reset_signal, Float64)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
    rospy.Subscriber(self.button_topic, OmniButtonEvent, self.buttons_cb)
    rospy.Subscriber(self.sm_control_topic, Float64, self.control_cb)
    rospy.Subscriber(self.slave_collision_topic, Bool, self.slave_collision_cb)

    # Initial values
    self.center_pos = np.array([0, 0, 0])
    self.master_pos = np.zeros(3)
    self.master_rot = np.array([0, 0, 0, 1])
    self.master_vel = np.zeros(3)
    self.master_dir = np.zeros(3)
    self.slave_collision = False
    self.gripper_value = 0.0
    self.button_states = np.array([0, 0])
    self.reset_value = 1.0
    self.block_button = False
    self.pre_state_white = 0.0
    self.pre_state_grey = 0.0
    self.pre_control_value = 1.0
    self.sm_desired = 1.0
    self.prev_sm_desired = 0.0
    self.signal_activation = 0
    self.publish_frequency = self.read_parameter('~publish_rate', 1000.0)
    ### Axis modification
    self.position_axes = [0, 1, 2]
    self.position_sign = np.array([1.0, 1.0, 1.0])
    self.axes_mapping = self.read_parameter('~axes_mapping', ['x', 'y' ,'z'])
    rospy.logwarn('axes_mapping[0] -> %s' % self.axes_mapping[0])
    rospy.logwarn('axes_mapping[1] -> %s' % self.axes_mapping[1])
    rospy.logwarn('axes_mapping[2] -> %s' % self.axes_mapping[2])
    #Workspace
    width = self.read_parameter('~workspace/width', 140.0)
    height = self.read_parameter('~workspace/height', 100.0)
    depth = self.read_parameter('~workspace/depth', 55.0)
    self.workspace = np.array([width, depth, height])
    #Timer
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_frequency), self.publish_command)
    rospy.spin()

  def publish_command(self, event):

    if(self.mode_name == 'a'):   # rate_pos_bola

      if self.inside_workspace(self.master_pos):
        self.sm_desired = 1.0
      else:
        self.sm_desired = 2.0

      if (self.prev_sm_desired != self.sm_desired):
        self.prev_sm_desired = self.pre_control_value
        self.sm_desired_pub.publish(Float64(self.sm_desired))
      else:
        self.prev_sm_desired = self.pre_control_value

    elif(self.mode_name == 'b'): # rate_pos_button

      if (self.button_states[GREY_BUTTON] and self.signal_activation==1 and (not self.button_states[WHITE_BUTTON])):
        self.command_control()

      self.reset_value = self.button_states[WHITE_BUTTON]
      self.reset_pub.publish(Float64(self.reset_value))
      
      #~ elif (self.button_states[WHITE_BUTTON] and self.signal_activation==1 and (not self.button_states[GREY_BUTTON])):
        #~ if (self.gripper_value == 0.0):
           #~ self.gripper_value = 1.0
        #~ elif (self.gripper_value == 1.0):
           #~ self.gripper_value = 0.0
        #~ self.gripper_pub.publish(Float64(self.gripper_value))

      self.signal_activation = 0

    elif(self.mode_name == 'c'): # rate_pos_sof

      #TRANSITION BLOCK
      if(self.signal_activation == 1):
          if  (self.button_states[GREY_BUTTON] and self.button_states[WHITE_BUTTON]): #      self.loginfo('TRANSITION')
            self.block_button = not self.block_button


          if( self.pre_state_white == 1.0 and self.pre_state_grey == 1.0):
              pass
              
          else:
              #BLOCK0 = reset&change_mode
              if (self.button_states[GREY_BUTTON] and (not self.block_button) and (not self.button_states[WHITE_BUTTON])):
                   self.command_control()               # self.loginfo('CHANGE MODE')
                   
              if (not self.block_button): # self.loginfo('RESET')
                 self.reset_value = self.button_states[WHITE_BUTTON]
                 self.reset_pub.publish(Float64(self.reset_value))
                
              #~ if (self.button_states[WHITE_BUTTON] and (not self.block_button)  and (not self.button_states[GREY_BUTTON])): # self.loginfo('RESET')
                 #~ self.reset_value = 1.0
                 #~ self.reset_pub.publish(Float64(self.reset_value))
                 #~ 
              #~ if (self.reset_value == 1.0 and (not self.button_states[WHITE_BUTTON])):
                 #~ self.reset_value = 0.0
                 #~ self.reset_pub.publish(Float64(self.reset_value))

          self.pre_state_white = self.button_states[WHITE_BUTTON]
          self.pre_state_grey = self.button_states[GREY_BUTTON]
          self.signal_activation = 0

      #BLOCK1 = close&open grip
      elif(self.block_button):
         if (self.button_states[GREY_BUTTON] == 1.0 and self.button_states[WHITE_BUTTON] == 0.0):   # Close
           self.gripper_value += 0.0001       # self.loginfo('CLOSE')
           if self.gripper_value > 1.0:
             self.gripper_value = 1.0

         elif (self.button_states[GREY_BUTTON] == 0.0 and self.button_states[WHITE_BUTTON] == 1.0):     # Open
           self.gripper_value -= 0.0001       # self.loginfo('OPEN')
           if self.gripper_value < 0.0:
             self.gripper_value = 0.0

         self.gripper_pub.publish(Float64(self.gripper_value))


  def command_control(self):
    if(self.pre_control_value == 1.0):   #position -> rate
       self.sm_desired = 2.0
    elif(self.pre_control_value == 2.0): #rate -> position
       self.sm_desired = 1.0

    self.sm_desired_pub.publish(Float64(self.sm_desired))

  def buttons_cb(self, msg):
    self.button_states = [msg.grey_button, msg.white_button]
    self.signal_activation=1

  def control_cb(self, msg): #Valor ejecutado de control en rate_pos_control
    self.pre_control_value = msg.data

  def cb_master_state(self, msg):
    self.master_real_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) - self.center_pos
    vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
    self.master_pos = self.change_axes(pos)
    self.master_vel = self.change_axes(vel)

    #Normalize velocitity
    self.master_dir = self.normalize_vector(self.master_vel)

  def inside_workspace(self, point):
    # The workspace as an ellipsoid: http://en.wikipedia.org/wiki/Ellipsoid
    return np.sum(np.divide(point**2, self.workspace**2)) < 1

  def change_axes(self, array, index=None, sign=None):
    if index == None:
      index = self.position_axes
    if sign == None:
      sign = self.position_sign
    result = np.zeros(len(array))
    for i, idx in enumerate(index):
      result[i] = array[idx] * sign[idx]
    return result
  def slave_collision_cb(self, msg):
    self.slave_collision = msg.data

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def normalize_vector(self, v):
    result = np.array(v)
    norm = np.sqrt(np.sum((result ** 2)))
    if norm:
      result /= norm
    return result

if __name__ == '__main__':
  rospy.init_node('strategies', log_level=rospy.WARN)
  try:
    controller = Strategies()
  except rospy.exceptions.ROSInterruptException:
      pass
