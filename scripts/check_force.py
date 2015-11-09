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




class Check_force:


  def __init__(self):
    # Read all the parameters from the parameter server
    # Topics to interact
    self.feedback_topic = '/phantom/force_feedback' 
    self.master_state_topic = '/phantom/state' 

    # Setup Subscribers/Publishers
    self.feedback_pub = rospy.Publisher(self.feedback_topic, OmniFeedback)
    rospy.Subscriber(self.master_state_topic, OmniState, self.cb_master_state)
	
    #~ self.is_drawn = False
    self.external_forces = np.zeros(3)
    self.k_prop = 0.5
    self.master_real_pos = np.zeros(3)
    rospy.spin()

  def cb_master_state(self, msg):
    self.master_real_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    # Rotate tu use the same axes orientation between master and slave
    real_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    if ((self.master_real_pos[0]>0) ):
        self.external_forces[0] = self.k_prop * (0 - self.master_real_pos[0])
    else:
        self.external_forces = np.zeros(3)

    feedback_msg = OmniFeedback()
    feedback_msg.force = Vector3(*self.external_forces)
    feedback_msg.position = Vector3(*self.master_real_pos)
    self.feedback_pub.publish(feedback_msg)



if __name__ == '__main__':
    rospy.init_node('Check_force', log_level=rospy.WARN)
    loader = Check_force()
