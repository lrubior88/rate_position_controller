#! /usr/bin/env python
import rospy, time, math
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent

_struct_10d = struct.Struct("<10d")


class HardwareInterface:
  def __init__(self): 

    # Load parameters
    self.device_name = self.read_parameter('~device_name', 'haptic_device')
    self.reference_frame = self.read_parameter('~reference_frame', 'world')
    self.units = self.read_parameter('~units', 'mm')
    # Topics
    self.state_topic = '/%s/state' % self.device_name
    self.feedback_topic = '/%s/force_feedback' % self.device_name
    self.pose_topic = '/%s/pose' % self.device_name
    self.button_topic = '/%s/button' % self.device_name
    # Setup Subscribers/Publishers
    #~ rospy.Subscriber(self.feedback_topic, OmniFeedback, self.cb_feedback)
    self.state_pub = rospy.Publisher(self.state_topic, OmniState)
    self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped)
    self.button_pub = rospy.Publisher(self.button_topic, OmniButtonEvent)
    
    self.read_port = int(self.read_parameter('~read_port', 5051))
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')

    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
  
    while True:
        #rospy.loginfo('Got connection from [%d]' % (addr))
        data,addr=self.read_socket.recvfrom(1024)
        #rospy.loginfo('Data %s' % (data))
        if data:
          (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w, vel_x, vel_y, vel_z) = _struct_10d.unpack(data[0:80])
          if not(math.isnan(pos_x)):
            cmd_msg = PoseStamped()
            cmd_msg.header.frame_id = self.reference_frame
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.pose.position.x = pos_x/1000.0
            cmd_msg.pose.position.y = pos_y/1000.0
            cmd_msg.pose.position.z = pos_z/1000.0
            cmd_msg.pose.orientation.x = rot_x
            cmd_msg.pose.orientation.y = rot_y
            cmd_msg.pose.orientation.z = rot_z
            cmd_msg.pose.orientation.w = rot_w
            
            omni_msg = OmniState()
            omni_msg.header.frame_id = self.reference_frame
            omni_msg.header.stamp = rospy.Time.now()
            omni_msg.pose.position.x = pos_x
            omni_msg.pose.position.y = pos_y
            omni_msg.pose.position.z = pos_z
            omni_msg.pose.orientation.x = rot_x
            omni_msg.pose.orientation.y = rot_y
            omni_msg.pose.orientation.z = rot_z
            omni_msg.pose.orientation.w = rot_w
            omni_msg.velocity.x = vel_x
            omni_msg.velocity.y = vel_y
            omni_msg.velocity.z = vel_z
            
            button_msg = OmniButtonEvent()
            button_msg.grey_button = 0
            button_msg.white_button = 0
            
            self.state_pub.publish(omni_msg)
            self.pose_pub.publish(cmd_msg)
            self.button_pub.publish(button_msg)
        
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
    
  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  rospy.init_node('haptic_device_interface')
  interface = HardwareInterface()
