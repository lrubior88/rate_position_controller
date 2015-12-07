#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

_struct_9d = struct.Struct("<9d")

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

class Send_udp_pose:
  def __init__(self):
    # Set-up publishers/subscribers
    rospy.Subscriber('/justin/ik_command', PoseStamped, self.ik_command_cb)
    rospy.Subscriber('/reset', Float64, self.reset_cb)
    rospy.Subscriber('/counter', Float64, self.counter_cb)
    #~ rospy.Subscriber('/phantom/pose', PoseStamped, self.ik_command_cb)
    # Read all the parameters from the parameter server
    self.publish_frequency = self.read_parameter('~publish_frequency', 1000.0)
    # TCP
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = int(self.read_parameter('~write_port', 5052))
    self.reset_value = 0.0
    self.counter_value = 0.0

    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #~ self.write_socket.bind((self.write_ip,self.write_port))
    rospy.spin()
    
    
  def reset_cb(self, msg):
    self.reset_value = msg.data

  def counter_cb(self, msg):
    self.counter_value = msg.data

  def ik_command_cb(self, msg):

    try:
        cmd_msg =  PoseStamped()
        cmd_msg.pose = msg.pose

        # Serialize cmd_msg
        file_str = StringIO()
        self.double_serialize(cmd_msg, file_str)

        #~ rospy.loginfo('Info to send %s' % (file_str.getvalue()))
        self.write_socket.sendto(file_str.getvalue(), (self.write_ip, self.write_port))
        #self.write_socket.close()

    except socket.error, e:
        result = self.write_socket.bind((self.write_ip,self.write_port))
        if result:
			rospy.logwarn('Connection refused')
	

  def double_serialize(self, msg, buff):
    try:
       buff.write(_struct_9d.pack(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, self.reset_value, self.counter_value))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)


  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Send_udp_pose()
