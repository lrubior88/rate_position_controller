#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

import roslib.message

_struct_7d = struct.Struct("<7d")

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

class Receive_tcp_pose:
  def __init__(self):

    # Setup Subscribers/Publishers
    self.justin_endpoint_pub = rospy.Publisher("/justin/endPoint_state", PoseStamped)

    # Read all the parameters from the parameter server
    self.publish_frequency = self.read_parameter('~publish_frequency', 1000.0)
    # UDP
    self.read_port = int(self.read_parameter('~read_port', 5051))
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = int(self.read_parameter('~write_port', 5052))
    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.read_socket.bind((self.write_ip, self.read_port))
    rospy.loginfo('TCP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    # rospy.on_shutdown(self.shutdown_hook)
    #Initialize read socket
    self.read_socket.listen(1)
    
    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
    
    while True:
      conn,addr = self.read_socket.accept()   # Establish connection with client.       
      try:
        while True:
        
            #rospy.loginfo('Got connection from [%d]' % (addr))
            data=conn.recv(1024)
            #rospy.loginfo('Data %s' % (data))
            if not data:
                break
                
            (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w) = _struct_7d.unpack(data[0:56])
            cmd_msg=PoseStamped()
            cmd_msg.header.frame_id ="base_link"
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.pose.position.x = pos_x
            cmd_msg.pose.position.y = pos_y
            cmd_msg.pose.position.z = pos_z
            cmd_msg.pose.orientation.x = rot_x
            cmd_msg.pose.orientation.y = rot_y
            cmd_msg.pose.orientation.z = rot_z
            cmd_msg.pose.orientation.w = rot_w

            #~ rospy.loginfo('pos_x %s' % (pos_x))
            #~ rospy.loginfo('pos_y %s' % (pos_y))
            #~ rospy.loginfo('pos_z %s' % (pos_z))
            #~ rospy.loginfo('rot_x %s' % (rot_x))
            #~ rospy.loginfo('rot_y %s' % (rot_y))
            #~ rospy.loginfo('rot_z %s' % (rot_z))
            #~ rospy.loginfo('rot_w %s' % (rot_w))

            self.justin_endpoint_pub.publish(cmd_msg)
      finally:
        conn.close()     # Close the connection

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)
    
  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Receive_tcp_pose()



