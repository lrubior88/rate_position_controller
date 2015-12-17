#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent

_struct_3d = struct.Struct("<3d")

class Send_udp_force:
  def __init__(self):
      
    # Read parameters
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port = self.read_parameter('~write_port', '34900')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.device_name = self.read_parameter('~device_name', 'phantom')
    self.force_topic_name = "/%s/force_feedback" % self.device_name
    
    rospy.loginfo('UDP Socket writing on port [%s]' % (self.write_port))

      
    # Setup Subscriber
    rospy.Subscriber(self.force_topic_name, OmniFeedback, self.force_command_cb)
    self.f_msg = OmniFeedback()

    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.send_udp)
    rospy.spin()

  def force_command_cb(self, msg):
    self.f_msg = msg

  def send_udp(self, event):
    try:
        # Serialize a OmniStamp msg
        buff = StringIO()
        f_x = self.f_msg.force.x
        f_y = self.f_msg.force.y
        f_z = self.f_msg.force.z
  
        buff.write(_struct_3d.pack(f_x, f_y, f_z))
        
        # Send values
        self.write_socket.sendto(buff.getvalue(), (self.write_ip, self.write_port))
   
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)
    except socket.error, e:
        result = self.write_socket.bind((self.write_ip,self.write_port))
        if result:
            rospy.logwarn('Connection refused')

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Send_udp_force()
