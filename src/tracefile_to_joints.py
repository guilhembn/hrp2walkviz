#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
import socket
import struct
import nav_msgs

JOINT_STATE_PUBLISHER_PUB = "/hrp2_joints"

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)



class JointReaderPublisher:
  def __init__(self, node_name, tracefile_param):
    self.name = node_name
    self.conn = None
    self.addr = None
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT)) 
    s.listen(1)
    self.conn, self.addr = s.accept()
    rospy.init_node(self.name)
    #self.tracefile = open(rospy.get_param(tracefile_param), 'r')
    self.tf_broadcaster = tf.TransformBroadcaster()
    self.joint_pub = rospy.Publisher(JOINT_STATE_PUBLISHER_PUB, JointState, queue_size=10)
    

    print('Connected by', self.addr)
    while True:
        data = self.conn.recv(36 * 8)
        if not data:
            break
        values = struct.unpack('36d', data)  
        x, y, z, roll, pitch, yaw = values[0:6]
        dofs = values[6:]
                  
        # Publish base_link pose through tf
        self.tf_broadcaster.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(roll, pitch, yaw), rospy.Time.now(), "base_link", "odom")
        self.tf_broadcaster.sendTransform((0, 0, -z), tf.transformations.quaternion_from_euler(-roll, -pitch, 0), rospy.Time.now(), "base_footprint", "base_link")
       
        # Publish DoF values through joint_state_publisher
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ['CHEST_JOINT0', 'CHEST_JOINT1', 'HEAD_JOINT0', 'HEAD_JOINT1', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']
        joint_msg.position = dofs
        self.joint_pub.publish(joint_msg)


if __name__ == '__main__':
  joint_reader = JointReaderPublisher("joint_reader_publisher", "tracefile")
  
  rospy.spin()
