#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState

JOINT_STATE_PUBLISHER_PUB = "/hrp2_joints"

class JointReaderPublisher:
  def __init__(self, node_name, tracefile_param):
    self.name = node_name
    rospy.init_node(self.name)
    self.tracefile = open(rospy.get_param(tracefile_param), 'r')
    self.tf_broadcaster = tf.TransformBroadcaster()
    self.joint_pub = rospy.Publisher(JOINT_STATE_PUBLISHER_PUB, JointState, queue_size=10)
    self.timer = rospy.Timer(rospy.Duration(0.005), self.read_send_hrp2_pose)

  def read_send_hrp2_pose(self, _):
    line = self.tracefile.readline().strip()
    values = map(float, line.split(" "))
    x, y, z, roll, pitch, yaw = values[47:53]
    dofs = values[53:83]

    # Publish base_link pose through tf
    self.tf_broadcaster.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(roll, pitch, yaw), rospy.Time.now(), "base_link", "robot_start")
    
    # Publish DoF values through joint_state_publisher
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['CHEST_JOINT0', 'CHEST_JOINT1', 'HEAD_JOINT0', 'HEAD_JOINT1', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']
    joint_msg.position = dofs
    self.joint_pub.publish(joint_msg)


if __name__ == '__main__':
  joint_reader = JointReaderPublisher("joint_reader_publisher", "tracefile")
  
  rospy.spin()
