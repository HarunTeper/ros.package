#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from drive_msgs.msg import drive_param

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global frame_id
  global pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

  msg = drive_param()
  msg.velocity = v
  msg.angle = steering
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_drive_param')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    drive_param_cmd_topic = rospy.get_param('~drive_param_cmd_topic', '/commands/drive_param')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(drive_param_cmd_topic, drive_param, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_drive_param' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", drive_param_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
