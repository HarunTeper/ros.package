#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState

def move(data):
  global time
  global oldtime
  global forward 
  
  time = rospy.get_rostime()
  if time.secs-oldtime.secs > 5:
    msg = ModelState()
    msg.model_name ='unit_sphere'
    x=data.name.index('unit_sphere')
    msg.pose = data.pose[x]
    msg.twist = data.twist[x]
    msg.reference_frame = 'ground_plane'
    
    forward = not forward
    oldtime = time
    if forward:
      msg.twist.linear.x=1
      msg.twist.linear.y=0
      msg.twist.linear.z=0
      msg.twist.angular.x=0
      msg.twist.angular.y=0
      msg.twist.angular.z=0
    if not forward:
      msg.twist.linear.x=-1
      msg.twist.linear.y=0
      msg.twist.linear.z=0
      msg.twist.angular.x=0
      msg.twist.angular.y=0
      msg.twist.angular.z=0
    pub.publish(msg)


  
if __name__ == '__main__': 
  try:
    rospy.init_node('spawner')
    model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states') 
    set_model_state_topic = rospy.get_param('~set_model_state_topic', '/gazebo/set_model_state') 
     
    forward = True
    time = rospy.get_rostime()
    oldtime= time
    
    rospy.Subscriber(model_state_topic, ModelStates, move, queue_size=1)
    pub = rospy.Publisher(set_model_state_topic, ModelState, queue_size=1)
    
    rospy.loginfo("Node 'spawner' started.\nListening to %s, publishing to %s.", model_state_topic, set_model_state_topic)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
