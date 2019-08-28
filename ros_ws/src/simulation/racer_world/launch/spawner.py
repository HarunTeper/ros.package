#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
import sys



def move(data):
  #global time
  #global oldtime
  #global forward
  global waypointsx
  global waypointsy
  global waypointx
  global waypointy
  global index
  
  speed = int(sys.argv[4])
  
  waypointx=waypointsx[index]
  waypointy=waypointsy[index]

  msg = ModelState()
  msg.model_name =sys.argv[1]
  x=data.name.index(msg.model_name)
  msg.pose = data.pose[x]
  msg.twist = data.twist[x]
  msg.reference_frame = 'ground_plane'


  y=data.name.index('racer')
  racer_pose = data.pose[y]

  distancexracer = msg.pose.position.x  - racer_pose.position.x 
  distanceyracer = msg.pose.position.y - racer_pose.position.y
  lengthracer = math.sqrt(math.pow(distancexracer,2)+math.pow(distanceyracer,2))
  if lengthracer < 1:
    msg.twist.linear.x=0
    msg.twist.linear.y=0
  else:
    distancex = waypointx - msg.pose.position.x 
    distancey = waypointy - msg.pose.position.y
    length = math.sqrt(math.pow(distancex,2)+math.pow(distancey,2))

    if length < 0.05:
      index = (index+1) % len(waypointsx)
      waypointx = waypointsx[index]
      waypointy = waypointsy[index]

      distancex = waypointx - msg.pose.position.x 
      distancey = waypointy - msg.pose.position.y
      length = math.sqrt(math.pow(distancex,2)+math.pow(distancey,2))
    else:
      msg.twist.linear.x=distancex/length*speed
      msg.twist.linear.y=distancey/length*speed
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

    waypointsx= map(int,list(sys.argv[2].split(",")))
    waypointsy= map(int,list(sys.argv[3].split(",")))
    index = 0
    
    rospy.Subscriber(model_state_topic, ModelStates, move, queue_size=1)
    pub = rospy.Publisher(set_model_state_topic, ModelState, queue_size=1)
    
    rospy.loginfo("Node 'spawner' started.\nListening to %s, publishing to %s.", model_state_topic, set_model_state_topic)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
