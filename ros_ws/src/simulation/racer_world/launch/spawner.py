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
  
  waypointx=waypointsx[index]
  waypointy=waypointsy[index]

  msg = ModelState()
  msg.model_name =sys.argv[1]
  x=data.name.index(msg.model_name)
  msg.pose = data.pose[x]
  msg.twist = data.twist[x]
  msg.reference_frame = 'ground_plane'
  distancex = waypointx - msg.pose.position.x 
  distancey = waypointy - msg.pose.position.y

  length = math.sqrt(math.pow(distancex,2)+math.pow(distancey,2))
  if length < 0.01:
    index = (index +x) % len(waypointsx)
    waypointx = waypointsx[index]
    waypointy = waypointsy[index]
    distancex = waypointx - msg.pose.position.x 
    distancey = waypointy - msg.pose.position.y
    length = math.sqrt(math.pow(distancex,2)+math.pow(distancey,2))
  if length == 0 :
    msg.twist.linear.x=0
    msg.twist.linear.y=0
  else:
    msg.twist.linear.x=distancex/length
    msg.twist.linear.y=distancey/length
  msg.twist.linear.z=0
  msg.twist.angular.x=0
  msg.twist.angular.y=0
  msg.twist.angular.z=0
  pub.publish(msg)
  #rospy.loginfo("waypointx is %f",waypointx)
  #rospy.loginfo("waypointy is %f",waypointy)
  #rospy.loginfo("waypointx is %f",waypointx)
  #rospy.loginfo("waypointy is %f",waypointy)
  #rospy.loginfo("distancex is %f",distancex)
  #rospy.loginfo("distancey is %f",distancey)
  #rospy.loginfo("length is %f",length)
  #rospy.loginfo("linearx is %f",distancex/length)
  #rospy.loginfo("lineary is %f",distancey/length)
    
  #time = rospy.get_rostime()
  #if time.secs-oldtime.secs > 5:
  #  msg = ModelState()
  #  msg.model_name ='unit_sphere'
  #  x=data.name.index('unit_sphere')
  #  msg.pose = data.pose[x]
  #  msg.twist = data.twist[x]
  #  msg.reference_frame = 'ground_plane'
  #  
  #  forward = not forward
  #  oldtime = time
  #  if forward:
  #    msg.twist.linear.x=1
  #    msg.twist.linear.y=0
  #    msg.twist.linear.z=0
  #    msg.twist.angular.x=0
  #    msg.twist.angular.y=0
  #    msg.twist.angular.z=0
  #  if not forward:
  #    msg.twist.linear.x=-1
  #    msg.twist.linear.y=0
  #    msg.twist.linear.z=0
  #    msg.twist.angular.x=0
  #    msg.twist.angular.y=0
  #    msg.twist.angular.z=0
  #  pub.publish(msg)


  
if __name__ == '__main__': 
  try:
    rospy.init_node('spawner')
    model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states') 
    set_model_state_topic = rospy.get_param('~set_model_state_topic', '/gazebo/set_model_state') 
    
    #forward = True
    #time = rospy.get_rostime()
    #oldtime= time

    #rospy.loginfo("test")
    #rospy.loginfo(sys.argv[1])
    #rospy.loginfo(sys.argv[2])
    #rospy.loginfo(sys.argv[3])
    #rospy.loginfo("test")

    waypointsx= map(int,list(sys.argv[2].split(",")))
    waypointsy= map(int,list(sys.argv[3].split(",")))
    index = 0
    #rospy.loginfo(waypointsx)
    #rospy.loginfo(waypointsy)
    
    rospy.Subscriber(model_state_topic, ModelStates, move, queue_size=1)
    pub = rospy.Publisher(set_model_state_topic, ModelState, queue_size=1)
    
    rospy.loginfo("Node 'spawner' started.\nListening to %s, publishing to %s.", model_state_topic, set_model_state_topic)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
