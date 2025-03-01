#!/usr/bin/env python

import rospy, math, tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

import sys

def calcodom(data):
    current_time = rospy.Time.now();
    odom_broadcaster = tf.TransformBroadcaster();
    index = data.name.index("racer")
    rospy.loginfo("%s",data.pose[index].orientation)
    #rospy.loginfo("%s",data.twist[1])

    odom_trans = TransformStamped()
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    

    odom_trans.transform.translation.x = data.pose[index].position.x;
    odom_trans.transform.translation.y = data.pose[index].position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = data.pose[index].orientation;
    
    odom_broadcaster.sendTransform(
	(data.pose[index].position.x,data.pose[index].position.y,0.0),
	(data.pose[index].orientation.x,data.pose[index].orientation.y,data.pose[index].orientation.z,data.pose[index].orientation.w),
	rospy.Time.now(),
	"base_link",
	"odom");

    odom = Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = data.pose[index].position.x;
    odom.pose.pose.position.y = data.pose[index].position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = data.pose[index].orientation;
    odom.pose.covariance[0] = 0.2;
    odom.pose.covariance[7] = 0.2;
    odom.pose.covariance[35] = 0.4;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = data.twist[index].linear.x;
    odom.twist.twist.linear.y = data.twist[index].linear.y;
    odom.twist.twist.angular.z = data.twist[index].angular.z;

    pub.publish(odom);
	

if __name__ == '__main__': 
  try:
    rospy.init_node('idealodometry')
    listener = tf.TransformListener()
    model_state_topic = rospy.get_param('~model_state_topic', '/gazebo/model_states') 
    odom_topic = rospy.get_param('~odom_topic', '/odom') 
    
    rospy.Subscriber(model_state_topic, ModelStates, calcodom, queue_size=1)
    pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)

    rospy.spin()
    
#    rospy.loginfo("Node 'odom' started.\nListening to /tf, publishing to %s.", odom_topic)
#    rate = rospy.Rate(1)
#    while not rospy.is_shutdown():
#	try:
#		(trans,rot) = listener.lookupTransform('chassis','right_wheel_back',rospy.Time(0))
#   	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#		continue
#	rospy.loginfo()
  except rospy.ROSInterruptException:
    pass
