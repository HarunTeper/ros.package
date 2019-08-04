#!/usr/bin/env python

import rospy, math, tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.msg import tfMessage
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

import sys

def direction(data):
    global forward
    if data.data >0:
	forward = 1
    if data.data < 0:
	forward = 0

def calcodom(data):
    global forward
    global rwb_last_rotation
    global lwb_last_rotation
    global x_pos
    global y_pos
    global th
    global x_orientation
    global y_orientation
    global z_orientation
    global w_orientation
    global last_time


    current_time = rospy.Time.now()
    time_diff = (current_time - last_time).nsecs
    last_time = current_time

    odom_broadcaster = tf.TransformBroadcaster()
    #rospy.loginfo(data.transforms[0])

    if data.transforms[0].child_frame_id == "left_steering" and time_diff != 0:

	lwb = data.transforms[1]
	rwb = data.transforms[4]

	#rospy.loginfo(lwb.transform.rotation.y)
	#rospy.loginfo(rwb.transform.rotation.y)

	lwb_diff = lwb.transform.rotation.y - lwb_last_rotation
	rwb_diff = rwb.transform.rotation.y - rwb_last_rotation

	if lwb_diff < 0.01 and lwb_diff > -0.01:
		lwb_diff = 0
	if rwb_diff < 0.01 and rwb_diff > -0.01:
		rwb_diff = 0
	if lwb_diff < -0.5 and forward == 1:
	    #rospy.loginfo("lwbbefore %f",lwb_diff)
	    #rospy.loginfo("lwb %f",lwb_last_rotation)
	    #rospy.loginfo("lwb %f",lwb.transform.rotation.y)
	    lwb_diff = 2+lwb.transform.rotation.y-lwb_last_rotation
	    #rospy.loginfo("lwb %f",lwb_diff)
	if rwb_diff < -0.5 and forward == 1:
	    #rospy.loginfo("rwbbefore %f",rwb_diff)
	    #rospy.loginfo("rwb %f",rwb_last_rotation)
	    #rospy.loginfo("rwb %f",rwb.transform.rotation.y)
	    rwb_diff = 2+rwb.transform.rotation.y-rwb_last_rotation
	    #rospy.loginfo("rwb %f",rwb_diff)
	if lwb_diff > 0.5 and forward == 0:
	    #rospy.loginfo("lwbbefore %f",lwb_diff)
	    #rospy.loginfo("lwb %f",lwb_last_rotation)
	    #rospy.loginfo("lwb %f",lwb.transform.rotation.y)
	    lwb_diff = 2-lwb.transform.rotation.y+lwb_last_rotation
	    #rospy.loginfo("lwb %f",lwb_diff)
	if rwb_diff > 0.5 and forward == 0:
	    #rospy.loginfo("rwbbefore %f",rwb_diff)
	    #rospy.loginfo("rwb %f",rwb_last_rotation)
	    #rospy.loginfo("rwb %f",rwb.transform.rotation.y)
	    rwb_diff = 2-rwb.transform.rotation.y+rwb_last_rotation
	    #rospy.loginfo("rwb %f",rwb_diff)
	
	lwb_diff = lwb_diff * 2 * 0.05 * math.pi
	rwb_diff = rwb_diff * 2 * 0.05 * math.pi

	#rospy.loginfo(lwb_diff)
	#rospy.loginfo(rwb_diff)
	#rospy.loginfo("")

	lwb_last_rotation = lwb.transform.rotation.y
	rwb_last_rotation = rwb.transform.rotation.y
	
	vel_left = lwb_diff / time_diff
	vel_right = rwb_diff / time_diff
	
	vel_x = (vel_left + vel_right / 2)
	vel_y = 0
	vel_th = (vel_right - vel_left) / 0.325

	delta_x = (vel_x * math.cos(th))*time_diff
	delta_y = (vel_x * math.sin(th))*time_diff
	delta_th = vel_th * time_diff

	x_pos += delta_x
	y_pos += delta_y
	th += delta_th

	rospy.loginfo("xpos at %f", x_pos)
	rospy.loginfo("ypos at %f", y_pos)
	rospy.loginfo("th at %f", th)
	rospy.loginfo("lwbdiff at %f", lwb_diff)
	rospy.loginfo("rwbdiff at %f", rwb_diff)
	

	odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
	x_orientation = odom_quat[0]
	y_orientation = odom_quat[1]
	z_orientation = odom_quat[2]
	w_orientation = odom_quat[3]
	quat = Quaternion(x_orientation,y_orientation,z_orientation,w_orientation);

	odom_trans = TransformStamped()
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_pos
	odom_trans.transform.translation.y = y_pos
	odom_trans.transform.translation.z = 0.0;
	#odom_trans.transform.rotation = data.pose[1].orientation;
    
	odom_broadcaster.sendTransform(
	(x_pos,y_pos,0.0),
	(x_orientation,y_orientation,z_orientation,w_orientation),
	current_time,
	"base_link",
	"odom");

	odom = Odometry();
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	odom.pose.pose.position.x = x_pos
	odom.pose.pose.position.y = y_pos
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = quat
	odom.pose.covariance[0] = 0.2;
	odom.pose.covariance[7] = 0.2;
	odom.pose.covariance[35] = 0.4;

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vel_x;
	odom.twist.twist.linear.y = vel_y;
	odom.twist.twist.angular.z = vel_th;

	pub.publish(odom);
	

if __name__ == '__main__': 
  try:
    rospy.init_node('odometry')
    listener = tf.TransformListener()
    motor_topic = rospy.get_param('~motor_topic', '/commands/motor/speed')
    tf_topic = rospy.get_param('~tf_topic', '/tf')
    odom_topic = rospy.get_param('~odom_topic', '/odom') 
    
    lwb_last_rotation = 0
    rwb_last_rotation = 0
    x_pos = 0
    y_pos = 0
    x_orientation = 0
    y_orientation = 0
    z_orientation = 0
    w_orientation = 0
    last_time = rospy.Time.now()
    th = 0
    forward = 1

    rospy.Subscriber(motor_topic, Float64, direction, queue_size=1)
    rospy.Subscriber(tf_topic, tfMessage, calcodom, queue_size=1)
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
