#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.left = 0
    vel_msg.right = 0
    
    if data.linear.x == 2:
        vel_msg.left = 20
        vel_msg.right = 20
    elif data.linear.x == -2:
        vel_msg.left = -20
        vel_msg.right = -20
    if data.angular.z == 2:
        vel_msg.left -= 10
        vel_msg.right += 10
    elif data.angular.z == -2:
        vel_msg.left += 10
        vel_msg.right -= 10
    
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)

def balboa_teleop():
    global pub
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('balboa_teleop', anonymous=True)
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)

    rospy.spin()        

if __name__ == '__main__':
    try:
        balboa_teleop()
    except rospy.ROSInterruptException:
        pass
