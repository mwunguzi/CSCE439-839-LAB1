#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.linear.x)
    pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)
    vel = Twist()
    if data.linear.x == 2:
        pub.publish(drive(20, 20))

    if data.linear.x == -2:
        pub.publish(drive(-20, -20))

    if data.angular.z == -2:
        pub.publish(drive(25, 10))

    if data.angular.z == 2:
        pub.publish(drive(10, 25))
    
    if data.angular.z == 0 and data.angular.z == 0:
        pub.publish(drive(0,0))


def drive(lmotor, rmotor):
    velo = balboaMotorSpeeds()
    velo.left = lmotor
    velo.right = rmotor

    return velo


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_with_keyboard', anonymous=True)

    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
