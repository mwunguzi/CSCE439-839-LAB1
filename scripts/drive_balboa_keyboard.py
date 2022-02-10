#!/usr/bin/env python

import rospy
#from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Header
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.linear.x)
    rate = rospy.Rate(10)

    velo = balboaMotorSpeeds()
    velo.header.stamp = rospy.Time.now()
    #velo.header.stamp = rospy.Time.now()
    #velo.header.frame_id="drive"
    velo.left = 0
    velo.right = 0

    if data.linear.x == 2.0:
        rospy.loginfo('detected press of up-arrow')
        velo.left = 20
        velo.right = 20
        pub.publish(velo)

    elif data.linear.x == -2:
        velo.left = -20
        velo.right = -20
        pub.publish(velo)

    elif data.angular.z == -2:
        velo.left = 10
        velo.right = -10
        pub.publish(velo)

    elif data.angular.z == 2:
        velo.left = -10
        velo.right = 10
        pub.publish(velo)
    
    elif data.angular.z == 0 and data.angular.x == 0:
        velo.left = 0
        velo.right = 0
        pub.publish(velo)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global pub
    pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('drive_with_keyboard', anonymous=True)
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
