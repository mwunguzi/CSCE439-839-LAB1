#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def timeout():
    print("No message received for 1 seconds")
    stop_bal()# stop the robot

def callback(data):
    global timer
    # use timeout if 0.5 s elapse with no message to stop the robot
    timer = threading.Timer(0.5,timeout)
    timer.start()
    
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    vel_msg.left = 0
    vel_msg.right = 0
    
    if data.linear.x == 2: # forward
        vel_msg.left = 10
        vel_msg.right = 10
    elif data.linear.x == -2: # backward
        vel_msg.left = -10
        vel_msg.right = -10
    if data.angular.z == 2: # left
        vel_msg.left = -5
        vel_msg.right = 5
    elif data.angular.z == -2: # right
        vel_msg.left = 5
        vel_msg.right = -5
    
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    
def stop_bal():
    velo_msg = balboaMotorSpeeds()
    velo_msg.header.stamp = rospy.Time.now()
    velo_msg.left = 0
    velo_msg.right = 0
    
    rospy.loginfo(velo_msg)
    pub.publish(velo_msg)
    
def balboa_teleop():
    global pub
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('balboa_teleop', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback)

    rospy.spin()        

if __name__ == '__main__':
    try:
        balboa_teleop()
    except rospy.ROSInterruptException:
        pass

