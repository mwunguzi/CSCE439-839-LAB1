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
    print("Message received")
    #timer.cancel()
    timer = threading.Timer(0.5,timeout)
    timer.start()
    
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
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    
    timer = threading.Timer(0.5,timeout) # If 1 seconds elapse, call timeout()
    timer.start()

    rospy.spin()        

if __name__ == '__main__':
    try:
        balboa_teleop()
    except rospy.ROSInterruptException:
        pass

