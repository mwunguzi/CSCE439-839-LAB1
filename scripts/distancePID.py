#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    k_p = 0.005
    k_d = -0.0
    global running
    global pos_init
    global pos_tar
    global e_prev
    global t_prev
    global t_end
    global end_flag
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    
    if running == False: # Only run if not currently moving towards a target
        pos_tar = float(raw_input("Input target distance (ft): "))*2156 # 2156 encoder counts/ft
        running = True
        pos_init = (data.distanceLeft + data.distanceRight)/2
        e_prev = pos_tar
        t_prev = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9) # use nsecs and convert to secs to allow fractional seconds
        return # wait until next time to start sending signals
    
    pos_cur = (data.distanceLeft + data.distanceRight)/2 - pos_init
    t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
    
    if pos_tar - 180 <= pos_cur and pos_cur <= pos_tar + 180: # within 1" of target
        if end_flag == False:
            t_end = t_cur
            end_flag = True
        elif t_cur - t_end >= 1: # robot has been within 1" of target for 1 sec
            vel_msg.left = vel_msg.right = int(k_p*e + k_d*d) # PID control
            rospy.loginfo(vel_msg)
            pub.publish(vel_msg)
            running = False
            return
    else:
        end_flag = False
    
    dt = t_cur - t_prev   # time interval since last message
    e = pos_tar - pos_cur # error
    d = (e - e_prev)/dt   # derivative of error

    speed = int(k_p*e + k_d*d) # PID control
    # Cap speed at +/- 30
    if speed > 30:
        speed = 30
    elif speed <-30:
        speed = -30
        
    vel_msg.left = vel_msg.right = speed
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    
    t_prev = t_cur
    e_prev = e
    

def distancePID():
    global pub
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('distancePID')
    rospy.Subscriber('balboaLL', balboaLL, callback)
 
    global running
    running = False

    rospy.spin()

if __name__ == '__main__':
    try:
        distancePID()
    except rospy.ROSInterruptException:
        pass
