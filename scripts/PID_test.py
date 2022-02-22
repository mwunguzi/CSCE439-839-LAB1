#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds



def callback(data):
    k_p = 0.009
    k_d = 0.0
    global pos_curr
    global pos_tar
    global e_prev
    global e_curr
    global running
    global encoder_prev
    #global pub

    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()

    rospy.loginfo('Running Flag set to: %s', running)

    if running == False: # Only run if not currently moving towards a target
        pos_tar = float(raw_input("Input target distance (ft): "))*1572 #1572  encoder counts/ft
        running = True
        encoder_prev = abs((data.distanceLeft + data.distanceRight)/2)
        pos_curr = 0
        e_curr = 0
        e_prev = 0

    elif running == True:
        encoder_now = abs((data.distanceLeft + data.distanceRight)/2)
        rospy.loginfo('encoder reading now: %s', encoder_now)
        rospy.loginfo('encoder reading prev: %s', encoder_prev)

        pos_curr += (encoder_now - encoder_prev)
        
        
        encoder_prev = encoder_now
        e_prev = e_curr 
        e_curr = pos_tar-pos_curr
    

    if e_curr == 0 and e_prev == 0 :
        e_p = pos_tar # proportional error term 
        e_d = e_curr - e_prev    # derivative error term
    else:
        e_p = pos_tar - pos_curr # proportional error term 
        e_d = e_curr - e_prev    # derivative error term

    speed = int(k_p*e_p + k_d*e_d) # PID control

    # Cap speed at +/- 30 to guard the motor to run at the high speed
    
        
    vel_msg.left = speed
    vel_msg.right = speed

    
    rospy.loginfo('current position: %s', pos_curr)
    rospy.loginfo('proportionla error term: %s', e_p)
    rospy.loginfo('speed set: %s',speed)
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    
    

def distancePID():
    global pub
    global running
    global count

    running = False 
    count = 0


    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('distancePID')
    rospy.Subscriber('balboaLL', balboaLL, callback)
    
    
    if count == 0 :
        running = False
        count+=1
    elif count > 0 :
        running = True
    

    rospy.spin()

if __name__ == '__main__':
    try:
        distancePID()
    except rospy.ROSInterruptException:
        pass
