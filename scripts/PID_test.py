#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds



def callback(data):
    global k_p 
    global k_d 
    global k_i
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
        pos_tar = float(raw_input("Input target distance (ft): "))*1572 # 2156 encoder counts/ft
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

    # Cap speed at +/- 10 to guard the motor to run at the high speed
    if speed > 10:
        speed = 10
    elif speed <-10:
        speed = -10
        
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

    global k_p 
    global k_d 
    global k_i

    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.009
    if rospy.has_param('~rCtrl/D'):
        k_d = rospy.get_param('~rCtrl/D')
    else:
        k_d = 0
    if rospy.has_param('~rCtrl/I'):
        k_i = rospy.get_param('~rCtrl/I')
    else:
        k_i = 0
    print(k_p)
    print(k_d)
    print(k_i)
    
    
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