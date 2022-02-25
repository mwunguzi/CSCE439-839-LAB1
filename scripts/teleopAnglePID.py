#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callbackVel(data):
    global pos_tar #variable to store the position target
    global running #variable to check if the PID controller is still running

    teleopVel = Twist()
    running = False
    if data.data == 2:
        pos_tar = 30000 # if the right arrow key is pressed it will be an input of 30 degrees. we multiply by 1000 to put it in millidegrees

    elif data.data == -2:
        pos_tar = -30000 # if the left arrow key is pressed it will be an input of -30 degrees. we multiply by 1000 to put it in millidegrees

    else:
        pos_tar = 0

    rospy.Subscriber('balboaLL', balboaLL, callback)

def callback(data):
    global k_p 
    global k_d 
    global k_i
    global pub
    global running
    global angle_prev
    global pos_tar
    global e_prev
    global e_curr
    global pos_curr
    global end_flag
    global user_in
    global t_cur
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    
    if running == False: # Only run if not currently moving towards the target angle
        angle_prev = data.angleX
        running = True
        pos_curr = 0
        e_prev = 0
        e_curr = 0
        t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
        rospy.loginfo('time.now: %s', t_cur)
        rospy.loginfo('target angle: %s', pos_tar)
        rospy.loginfo('angle reading prev: %s', angle_prev)

    elif running == True:
        angle_now = data.angleX
        t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
        rospy.loginfo('time.now: %s', t_cur)
        rospy.loginfo('angle reading now: %s', angle_now)
        rospy.loginfo('angle reading prev: %s', angle_prev)
        rospy.loginfo('target angle: %s', pos_tar)

        pos_curr = pos_curr + (angle_now - angle_prev)

        angle_prev = angle_now
        e_prev = e_curr 
        e_curr = pos_tar-pos_curr

    if e_curr == 0 and e_prev == 0 :
        e_p = pos_tar # proportional error term 
        e_d = e_curr - e_prev    # derivative error term
    else:
        e_p = pos_tar - pos_curr # proportional error term 
        e_d = e_curr - e_prev    # derivative error term

    speed = int(k_p*e_p + k_d*e_d) # PID control

    # Cap speed at +/- 8
    if speed > 4:
        speed = 4
    elif speed <-4:
        speed = -4
    
        
    vel_msg.left = -speed
    vel_msg.right = speed
    
        
    rospy.loginfo('current angle: %s', pos_curr)
    rospy.loginfo('proportionla error term: %s', e_p)
    rospy.loginfo('speed set: %s',speed)
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    

def anglePID():
    global pub
    global running
    global count
    
    running = False
    count = 0
    
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('teleopAnglePID')
    rospy.Subscriber('angularPID', Int16, callbackVel)

    global k_p 
    global k_d 
    global k_i

    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.00009
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
        anglePID()
    except rospy.ROSInterruptException:
        pass
