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
    global pub
    global running
    global angle_prev
    global pos_tar
    global e_prev
    global e_curr
    global pos_curr
    global end_flag
    global user_in
    global t_prev
    global t_cur
    global angular_rate
    global speed
    
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    
    if running == False: # Only run if not currently moving towards the target angle
        angle_prev = data.angleX
        user_in = float(raw_input("Input target anglar rate (deg/s): "))
        #if user_in >= 360: #if the user inputs an angle bigger than 360
            #user_in = user_in - 360
        pos_tar = (user_in * 1000) # we multiply by 1000 because the angular rate is given in millidegrees/s
        running = True
        angular_rate = 0
        e_prev = 0
        e_curr = 0
        
        t_prev = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
        rospy.loginfo('time.now: %s', t_prev)
        rospy.loginfo('target angle: %s', pos_tar)
        rospy.loginfo('angular rate prev: %s', (angle_prev/t_prev))

    elif running == True:
        angle_now = data.angleX
        t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
        rospy.loginfo('time prev: %s', t_prev)
        dt = t_cur - t_prev
        t_prev = t_cur

        angular_rate = (angle_now - angle_prev)/dt

        angle_now_rate=angle_now/dt

        angle_prev_rate=angle_prev/dt

        angle_prev = angle_now
        e_prev = e_curr 
        e_curr = pos_tar - angular_rate

        rospy.loginfo('time.now: %s', t_cur)
        
        rospy.loginfo('angle rate reading now: %s', angle_now_rate)
        rospy.loginfo('angle rate reading prev: %s', angle_prev_rate)
        rospy.loginfo('current angular rate: %s', angular_rate)
        rospy.loginfo('target angular rate: %s', pos_tar)

    if e_curr == 0 and e_prev == 0 :
        e_p = pos_tar # proportional error term 
        e_d = e_curr - e_prev    # derivative error term
        speed = int(k_p*e_p + k_d*e_d) # PID control
    else:
        e_p = pos_tar - angular_rate # proportional error term 
        e_d = e_curr - e_prev    # derivative error term
        speed += int(k_p*e_p + k_d*e_d) # PID control

    

    # Cap speed at +/- 8
    """ if speed > 4:
        speed = 4
    elif speed <-4:
        speed = -4 """
    
        
    vel_msg.left = -speed
    vel_msg.right = speed
    
        
    #rospy.loginfo('current angular rate: %s', angular_rate)
    rospy.loginfo('proportionla error term: %s', e_p)
    rospy.loginfo('differential error term: %s', e_d)
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
    rospy.init_node('angleRatePID')
    rospy.Subscriber('balboaLL', balboaLL, callback)

    global k_p 
    global k_d 
    global k_i

    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.00003
    if rospy.has_param('~rCtrl/D'):
        k_d = rospy.get_param('~rCtrl/D')
    else:
        k_d = 0.00001
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
