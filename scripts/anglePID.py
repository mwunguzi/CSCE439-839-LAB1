#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    # make these vars global so they persist
    global k_p 
    global k_d 
    global k_i
    global i
    global pub
    global running
    global angle_init
    global angle_prev
    global pos_tar
    global e_prev
    ang_vel = Int16()
        
    if running == False: # Only run if not currently moving towards the target angle
        if pos_tar == 0:
            user_in = float(raw_input("Input target angle (deg): "))
            pos_tar = (user_in * 1000) # multiply by 1000 because the angle is given in millidegrees
        else: # this will run only once, the iteration after the target is given
            running = True
            angle_init = data.angleX
            print(pos_tar + angle_init)
            e_prev = pos_tar
            angle_prev = 0
            i = 0  # reset integral term
        return # wait until next time to start sending signals

    angle_now = data.angleX - angle_init # how far the robot has turned
    rospy.loginfo('angle reading now: %s', angle_now)

    e = pos_tar - angle_now  # proportional error term 
    d = e - e_prev           # derivative error term
    i += e                   # integral error term

    rospy.loginfo(e)
    
    angle_prev = angle_now
    e_prev = e
    
    ang_vel = k_p*e + k_d*d + k_i*i # PID control

    # Cap speed at +/- 60 degrees/s
    if ang_vel > 60:
        ang_vel  = 60
    elif ang_vel <-60:
        ang_vel = -60
        
    rospy.loginfo('speed set: %s', ang_vel)
    pub.publish(ang_vel)
    

def anglePID():
    global pub
    global running
    global pos_tar
    
    running = False
    pos_tar = 0
    
    pub = rospy.Publisher('ang_vel', Int16, queue_size=10)
    rospy.init_node('anglePID')
    rospy.Subscriber('balboaLL', balboaLL, callback)

    global k_p 
    global k_d 
    global k_i

    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.002
    if rospy.has_param('~rCtrl/D'):
        k_d = rospy.get_param('~rCtrl/D')
    else:
        k_d = 0
    if rospy.has_param('~rCtrl/I'):
        k_i = rospy.get_param('~rCtrl/I')
    else:
        k_i = 0.000001
    print(k_p)
    print(k_d)
    print(k_i)

    rospy.spin()

if __name__ == '__main__':
    try:
        anglePID()
    except rospy.ROSInterruptException:
        pass
