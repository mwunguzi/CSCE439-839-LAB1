#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds
import random

def target_callback(data):
    global running
    global pos_tar

    pos_tar = data.data*1000 # convert from degrees to millidegrees
    running = False


def balboa_callback(data):
    # make these vars global so they persist
    global k_p 
    global k_d 
    global k_i
    global i
    global pub
    global running
    global angle_init
    global pos_tar
    global e_prev
    global t_prev
    global goal_pub
    global end_flag
    global t_end

    msg = balboaMotorSpeeds()
    msg.header.stamp = rospy.Time.now()
    
    if running == False: # for initialization
        """ if pos_tar == 0:
            user_in = float(raw_input("Input target angle (deg): "))
            pos_tar = user_in*1000 # multiply by 1000 because the angle is given in millidegrees """
        if pos_tar != 0: # this will run the iteration after the target is given
            running = True
            angle_init = data.angleX
            e_prev = pos_tar
            t_prev = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9) # add nsecs to secs to allow fractional seconds
            i = 0  # reset integral term
        return # wait until next time to start sending signals

    angle_now = data.angleX - angle_init # how far the robot has turned
    rospy.loginfo('angle reading now: %s', angle_now)

    t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
    # get nsecs and add to secs to allow fractional seconds

    if pos_tar - 2000 <= angle_now and angle_now <= pos_tar + 2000: # within 2 degrees of target
        if end_flag == False:
            t_end = t_cur
            end_flag = True
        elif t_cur - t_end >= 1: # robot has been within 2 degrees of target for 1 sec
            msg.left = msg.right = 0
            pub.publish(msg) # stop moving
            pos_tar = 0
            running = False # stop running this loop until a new target is received
            goal_pub.publish(True) # publish that position has been reached
            return
    else:
        end_flag = False

    dt = t_cur - t_prev   # time interval since last message
    e = pos_tar - angle_now # error
    d = (e - e_prev)/dt   # derivative of error
    i += e*dt             # integral of error

    rospy.loginfo(e)
    
    e_prev = e
    
    ang_vel = k_p*e + k_d*d + k_i*i # PID control

    # Cap speed at +/- 4
    if ang_vel > 4:
        ang_vel  = 4
    elif ang_vel <-4:
        ang_vel = -4

    elif ang_vel > -1 and ang_vel < 0:
        if random.random() < abs(ang_vel):
            ang_vel = -1
    elif ang_vel < 1 and ang_vel > 0:
        if random.random() < ang_vel:
            ang_vel = 1
    
    
    msg.left = -ang_vel
    msg.right = ang_vel
    rospy.loginfo('speed set: %s', ang_vel)
    pub.publish(msg)
    

def anglePID():
    global pub
    global goal_pub
    global running
    global pos_tar
    
    running = False
    pos_tar = 0
    
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=1)
    goal_pub = rospy.Publisher('angleReached', Bool, queue_size=1)
    rospy.init_node('anglePID')
    rospy.Subscriber('targetInputAng',Int16, target_callback)
    rospy.Subscriber('balboaLL', balboaLL, balboa_callback)

    global k_p 
    global k_d 
    global k_i

    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.0001
    if rospy.has_param('~rCtrl/D'):
        k_d = rospy.get_param('~rCtrl/D')
    else:
        k_d = 0
    if rospy.has_param('~rCtrl/I'):
        k_i = rospy.get_param('~rCtrl/I')
    else:
        k_i = 0.0000

    rospy.spin()

if __name__ == '__main__':
    try:
        anglePID()
    except rospy.ROSInterruptException:
        pass
