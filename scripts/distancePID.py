#!/usr/bin/env python

import rospy
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def callback(data):
    # make these vars global so they persist
    global k_p
    global k_d
    global k_i
    global i
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
        i = 0  # reset integral term
        return # wait until next time to start sending signals
    
    pos_cur = (data.distanceLeft + data.distanceRight)/2 - pos_init
    t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
    
    if pos_tar - 180 <= pos_cur and pos_cur <= pos_tar + 180: # within 1" of target
        if end_flag == False:
            t_end = t_cur
            end_flag = True
        elif t_cur - t_end >= 1: # robot has been within 1" of target for 1 sec
            vel_msg.left = vel_msg.right = 0
            rospy.loginfo(vel_msg)
            pub.publish(vel_msg)
            running = False
            return
    else:
        end_flag = False
    
    dt = t_cur - t_prev   # time interval since last message
    e = pos_tar - pos_cur # error
    d = (e - e_prev)/dt   # derivative of error
    i += e*dt # integral of error

    speed = int(k_p*e + k_d*d + k_i*i) # PID control
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

    global k_p
    global k_d
    global k_i
    # set PID constants from parameters
    if rospy.has_param('~rCtrl/P'):
        k_p = rospy.get_param('~rCtrl/P')
    else:
        k_p = 0.005
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
 
    global running
    running = False

    rospy.spin()

if __name__ == '__main__':
    try:
        distancePID()
    except rospy.ROSInterruptException:
        pass
