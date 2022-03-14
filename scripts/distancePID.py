#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def target_callback(data):
    global pos_tar 
    global running

    pos_tar = data.data * 1572 # convert from feet to encoder counts
    running = False 


def balboa_callback(data):
    # make these vars global so they persist
    global k_p 
    global k_d 
    global k_i
    global i
    global pos_tar
    global pos_init
    global e_prev
    global t_prev
    global running
    global pub
    global goal_pub
    global end_flag
    global t_end

    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()

    if running == False: # For initialization       
        #pos_tar = float(raw_input("Input target distance (ft): "))*1572 # 1572 encoder counts/ft
        if pos_tar != 0:
            running = True
            pos_init = (data.distanceLeft + data.distanceRight)/2 # average the encoder readings to get pos
            e_prev = pos_tar # previous error = 
            t_prev = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9) - 0.1 # subtract 0.1 since usually sent at 10 Hz
            i = 0  # reset integral term
        else:
            return

    pos_cur = (data.distanceLeft + data.distanceRight)/2 - pos_init # average the encoder readings to get pos
    t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9)
    # add nsecs to secs to allow fractional seconds

    # need to stop this loop once position is reached, otherwise it interferes with rotating
    if pos_tar - 131 <= pos_cur and pos_cur <= pos_tar + 131: # within 1" of target
        if end_flag == False:
            t_end = t_cur
            end_flag = True
        elif t_cur - t_end >= 1: # robot has been within 1" of target for 1 sec
            vel_msg.left = vel_msg.right = 0
            rospy.loginfo(vel_msg)
            pub.publish(vel_msg)
            pos_tar = 0
            running = False # stop running this loop until a new target is received
            goal_pub.publish(True) # publish that position has been reached
            return
    else:
        end_flag = False

    dt = t_cur - t_prev   # time interval since last message
    e = pos_tar - pos_cur # error
    d = (e - e_prev)/dt   # derivative of error
    i += e*dt             # integral of error

    speed = int(k_p*e + k_d*d + k_i*i) # PID control

    # Cap speed at +/- 3
    if speed > 3:
        speed = 3
    elif speed <-3:
        speed = -3

    vel_msg.left = speed
    vel_msg.right = speed

    rospy.loginfo('current position: %s', pos_cur)
    rospy.loginfo('proportional error term: %s', e)
    rospy.loginfo('speed set: %s',speed)
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    
    t_prev = t_cur
    e_prev = e
   

def distancePID():
    global pub
    global running
    global pos_tar
    global goal_pub

    running = False
    pos_tar = 0

    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    goal_pub = rospy.Publisher('posReached', Bool, queue_size=1)
    rospy.init_node('distancePID')
    
    rospy.Subscriber('targetInputDist', Float32, target_callback)
    rospy.Subscriber('balboaLL', balboaLL, balboa_callback)

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

    rospy.spin()

if __name__ == '__main__':
    try:
        distancePID()
    except rospy.ROSInterruptException:
        pass
