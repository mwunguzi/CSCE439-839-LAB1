#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

def ang_vel_callback(data):
    global angle_rate_tar
    angle_rate_tar = data.data*1000 # convert from degrees/s to millidegrees/s

def balboa_callback(data):
    # make these vars global so they persist
    global k_p 
    global k_d 
    global k_i
    global i
    global pub
    global running
    global angle_prev
    global e_prev
    global t_prev
    global angle_rate_tar

        
    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()
    
    if angle_rate_tar == 0:
        return # do nothing when target rate is 0 to avoid unwanted behavior        
              
    angle_now = data.angleX
    t_cur = float(data.header.stamp.secs%1000000)+float(data.header.stamp.nsecs)*10**(-9) # use nsecs and convert to secs to allow fractional seconds
    dt = t_cur - t_prev  # time interval since last message
    t_prev = t_cur

    angular_rate = (angle_now - angle_prev)/dt
    angle_prev = angle_now
    e = angle_rate_tar - angular_rate # proportional error
    d = (e-e_prev)/dt # error derivative
    i += e*dt # error integral
    
    e_prev = e

    speed = int(k_p*e + k_d*d + k_i*i) # PID control

    # Cap speed at +/- 4
    if speed > 4:
        speed = 4
    elif speed < -4:
        speed = -4   
        
    vel_msg.left = -speed
    vel_msg.right = speed
    
    rospy.loginfo(vel_msg)
    pub.publish(vel_msg)
    

def anglePID():
    global pub  
    pub = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size=10)
    rospy.init_node('angleRatePID')
    rospy.Subscriber('balboaLL', balboaLL, balboa_callback)
    rospy.Subscriber('ang_vel', Int16, ang_vel_callback)

    global angle_rate_tar
    global angle_prev
    global angle_rate_prev
    global e_prev
    global i
    global t_prev
    angle_rate_tar = 0
    angle_prev = 0
    angle_rate_prev = 0
    e_prev = 0
    i = 0
    t_prev = 0
    
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
        k_i = 0

    rospy.spin()

if __name__ == '__main__':
    try:
        anglePID()
    except rospy.ROSInterruptException:
        pass
