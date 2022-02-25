#!/usr/bin/env python

import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds

#global teleopKey # variable to store the pressed key

def callbackVel(data):
    #global teleopKey # variable to store the pressed key

    #rospy.init_node('PIDteleopTranslate')
    #rospy.loginfo('I heard: %s', data.linear.x)
    teleopVel = Twist()
    """ if data.linear.x == 2:
        teleopKey = 1 # if the up arrow key is pressed it will be an input of 1 foot

    if data.linear.x == -2:
        teleopKey = -1 # if the down arrow key is pressed it will be an input of 1 foot (backward)

    #return teleopKey """

def translateKeyPress():
    pressedKey = Twist()

    return pressedKey.linear.x



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
    global teleopKey # variable to store the pressed key
    global user_in
    #global pub

    vel_msg = balboaMotorSpeeds()
    vel_msg.header.stamp = rospy.Time.now()

    rospy.loginfo('Running Flag set to: %s', running)
    

    if running == False: # Only run if not currently moving towards a target
        teleopKey = translateKeyPress

        if teleopKey == 2:
            rospy.loginfo('I heard: %s', teleopKey)
            user_in = 1 # if the up arrow key is pressed it will be an input of 1 foot

        elif teleopKey == -2:
            rospy.loginfo('I heard: %s', teleopKey)
            user_in = -1 # if the down arrow key is pressed it will be an input of 1 foot (backward)

        else:
            user_in = 0

        pos_tar = float(user_in) * 1572
        #pos_tar = float(raw_input("Input target distance (ft): "))*1572 # 2156 encoder counts/ft
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
    rospy.init_node('PIDteleopTranslate')
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callbackVel)
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


#if we receive linear.x == 2: #the up arrow key is pressed
#   send 1 ft to the distance PID #this might mean making the PID node subscribe to the cmd_vel topic
#   or we might not need this translation node and we can go to then distance node, make it subscribe to cmd_vel and give the PID
#controller one foot when the up key is pressed and -1 when the down key is pressed. Remember to use threading to make the robot stop.