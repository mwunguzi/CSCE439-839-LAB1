#!/usr/bin/env python
import rospy
import threading # Needed for Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds


def callback(data):
    global pub1 #for publishing on the distance PID topic
    global pub2 #for publishing on the angle PID topic
    global telekey #variable to store the pressed key
  
    if data.linear.x == 2:
        telekey = 2 
        pub1.publish(telekey)

    elif data.linear.x == -2:
        telekey = -2 
        pub1.publish(telekey)

    elif data.linear.z == 2:
        telekey = 2 
        pub2.publish(telekey)

    elif data.linear.z == -2:
        telekey = -2
        pub2.publish(telekey)

    """ else:
        telekey = 0
        pub1.publish(telekey)
        pub2.publish(telekey) """
        

def translate():
    global pub1
    global pub2

    rospy.init_node('teleopTranslation', anonymous=True)
    pub1 = rospy.Publisher('distPID', Int16 , queue_size=10) #creating the topic for the distance PID node
    pub2 = rospy.Publisher('angularPID', Int16 , queue_size=10) #creating the topic for the angle PID node
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callback) #subscribing to the cmd_vel topic
    

    rospy.spin()        

if __name__ == '__main__':
    try:
        translate()
    except rospy.ROSInterruptException:
        pass