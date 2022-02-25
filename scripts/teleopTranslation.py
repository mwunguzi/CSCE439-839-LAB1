#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from balboa_core.msg import balboaLL
from balboa_core.msg import balboaMotorSpeeds


def callback(data):
    global pos_pub #for publishing on the distance PID topic
    global ang_pub #for publishing on the angle PID topic
  
    if data.linear.x == 2: # forward
        telekey = 1 # set target pos to 1 ft
        pos_pub.publish(telekey)

    elif data.linear.x == -2: # backward
        telekey = -1 # set target pos to -1 ft
        pos_pub.publish(telekey)

    elif data.angular.z == 2: # left
        telekey = 90 # set target angle to 90 degrees
        ang_pub.publish(telekey)

    elif data.angular.z == -2: # right
        telekey = -90 # set target angle to -90 degrees
        ang_pub.publish(telekey)

    """ else:
        telekey = 0
        pub1.publish(telekey)
        pub2.publish(telekey) """
        

def translate():
    global pos_pub
    global ang_pub

    rospy.init_node('teleopTranslation', anonymous=True)
    pos_pub = rospy.Publisher('targetInputDist', Int16, queue_size=10)
    ang_pub = rospy.Publisher('targetInputAng', Int16, queue_size=10)
    rospy.Subscriber('PID_cmd_vel', Twist, callback) #subscribing to the cmd_vel topic

    print("teleopTranslate active: open turtle_teleop_key to control robot via arrow keys with PID")

    rospy.spin()        

if __name__ == '__main__':
    try:
        translate()
    except rospy.ROSInterruptException:
        pass