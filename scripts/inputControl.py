#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def command():
    pos_pub = rospy.Publisher('targetInputDist', Int16, queue_size=10)
    ang_pub = rospy.Publisher('targetInputAng', Int16, queue_size=10)
    rospy.init_node('positionCommand')
    while not rospy.is_shutdown():
        # prompt user for position input
        user_input = int(raw_input("input target distance (1-9ft) or angle (10+ degrees): ")) 

        # if abs(input) is 9 or less, move that many feet, otherwise, rotate that many degrees
        if abs(user_input) <= 9:
            pos_pub.publish(user_input)
        elif abs(user_input) > 10:
            ang_pub.publish(user_input)

if __name__ == '__main__' :
    try:
        command()
    except rospy.ROSInterruptException:
        pass