#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def command():

    while not rospy.is_shutdown():
        user_input = int(raw_input("input target distance (1-9ft) or angle (10+ degrees): "))
        pub = rospy.Publisher('targetInputDist', Int16, queue_size=10)
        pub2 = rospy.Publisher('targetInputAng', Int16, queue_size=10)
        rospy.init_node('keyboardCommand', anonymous=True)
        rate = rospy.Rate(10)
        if abs(user_input) <= 9:
            pub.publish(user_input)
        elif abs(user_input) > 10:
            pub2.publish(user_input)

if __name__ == '__main__' :
    try:
        command()
    except rospy.ROSInterruptException:
        pass