#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def command():

    while not rospy.is_shutdown():
        user_input = int(raw_input("input target distance(ft): "))
        pub = rospy.Publisher('targetInput', Int16, queue_size=10)
        rospy.init_node('keyboardCommand', anonymous=True)
        rate = rospy.Rate(10)
        pub.publish(user_input)

if __name__ == '__main__' :
    try:
        command()
    except rospy.ROSInterruptException:
        pass