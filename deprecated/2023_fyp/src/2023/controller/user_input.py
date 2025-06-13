#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import sys


def main():
    rospy.init_node('user_input')
    user_input_pub = rospy.Publisher('/user_input/state', String, queue_size=10)
    while not rospy.is_shutdown():
        message = String()
        message.data = input()
        user_input_pub.publish(message)


if __name__ == "__main__": 
    main()
