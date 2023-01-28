#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import curses
import os

def stop(vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


def main(win):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('bot_controller', anonymous=True)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    stop(vel_msg)

    win.nodelay(True)
    key = ""
    win.clear()
    win.addstr("Detected key: ")

    while not rospy.is_shutdown():
        try:
            key = win.getkey()
            win.clear()
            win.addstr("Detected key: ")
            win.addstr(str(key))
            if str(key) == "w":
                vel_msg.linear.x = 0.5
            elif str(key) == "s":
                vel_msg.linear.x = -0.5
            elif str(key) == "a":
                vel_msg.angular.z = 2
            elif str(key) == "d":
                vel_msg.angular.z = -2
            elif str(key) == "f":
                stop(vel_msg)
            if key == os.linesep:
                break
        except Exception as e:
            pass
        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
