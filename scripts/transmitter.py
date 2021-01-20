#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys

import sys, select, termios, tty

from drive import Drive

moveBindings = {
        '8': '',
        '2': '',
        '4': '',
        '6': '',
    }

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    d = Drive()
    try:
        while(1):
            key = getKey(1)
            if key in moveBindings.keys():
                if key == '8':
                    d.current_speed += 0.2
                    d.keep_going_straight()
                elif key == '2':
                    d.current_speed = 0.0
                elif key == '4':
                    d.go_left()
                elif key == '6':
                    d.go_right()

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)