#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : tinker_start.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-02-01
#  Last modified: 2015-02-01, 21:16:12
# Description   : Startup script for tinker, main body.
#

import sys
import os
import rospy
import ConfigParser
from std_msgs.msg import String

say_pub = rospy.Publisher('/say/sentence', String, queue_size=1)
cwd = os.path.split(os.path.realpath(__file__))[0];
config = ConfigParser.SafeConfigParser(
        {'xdotool_sleep_time': '4'}
)

def Speak(text):
    print(text)
    say_pub.publish(str(text))

def Wait():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

def StartNewTab(cmd):
    t = config.get('scripts', 'xdotool_sleep_time')
    os.system('xdotool key ctrl+shift+t ; sleep ' + str(t) + ' ; '
            + 'xdotool type "' + cmd + '" ; xdotool key "Return"')

def SwitchBack():
    os.system('xdotool key alt+1 ; sleep 1')

def main(argv):
    print(cwd)
    config.read(cwd + '/settings.ini')

    StartNewTab('roscore')
    rospy.init_node('tinker_start', anonymous=True)
    StartNewTab('rosrun d_say say_node.py')

    SwitchBack()
    Speak('Hello world')
    print('Press C-c to exit...')
    Wait()
    print('Bye!')

if __name__ == '__main__':
    main(sys.argv)

