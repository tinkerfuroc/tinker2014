#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : tinker_start.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-02-01
#  Last modified: 2015-02-01, 22:50:35
# Description   : Startup script for tinker, main body.
#

import sys
import os
import rospy
import ConfigParser
from std_msgs.msg import String
from d_say.srv import *

rate = None
say_pub = rospy.Publisher('/say/sentence', String, queue_size=1)
cwd = os.path.split(os.path.realpath(__file__))[0];
config = ConfigParser.SafeConfigParser(
        {'xdotool_sleep_time': '4'}
)

def d_say_IsPlaying():
    rospy.wait_for_service('/say/IsPlaying')
    try:
        func_IsPlaying = rospy.ServiceProxy('/say/IsPlaying', IsPlaying)
        resp = func_IsPlaying()
        return resp.playing
    except rospy.ServiceException, e:
        print('fail: %s'%e)
        return False

def Speak(text):
    print('speak: "' + text + '"')
    if d_say_IsPlaying():
        print('waiting for d_say...')
        while (not rospy.is_shutdown()) and d_say_IsPlaying():
            rate.sleep()
    say_pub.publish(str(text))
    rate.sleep()

def Wait():     # wait for C-c
    while not rospy.is_shutdown():
        rate.sleep()

def StartNewTab(cmd):
    t = config.get('scripts', 'xdotool_sleep_time')
    os.system('xdotool key ctrl+shift+t ; sleep ' + str(t) + ' ; '
            + 'xdotool type "' + cmd + '" ; xdotool key "Return"')

def SwitchBack():   # switch back to this tab
    os.system('xdotool key alt+1 ; sleep 1')

def Run(filename):      # run tinkerstart script
    print('run ' + filename + '.tinkerstart')
    filepath = cwd + '/scripts/' + filename + '.tinkerstart'
    f = open(filepath, 'r')
    for line in f.readlines():
        line = line.strip()
        if (line.startswith('#') or line == ''):
            continue
        if line.startswith('print'):
            value = line[len('print'):].strip()
            print('print: ' + value)
        if line.startswith('speak'):
            value = line[len('speak'):].strip()
            if value.startswith('"') and value.startswith('"'):
                value = value[1:-1]
                Speak(value)
        #print(line)

def main(argv):
    print(cwd)
    config.read(cwd + '/settings.ini')

    #StartNewTab('roscore')
    rospy.init_node('tinker_start', anonymous=True)
    #StartNewTab('rosrun d_say say_node.py')

    SwitchBack()
    global rate
    rate = rospy.Rate(10)
    Run('main')
    print('Press C-c to exit...')
    Wait()
    print('Bye!')

if __name__ == '__main__':
    main(sys.argv)

