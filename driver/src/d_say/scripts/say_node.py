#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : say_node.py
# Module        : d_say@tinker
# Author        : bss
# Creation date : 2014-07-21
#  Last modified: 2015-04-25, 03:23:33
# Description   : say something through loudspeaker.
#

import sys
import os
import time
import getopt
import rospkg
import rospy
from std_msgs.msg import String
from d_say.srv import *

isPlaying = False

def getSpeechCallback(data):
    global isPlaying
    if isPlaying:
        return
    isPlaying = True
    print(data.data)
    playSound(data.data)
    isPlaying = False

def handle_IsPlaying(req):
    return isPlaying

def Usage():
    print('say_node.py usage:')
    print('speak')
    print('ros topic: /say/sentence')

def playSound(sent):
    sent = sent.strip()
    mp3dir = rospkg.RosPack().get_path('d_say') \
            + '/../../../share/d_say/sounds'
    if os.path.exists(mp3dir + '/' + sent + '.mp3'):
        os.system('mplayer "' + mp3dir + '/' + sent + '.mp3"')
    else:
        sent_speak = sent.replace("'", '')
        os.system("espeak -s 130 --stdout '" + sent_speak + "' | aplay")

def main(argv):
    # Listen to /say
    rospy.init_node('say_node')
    rospy.Subscriber('/say/sentence', String, getSpeechCallback)
    s = rospy.Service('/say/IsPlaying', IsPlaying, handle_IsPlaying)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

