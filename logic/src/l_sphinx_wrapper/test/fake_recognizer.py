#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : fake_recognizer.py
# Module        : l_sphinx_wrapper@tinker
# Author        : bss
# Creation date : 2014-05-09
#  Last modified: 2015-02-02, 19:40:10
# Description   : send /recognizer/output. debug only.
#

import sys
import os
import rospkg
import rospy
from std_msgs.msg import String

def main(argv):
    rospy.init_node('fake_recognizer', anonymous=True)
    pub = rospy.Publisher('/recognizer/output', String, queue_size=1)
    print('Press C-c + Enter to quit.')
    while not rospy.is_shutdown():
        cmd = raw_input().strip()
        pub.publish(cmd)

if __name__ == '__main__':
    main(sys.argv)

