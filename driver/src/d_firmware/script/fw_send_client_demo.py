#!/usr/bin/env python

__author__ = 'gjc'

import sys
sys.path.append('../../../devel/lib/python2.7/dist-packages')

import rospy
from fw_send.srv import *


def client_demo(command):
    rospy.wait_for_service('send_command')
    try:
        send_command = rospy.ServiceProxy('send_command', SendCommand)
        response = send_command(command)
        return response.echo
    except rospy.ServiceException as e:
        sys.stderr.write(
            "send_command service call failed: %s" % e.message + '\n')
    return ""


if __name__ == '__main__':
    if len(sys.argv) == 2:
        print "fw_send_client_demo"
        print "command" + sys.argv[1]
        print client_demo(sys.argv[1])
