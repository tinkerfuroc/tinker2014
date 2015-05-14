#!/usr/bin/env python

__author__ = 'gjc'

import sys
sys.path.append('../../../devel/lib/python2.7/dist-packages')

import rospy
import fw_send
from fw_send.srv import *
from threading import Lock
from socket import socket,AF_INET,SOCK_DGRAM

send_lock = Lock()
ECHO_SERVER_ADDRESS = "192.168.2.10"
ECHO_PORT = 7


def send_command_handler(request):
    print request.command
    try:
        send_lock.acquire()
        s = socket(family=AF_INET, type=SOCK_DGRAM)
        s.connect((ECHO_SERVER_ADDRESS, ECHO_PORT))
        s.settimeout(0.05)
        s.sendall(request.command)
        data = s.recv(1024)
        s.close()
        return data
    except Exception as e:
        sys.stderr.write("fw_send exception:" + e.message + '\n')
    finally:
        send_lock.release()
    return ""


def init_server():
    rospy.init_node('send_command_server')
    service = rospy.Service('send_command', SendCommand, send_command_handler)
    print "fw_send: server ready"
    rospy.spin()


if __name__ == '__main__':
    init_server()

