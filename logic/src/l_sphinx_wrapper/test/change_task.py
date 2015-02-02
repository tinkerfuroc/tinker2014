#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : change_task.py
# Module        : l_sphinx_wrapper@tinker
# Author        : bss
# Creation date : 2015-02-02
#  Last modified: 2015-02-02, 20:05:24
# Description   : 
#

import sys
import os
import time
import getopt
import rospy
from l_sphinx_wrapper.srv import *

def Usage():
    print('change_task.py in l_sphinx_wrapper usage:')
    print('change task name/model of l_sphinx_wrapper')
    print('-h,--help: print help message.')
    print('-t,--task: change to a task. for example: -t tasks/answer')
    print('--fsg: change fsg model. for example: --fsg share/xx/x.fsg')
    print('--dict: change dict.')
    print('--fsg & --dict must use together.')

def change_task_client(taskname):
    print('try to set task to %s' % taskname)
    rospy.wait_for_service('/recognizer/change_task')
    try:
        change_task = rospy.ServiceProxy('/recognizer/change_task',
                ChangeTask)
        resp = change_task(taskname)
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

def change_fsg_client(fsgname, dictname):
    print('try to set fsg to %s' % fsgname)
    print('try to set dict to %s' % dictname)
    rospy.wait_for_service('/recognizer/change_fsg')
    try:
        change_fsg = rospy.ServiceProxy('/recognizer/change_fsg',
                ChangeFSG)
        resp = change_fsg(fsgname, dictname)
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

def main(argv):
    try:
        opts, argv = getopt.getopt(argv[1:], 't:h',
                ['task=', 'fsg=', 'dict=', 'help'])
    except getopt.GetoptError, e:
        print('Error: %s'%e)
        Usage()
        sys.exit(2)
    except Exception, e:
        print('Error: %s'%e)
        Usage()
        sys.exit(1)

    taskname = ''
    fsgname = ''
    dictname = ''
    for o, a in opts:
        if o in ('-h', '--help'):
            Usage()
            sys.exit(0)
        if o in ('-t', '--task'):
            taskname = a
        if o in ('--fsg'):
            fsgname = a
        if o in ('--dict'):
            dictname = a
    if taskname != '':
        if fsgname != '' or dictname != '':
            print("Error: don't set task and model at the same time.")
            sys.exit(2)
        else:
            change_task_client(taskname)
    elif fsgname != '' and dictname != '':
        change_fsg_client(fsgname, dictname)
    else:
        print('Error: must set task or model')
        Usage()
        sys.exit(2)

if __name__ == '__main__':
    main(sys.argv)

