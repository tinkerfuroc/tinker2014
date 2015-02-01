#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : tinker_start.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-02-01
#  Last modified: 2015-02-02, 00:58:19
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
gets_str = ''

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

def RunFile(filename):      # run tinkerstart script
    print('run ' + filename + '.tinkerstart')
    filepath = cwd + '/scripts/' + filename + '.tinkerstart'
    try:
        f = open(filepath, 'r')
        lines = f.readlines()
    except:
        print('Error: cannot open %s.tinkerstart'%filename)
        return
    RunLines(lines, filename, 0)

def RunLines(lines, filename, linenumber):        # run script in memory
    nesting_level = 0
    nesting_type = ''
    values = []
    count = 0
    linenumberOfNesting = 0
    for originLine in lines:
        count += 1
        handled = False
        line = originLine.strip()
        global gets_str
        line = line.replace('$(str)', gets_str)
        if (line.startswith('#') or line == ''):
            continue
        if line == 'end':
            nesting_level -= 1
            if nesting_level < 0:
                RaiseError(filename, count, originLine.strip('\n'),
                        'SyntaxError: found an unexcepted "end".')
                break
            if nesting_level == 0:
                if nesting_type == 'switch':
                    HandleSwitch(values, filename, linenumberOfNesting)
                    values = []
            handled = True
        if nesting_level > 0:
            values.append(line)
            continue
        if line.startswith('print'):
            print('print: ' + line[len('print'):].strip())
            continue
        if line.startswith('speak'):
            value = line[len('speak'):].strip()
            value = RemoveQuotes(value)
            Speak(value)
            continue
        if line == 'switch':
            nesting_level += 1
            if nesting_level == 1:
                nesting_type = 'switch'
                values = []
                linenumberOfNesting = count
            continue
        if line == 'gets':
            try:
                # I hope it will work on python 2 and 3
                gets_str = sys.stdin.readline().strip('\n')
            except Exception, e:
                print('Warning: in gets: %s'%e)
                gets_str = ''
            continue
        if line.startswith('system'):
            os.system(line[len('system'):].strip())
            continue
        if line.startswith('ros'):
            StartNewTab(line)
            continue
        if line.startswith('exec'):
            value = RemoveQuotes(line[len('exec'):].strip())
            try:
                RunFile(value)
            except Exception, e:
                print('Error: In file ' + value + '. Exec in '
                        + filename + ': ' + str(count) + ': %s'%e)
            continue 
        if line.startswith('case'):
            RaiseError(filename, count, originLine.strip('\n'),
                    'SyntaxError: found an unexcepted "case".')
            break
        if line.startswith('break'):
            RaiseError(filename, count, originLine.strip('\n'),
                    'SyntaxError: found an unexcepted "break".')
            break
        if not handled:
            RaiseError(filename, count, originLine.strip('\n'),
                    'SyntaxError: unable to understand this line.')
            break

# run scripts between switch & end
def HandleSwitch(lines, filename, linenumber):
    count = linenumber
    cases = []
    nesting_level = 0
    # traversal all cases
    for originLine in lines:
        count += 1
        line = originLine.strip()
        if line.startswith('case'):
            nesting_level += 1
            if nesting_level == 1:
                cases.append(RemoveQuotes(line[len('case'):].strip()))
                continue
        if line.startswith('break'):
            nesting_level -= 1
            if nesting_level < 0:
                RaiseError(filename, count, originLine.strip('\n'),
                        'SyntaxError: found an unexcepted "break".')
                break

    sel = SelectACase(cases)  # select a case by keyboard (or voice?)
    HandleCase(lines, cases[sel-1], filename, linenumber)

def HandleCase(lines, theCase, filename, linenumber):
    count = linenumber
    values = []
    nesting_level = 0
    linenumberOfNesting = 0
    found = False
    for originLine in lines:
        count += 1
        line = originLine.strip()
        if line.startswith('break'):
            nesting_level -= 1
            if nesting_level < 0:
                RaiseError(filename, count, originLine.strip('\n'),
                        'SyntaxError: found an unexcepted "break".')
                break
            if nesting_level == 0 and found:
                print('run lines')
                RunLines(values, filename, linenumberOfNesting)
                break
            continue
        if found:
            values.append(originLine)
        if line.startswith('case'):
            nesting_level += 1
            case = RemoveQuotes(line[len('case'):].strip())
            if (nesting_level) == 1 and (case == theCase):
                found = True
                linenumberOfNesting = count
            continue

def SelectACase(cases):
    menu = 'Please select:'
    count = 0
    for case in cases:
        count += 1
        menu += ' %d.(%s)' % (count, case)
    sel = 0
    while (not rospy.is_shutdown()) and (sel == 0):
        print(menu)
        try:
            sel = int(input())
        except:
            print('Error: invalid input.')
            continue
        if sel <= 0 or sel > count:
            print('Error: input out of range.')
            sel = 0
    return sel

def RaiseError(filename, linenumber, line, desc):
    print(filename + ':' + str(linenumber) + ': ' + line)
    print('Error: ' + desc)
    raise Exception(desc)

def RemoveQuotes(value):
    if value.startswith('"') and value.startswith('"'):
        value = value[1:-1]
    return value

def main(argv):
    print(cwd)
    config.read(cwd + '/settings.ini')

    StartNewTab('roscore')
    rospy.init_node('tinker_start', anonymous=True)
    StartNewTab('rosrun d_say say_node.py')

    SwitchBack()
    global rate
    rate = rospy.Rate(10)
    try:
        RunFile('main')
    except Exception, e:
        print('Error: In file main.tinkerstart: %s'%e)
    print('Press C-c to exit...')
    Wait()
    print('Bye!')

if __name__ == '__main__':
    main(sys.argv)

