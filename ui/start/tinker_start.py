#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : tinker_start.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-02-01
#  Last modified: 2015-02-04, 11:23:00
# Description   : Startup script for tinker, main body.
#

import sys
import os
import rospy
import ConfigParser
import getopt
import signal

from std_msgs.msg import String
from d_say.srv import *

from prepare_speechrec import prepareSpeechrec

class Starter:
    def __init__(self):
        self.config = ConfigParser.SafeConfigParser(
                {'xdotool_sleep_time': '4',
                'allow_speech_recognize': '1'}
        )
        self.cwd = os.path.split(os.path.realpath(__file__))[0]
        self.gets_str = ''
        self.isRemote = False   # 在远程执行,无法启动新标签页
        self.tabNum = 0
        self.isQuiet = False    # 不启动前几个程序,仅供调试本脚本用

    def d_say_IsPlaying(self):
        rospy.wait_for_service('/say/IsPlaying')
        try:
            func_IsPlaying = rospy.ServiceProxy('/say/IsPlaying', IsPlaying)
            resp = func_IsPlaying()
            return resp.playing
        except rospy.ServiceException, e:
            print('fail: %s'%e)
            return False

    def Speak(self, text):
        print('speak: "' + text + '"')
        self.rate.sleep()
        if self.d_say_IsPlaying():
            print('waiting for d_say...')
            while (not rospy.is_shutdown()) and self.d_say_IsPlaying():
                self.rate.sleep()
        self.say_pub.publish(str(text))
        self.rate.sleep()

    def Wait(self):     # wait for C-c
        while not rospy.is_shutdown():
            self.rate.sleep()

    def StartNewTab(self, cmd):
        self.tabNum += 1
        if self.isRemote:
            logfile = self.cwd + '/../../' \
                    + 'share/start/log' + str(self.tabNum) + '.log'
            command = cmd + ' > ' + logfile
            print('Run sh: %s: log to %s' % (cmd, logfile))
            os.system(command + ' & ')
            sys.stdout.flush()
        else:
            t = self.config.get('scripts', 'xdotool_sleep_time')
            os.system('xdotool key ctrl+shift+t ; sleep ' + str(t) + ' ; '
                    + 'xdotool type "' + cmd + '" ; xdotool key "Return"')

    def SwitchBack(self):   # switch back to this tab
        if not self.isRemote:
            os.system('xdotool key alt+1 ; sleep 1')

    def RunFile(self, filename):      # run tinkerstart script
        print('run ' + filename + '.tinkerstart')
        filepath = self.cwd + '/scripts/' + filename + '.tinkerstart'
        try:
            f = open(filepath, 'r')
            lines = f.readlines()
        except Exception, e:
            print('Error: cannot open %s.tinkerstart: %s'
                    % (filename, e))
            return
        self.RunLines(lines, filename, 0)

    # run script in memory
    def RunLines(self, lines, filename, linenumber):
        nesting_level = 0
        nesting_type = ''
        values = []
        count = 0
        linenumberOfNesting = 0
        for originLine in lines:
            originLine = originLine.split('#')[0]
            count += 1
            handled = False
            line = originLine.strip()
            line = line.replace('$(str)', self.gets_str)
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
                        self.HandleSwitch(values, filename,
                                linenumberOfNesting)
                    if nesting_type == 'loop':
                        self.HandleLoop(values, repeatTimes, filename,
                                linenumberOfNesting)
                    values = []
                handled = True
            if line == 'switch':
                nesting_level += 1
                if nesting_level == 1:
                    nesting_type = 'switch'
                    values = []
                    linenumberOfNesting = count
                    continue
                handled = True
            if line.startswith('loop'):
                nesting_level += 1
                if nesting_level == 1:
                    nesting_type = 'loop'
                    values = []
                    linenumberOfNesting = count
                    repeatTimes = int(line[len('loop'):])
                    continue
                handled = True
            if nesting_level > 0:
                if line.startswith('case'): # appear between switch/end
                    nesting_level += 1
                values.append(line)
                continue
            if line.startswith('print'):
                print('print: ' + line[len('print'):].strip())
                continue
            if line.startswith('speak'):
                value = line[len('speak'):].strip()
                value = RemoveQuotes(value)
                self.Speak(value)
                continue
            if line == 'gets':
                try:
                    # I hope it will work on both python 2 and 3
                    self.gets_str = sys.stdin.readline().strip('\n')
                except Exception, e:
                    print('Warning: in gets: %s'%e)
                    self.gets_str = ''
                continue
            if line.startswith('system'):
                os.system(line[len('system'):].strip())
                continue
            if line.startswith('ros'):
                self.StartNewTab(line)
                continue
            if line.startswith('exec'):
                value = RemoveQuotes(line[len('exec'):].strip())
                try:
                    self.RunFile(value)
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
    def HandleSwitch(self, lines, filename, linenumber):
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
                    cases.append(RemoveQuotes(
                            line[len('case'):].strip()))
                    continue
            if line == 'end':
                nesting_level -= 1
                if nesting_level < 0:
                    RaiseError(filename, count, originLine.strip('\n'),
                            'SyntaxError: found an unexcepted "end".')
                    break

        self.SwitchBack()
        # select a case by keyboard (or voice?)
        sel = self.SelectACase(cases)
        if sel <= 0:
            RaiseError(filename, count, originLine.strip('\n'),
                    'Error: invalid selection.')
            return
        self.HandleCase(lines, cases[sel-1], filename, linenumber+1)

    def HandleCase(self, lines, theCase, filename, linenumber):
        count = linenumber
        values = []
        nesting_level = 0
        linenumberOfNesting = 0
        found = False
        for originLine in lines:
            count += 1
            line = originLine.strip()
            if line == 'end':
                nesting_level -= 1
                if nesting_level < 0:
                    RaiseError(filename, count, originLine.strip('\n'),
                            'SyntaxError: found an unexcepted "end".')
                    break
                if nesting_level == 0 and found:
                    self.RunLines(values, filename, linenumberOfNesting)
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

    def SelectACase(self, cases):
        menu = 'Please select:'
        count = 0
        menus = []
        for case in cases:
            count += 1
            menu += ' %d.(%s)' % (count, case)
            menus.append(case.lower().replace(',', '').replace('.', '')
                    .strip())

        sel = 0
        self.sel_key = None
        self.sel_voice = None

        if self.allow_speech_recognize:
            self.PrepareForSpeechrec(menus, hash(menu))

        # 暂时这样写:外部调用fromKeyboard,
        # 如果callback得到结果也通过fromKeyboard返回
        select_voice = self.SelectACase_Funcs(
                menus, self.allow_speech_recognize)
        sel = select_voice.fromKeyboard(menu, count)

        return sel

    # run scripts between loop & end
    def HandleLoop(self, lines, repeatTimes, filename, linenumber):
        if repeatTimes < 0:
            try:
                while True:
                    self.RunLines(lines, filename, linenumber+1)
            except Exception, e:
                if str(e) != 'EOL':     # EOL为准备使用的退出循环方式
                    raise e
        else:
            for i in range(0, repeatTimes):
                self.RunLines(lines, filename, linenumber+1)


    def PrepareForSpeechrec(self, menus, hashcode):
        ps = prepareSpeechrec(menus)
        ps.Prepare(hashcode)


    class SelectACase_Funcs:
        def __init__(self, menus, allow_speech_recognize):
            self.sel_voice = None
            self.menus = menus
            self.sel = 0
            self.allow_speech_recognize = allow_speech_recognize
            if self.allow_speech_recognize:
                self.sub = rospy.Subscriber('/recognizer/output', String,
                        self.callback)

        # input from voice
        def callback(self, data):
            for i in range(0, len(self.menus)):
                if data.data.strip() == self.menus[i]:    # done
                    self.sel_voice = i + 1
                    print(self.sel_voice)

        # input from keyboard
        def fromKeyboard(self, menu, count):
            sel = 0
            print(menu)
            signal.signal(signal.SIGALRM, self.timeoutHandler)
            while ((not rospy.is_shutdown()) and (sel == 0)
                    and (self.sel_voice == None)):
                signal.alarm(1)
                try:
                    sel = int(input())
                except Exception, e:
                    if str(e) != 'timeout':
                        print('Error: invalid input: %s'%e)
                        print(menu)
                    continue
                if sel <= 0 or sel > count:
                    print('Error: input out of range.')
                    sel = 0
            signal.alarm(0)
            if self.allow_speech_recognize:
                self.sub.unregister()
            if self.sel_voice is not None:
                return self.sel_voice
            else:
                return sel

        def timeoutHandler(self, signum, frame):
            raise Exception('timeout')


    def GetOpts(self, argv):
        try:
            opts, argv = getopt.getopt(argv[1:], 'h',
                    ['help', 'remote', 'quiet'])
        except getopt.GetoptError, e:
            print('Error: invalid argv: %s'%e)
            sys.exit(2)
        except Exception, e:
            print('Error: %s'%e)
            sys.exit(1)
        
        for o, a in opts:
            if o in ('-h', '--help'):
                print('https://github.com/tinkerfuroc/tinker2014')
                sys.exit(0)
            if o in ('--remote'):
                self.isRemote = True
            if o in ('--quiet'):
                self.isQuiet = True


    def Start(self, argv):
        print(self.cwd)
        # config
        self.config.read(self.cwd + '/settings.ini')
        # opts
        self.GetOpts(argv)

        t = self.config.get('scripts', 'allow_speech_recognize')
        self.allow_speech_recognize = (str(t) != '0')

        node_is_running = not os.system(
                'rosnode list | egrep ^/tinker_start$ -q')
        if node_is_running:
            print('Error: already running, will exit.')
            sys.exit(1)

        if not self.isQuiet:
            self.StartNewTab('roscore')
        rospy.init_node('tinker_start', anonymous=False)
        if not self.isQuiet:
            self.StartNewTab('export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-0.10 && rosrun d_say say_node.py')
        self.say_pub = rospy.Publisher(
                '/say/sentence', String, queue_size=1)
        if not self.isQuiet:
            self.StartNewTab('rosrun l_sphinx_wrapper recognizer.py')

        self.SwitchBack()
        self.rate = rospy.Rate(2)   # 不能太快,不然语音节点反应不过来
        try:
            self.RunFile('main')
        except Exception, e:
            print('Error: In file main.tinkerstart: %s'%e)
        print('Press C-c to exit...')
        self.Wait()
        
def RaiseError(filename, linenumber, line, desc):
    print(filename + '.tinkerstart:' + str(linenumber) + ': ' + line)
    print('Error: ' + desc)
    raise Exception(desc)

def RemoveQuotes(value):
    if value.startswith('"') and value.startswith('"'):
        value = value[1:-1]
    return value

def main(argv):
    starter = Starter()
    starter.Start(argv)
    print('Bye!')

if __name__ == '__main__':
    main(sys.argv)

