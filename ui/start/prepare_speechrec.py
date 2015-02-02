#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : prepare_speechrec.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-02-02
#  Last modified: 2015-02-02, 23:40:42
# Description   : generate files for l_sphinx_wrapper
#

import sys
import os
import rospy
import rospkg

class prepareSpeechrec:
    def __init__(self, menus):
        self.menus = menus
        self.cwd = os.path.split(os.path.realpath(__file__))[0]

    def Prepare(self, hashcode):
        self.workdir = self.cwd + '/../../share/l_sphinx_wrapper/start/' \
                + str(hashcode)
        try:
            self.prepare()
        except Exception, e:
            print('In prepareSpeechrec: %s'%e)

    def prepare(self):
        self.writeFile()
        self.generateFSG()
        self.updateFSGModel()

    def writeFile(self):
        if not os.path.exists(self.workdir):
            os.system('mkdir ' + self.workdir)

        try:
            f = open(self.workdir + '/sent.txt', 'w')
            for line in self.menus:
                f.write(line + '\n')
            f.close()
        except Exception, e:
            raise Exception('Error: In writing sent.txt: %s', e)

        try:
            f = open(self.workdir + '/gram.jsgf', 'w')
            f.write('#JSGF v1.0;\n')
            f.write('grammar starter;\n')
            f.write('public <cmd> =')
            f.write(' ' + self.menus[0].upper())
            for line in self.menus[1:]:
                f.write(' | ' + line.upper())
            f.write(' ;')
            f.close()
        except Exception, e:
            raise Exception('Error: In writing gram.jsgf: %s', e)

    def generateFSG(self):
        self.fsgfile = self.workdir + '/finite_state.fsg'
        if not os.path.exists(self.fsgfile):
            os.system('sphinx_jsgf2fsg -jsgf ' + self.workdir
                    + '/gram.jsgf -fsg ' + self.fsgfile)
        self.dictfile = self.workdir + '/words.dic'
        if not os.path.exists(self.dictfile):
            todict = self.cwd + '/../l_sphinx_wrapper/input2dict.py'
            os.system('python ' + todict + ' -i ' + self.workdir
                    + '/sent.txt -o ' + self.dictfile)

    def updateFSGModel(self):
        os.system('rosrun l_sphinx_wrapper change_task.py'
                + ' --fsg ' + self.fsgfile
                + ' --dict ' + self.dictfile)
