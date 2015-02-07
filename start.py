#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : start.py
# Module        : tinker
# Author        : bss
# Creation date : 2015-01-29
#  Last modified: 2015-02-02, 09:47:24
# Description   : Startup script for tinker.
#

import sys
import os

def Usage():
    print('start.py usage:')
    print('start the system.')

def main(argv):
    cwd = os.path.split(os.path.realpath(__file__))[0];
    print(cwd)
    command = "python " + cwd + "/ui/start/tinker_start.py"
    for i in range(1, len(argv)):   # skip argv[0]
        command += ' ' + argv[i]
    print(command)
    os.system(command)

if __name__ == '__main__':
    main(sys.argv)

