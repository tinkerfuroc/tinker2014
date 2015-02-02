#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : recognizer.py
# Module        : l_sphinx_wrapper@tinker
# Author        : bss
# Creation date : 2015-02-02
#  Last modified: 2015-02-02, 11:49:18
# Description   : pocketsphinx wrapper. support 
#       inspired by http://wiki.ros.org/pocketsphinx
#

import sys
import os
import time
import getopt
import rospkg
import rospy

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

from std_msgs.msg import String
from std_srvs.srv import *

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        """ Initialize the speech pipeline components. """
        rospy.init_node('recognizer')
        self.pub = rospy.Publisher('~output',String)
        rospy.on_shutdown(self.shutdown)

        # services to start/stop recognition
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

        # configure pipeline
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('partial_result', self.asr_partial_result)
        asr.connect('result', self.asr_result)
        asr.set_property('configured', True)
        asr.set_property('dsratio', 1)

        # parameters for lm and dic
        try:
            lm_ = rospy.get_param('~lm')
            asr.set_property('lm', lm_)
        except:
            try:
                fsg_ = rospy.get_param('~fsg')
                asr.set_property('fsg', fsg_)
            except:
                rospy.logerr('Please specify a language model file or a fsg grammar file')
                return
        try:
            dict_ = rospy.get_param('~dict')
        except:
            rospy.logerr('Please specify a dictionary')
            return

        asr.set_property('dict', dict_)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::application', self.application_message)
        self.start(None)
        gtk.main()
        
    def shutdown(self):
        """ Shutdown the GTK thread. """
        gtk.main_quit()

def main(argv):
    r = recognizer()

if __main__ == '__main__':
    main(sys.argv)
