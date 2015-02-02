#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : recognizer.py
# Module        : l_sphinx_wrapper@tinker
# Author        : bss
# Creation date : 2015-02-02
#  Last modified: 2015-02-02, 20:43:48
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
from l_sphinx_wrapper.srv import *

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        """ Initialize the speech pipeline components. """
        rospy.init_node('recognizer')
        self.pub = rospy.Publisher('/recognizer/output',String)
        rospy.on_shutdown(self.shutdown)
        
        # package path
        self.package_path = rospkg.RosPack().get_path('l_sphinx_wrapper')
        self.task_path = self.package_path \
                + '/../../../share/l_sphinx_wrapper'

        # services to start/stop recognition
        rospy.Service("/recognizer/start", Empty, self.start)
        rospy.Service("/recognizer/stop", Empty, self.stop)
        # services to change grammar model
        rospy.Service("/recognizer/change_fsg", ChangeFSG, self.changeFSG)
        rospy.Service("/recognizer/change_task",
                ChangeTask, self.changeTask)

        # configure pipeline
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert '
                + '! audioresample ! vader name=vad auto-threshold=true '
                + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('partial_result', self.asr_partial_result)
        asr.connect('result', self.asr_result)
        asr.set_property('configured', True)
        asr.set_property('dsratio', 1)

        # parameters for fsg and dic
        try:
            fsg_ = rospy.get_param('~fsg')
            asr.set_property('fsg', fsg_)
        except:
            rospy.logerr('Please specify a fsg grammar file')
        try:
            dict_ = rospy.get_param('~dict')
            asr.set_property('dict', dict_)
        except:
            rospy.logerr('Please specify a dictionary')

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::application', self.application_message)
        self.start(None)
        gtk.main()
        
    def shutdown(self):
        """ Shutdown the GTK thread. """
        gtk.main_quit()

    def start(self, req):
        self.pipeline.set_state(gst.STATE_PLAYING)
        return EmptyResponse()

    def stop(self, req):
        self.pipeline.set_state(gst.STATE_PAUSED)
        #vader = self.pipeline.get_by_name('vad')
        #vader.set_property('silent', True)
        return EmptyResponse()

    def changeFSG(self, req):
        # parameters for fsg and dic
        asr = self.pipeline.get_by_name('asr')
        fsg_ = req.fsg
        dict_ = req.dict

        if not os.path.isfile(fsg_):
            rospy.logerr('The fsg file %s does not exist.' % fsg_)
            return ChangeTaskResponse(False)
        if not os.path.isfile(dict_):
            rospy.logerr('The dict file %s does not exist.' % dict_)
            return ChangeTaskResponse(False)
            
        try:
            asr.set_property('fsg', fsg_)
        except:
            rospy.logerr('The fsg grammar file %s is invalid.' % fsg_)
            return ChangeFSGResponse(False)
        try:
            asr.set_property('dict', dict_)
        except:
            rospy.logerr('The dictionary %s is invalid.' % dict_)
            return ChangeFSGResponse(False)
        return ChangeFSGResponse(True)

    def changeTask(self, req):
        # parameters for task
        asr = self.pipeline.get_by_name('asr')
        filedir = self.task_path + '/' + req.name
        fsg_ = filedir + '/finite_state.fsg'
        dict_ = filedir + '/words.dic'

        if not os.path.isfile(fsg_):
            rospy.logerr('The fsg file %s does not exist.' % fsg_)
            return ChangeTaskResponse(False)
        if not os.path.isfile(dict_):
            rospy.logerr('The dict file %s does not exist.' % dict_)
            return ChangeTaskResponse(False)

        try:
            asr.set_property('fsg', fsg_)
        except Exception, e:
            rospy.logerr('Error: %s'%e)
            rospy.logerr('The fsg grammar file %s is invalid.' % fsg_)
            return ChangeTaskResponse(False)
        try:
            asr.set_property('dict', dict_)
        except Exception, e:
            rospy.logerr('Error: %s'%e)
            rospy.logerr('The dictionary %s is invalid.' % dict_)
            return ChangeTaskResponse(False)
        return ChangeTaskResponse(True)

    def asr_partial_result(self, asr, text, uttid):
        """ Forward partial result signals on the bus to the main thread. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """ Forward result signals on the bus to the main thread. """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """ Receive application messages from the bus. """
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        if msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])

    def partial_result(self, hyp, uttid):
        """ Delete any previous selection, insert text and select it. """
        print "Partial: " + hyp

    def final_result(self, hyp, uttid):
        """ Insert the final result. """
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo(msg.data)
        self.pub.publish(msg)

def main(argv):
    r = recognizer()

if __name__ == '__main__':
    main(sys.argv)
