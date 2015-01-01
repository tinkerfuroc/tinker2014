#!/usr/bin/env python
# -*- coding: utf-8 -*-
# File          : fetch_google_voice.py
# Author        : bss
# Creation date : 2014-05-10
#  Last modified: 2015-01-01, 14:40:54
# Description   : Hacking google tts api
#

import sys
import os
import rospkg

script = rospkg.RosPack().get_path('answer_questions') \
        + '/../../../ui/Google-Text-To-Speech/GoogleTextToSpeech.py'
mp3dir = rospkg.RosPack().get_path('answer_questions') \
        + '/../../../share/d_say/sounds'
txtdir = rospkg.RosPack().get_path('answer_questions') \
        + '/../../../share/answer_questions'

def getAnswerSpeech(answer):
    #answer.replace("'", r"\'")
    os.system('python ' + script
            + ' -o ' +  '"' + mp3dir + '/' +  answer + '.mp3"'
            + ' -s ' + '"' + answer + '"')

def main(argv):
    # answer
    fp = open(txtdir + '/answers.txt', 'r')
    for line in fp.readlines():
        sentence = line.strip()
        if sentence != '':
            getAnswerSpeech(str(sentence))
    fp.close()
    fp = open(txtdir + '/questions.txt', 'r')
    for line in fp.readlines():
        sentence = line.strip()
        if sentence != '':
            getAnswerSpeech(str(sentence))
    fp.close()
    
if __name__ == '__main__':
    main(sys.argv)

