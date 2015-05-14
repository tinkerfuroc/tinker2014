#!/usr/bin/env python
# -*- coding: utf-8 -*-
# file: init.py
# created by bss at 2014-05-09
# Last modified: 2015-04-25, 20:08:42
# 初始化一组语音识别的任务
#

import sys
import os
import getopt
import rospkg

lang_list = ('en', 'zh')


def Usage():
    print('init.py usage:')
    print('初始化一组语音识别任务，处理${task}/sent.txt,gram.jsgf')
    print('-h,--help: print help message.')
    print('--init: init a dir for your task')
    print('-t,--task: input the name of the task.')
    print('-l zh: use chinese')
    print('')
    print('example:')
    print('python init.py --init whoiswho')
    print('python init.py -t whoiswho')


def main(argv):
    if len(argv) <= 1:
        Usage()
        sys.exit(2)
    try:
        opts, args = getopt.getopt(argv[1:], 'ht:l:',
                ['help', 'task=', 'init='])
    except getopt.GetoptError, err:
        print(str(err))
        Usage()
        sys.exit(2)
    except:
        Usage()
        sys.exit(1)

    lang = 'en'
    initdir = ''
    taskdir = ''
    for o, a in opts:
        if o in ('-h', '--help'):
            Usage()
            sys.exit(0)
        elif o in ('-l'):
            lang = a
            if lang not in lang_list:
                print('unknown language %s.' % lang)
                Usage()
                sys.exit(0)
        elif o in ('--init'):
            if (initdir != '' or taskdir != ''):
                Usage()
                sys.exit(2)
            initdir = a
        elif o in ('-t', '--task'):
            if (initdir != '' or taskdir != ''):
                Usage()
                sys.exit(2)
            taskdir = a

    if initdir != '':
        initDir(initdir, lang)
    elif taskdir != '':
        processTask(taskdir, lang)
    else:
        Usage()
        sys.exit(0)


def initDir(task, lang):
    outdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/launches'
    taskdir = outdir + '/tasks/' + task
    os.system('mkdir ' + taskdir)
    os.system('touch ' + taskdir + '/sent.txt')
    os.system('touch ' + taskdir + '/gram.jsgf')

    fp = open(taskdir + '/gram.jsgf', 'w')
    fp.write('#JSGF v1.0;\n')
    fp.write('grammar ' + task + ';\n')
    fp.write('public <cmd> =')
    fp.close()

    fp = open(outdir + '/' + task + '.launch', 'w')
    fp.write('<launch>\n')
    fp.write('\n')
    fp.write('  <env name="GST_PLUGIN_PATH" '
            + 'value="/usr/local/lib/gstreamer-0.10" />\n')
    fp.write('\n')

    fp.write('  <node name="recognizer" pkg="l_sphinx_wrapper" ' \
            + 'type="recognizer.py" output="screen">\n')
    if lang == 'zh':
        fp.write('    <param name="hmm" value="/usr/share/' \
                + 'pocketsphinx/model/hmm/zh/tdt_sc_8k" />\n')
        fp.write('    <param name="lm_init" value="/usr/share/' \
                + 'pocketsphinx/model/lm/zh_CN/gigatdt.5000.DMP" />\n')
        fp.write('    <param name="dict_init" value="/usr/share/' \
                + 'pocketsphinx/model/lm/zh_CN/mandarin_notone.dic" />\n')
        fp.write('\n')
    fp.write('    <param name="fsg" ' \
            + 'value="$(find l_sphinx_wrapper)/launches/tasks/' \
            + task + '/finite_state.fsg"/>\n')
    fp.write('    <param name="dict" ' \
            + 'value="$(find l_sphinx_wrapper)/launches/tasks/'
            + task + '/words.dic"/>\n')
    fp.write('  </node>\n')
    fp.write('\n')
    fp.write('</launch>\n')
    fp.close()


def processTask(task, lang):
    scriptdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/../../../ui/l_sphinx_wrapper'
    os.system('python %s/input2dict.py -l %s -t %s' %
            (scriptdir, lang, task))
    os.system('python %s/jsgf2fsg.py -t %s' % (scriptdir, task))
    

if __name__ == '__main__':
    main(sys.argv)
