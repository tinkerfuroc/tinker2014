#!/usr/bin/env python
# -*- coding: utf-8 -*-
# file: input2dict.py
# created by bss at 2014-04-30
# Last modified: 2015-04-25, 20:11:30
# 把输入转化为字典
#

import sys
import rospkg
import getopt

lang_list = ('en', 'zh')


def Usage():
    print('input2dict.py usage:')
    print('把输入的句子、单词转化为发音字典')
    print('-h,--help: print help message.')
    print('-t,--task: input the name of the task.')
    print('-i,-o: specify input/out file name.')
    print('-l zh: use chinese')
    print('')
    print('example:')
    print('python input2dict.py -t whoiswho')


def main(argv):
    if len(argv) <= 1:
        Usage()
        sys.exit(2)
    try:
        opts, args = getopt.getopt(argv[1:], 'ht:i:o:l:', ['help', 'task='])
    except getopt.GetoptError, err:
        print(str(err))
        Usage()
        sys.exit(2)
    except:
        Usage()
        sys.exit(1)

    inputfile = ''
    outputfile = ''
    lang = 'en'
    taskname = ''
    for o, a in opts:
        if o in ('-h', '--help'):
            Usage()
            sys.exit(0)
        elif o in ('-l'):
            lang = a
            if lang not in lang_list:
                print('unknown language %s.' % lang)
                Usage()
                sys.exit(-2)
        elif o in ('-t', '--task'):
            if (taskname != '' or inputfile != '' or outputfile != ''):
                Usage()
                sys.exit(2)
            taskname = a
        elif o in ('-i'):
            inputfile = a
        elif o in ('-o'):
            outputfile = a
    if taskname != '':
        processTask(taskname, lang)
    elif (inputfile != '' and outputfile != ''):
        processFile(inputfile, outputfile, lang)
    else:
        Usage()
        sys.exit(0)


def processTask(task, lang):
    taskdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/launches/tasks/' + task
    inputfile = taskdir + '/sent.txt'
    outputfile = taskdir + '/words.dic'
    processFile(inputfile, outputfile, lang)


def processFile(inputfile, outputfile, lang):
    scriptdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/../../../ui/l_sphinx_wrapper'
    dictname = {'en' : 'cmudict_SPHINX_40', 'zh' : 'mandarin_notone.dic'}
    dictname = dictname[lang]

    fp = open('%s/%s' % (scriptdir, dictname), 'r')
    cmudict = {}
    for line in fp.readlines():
        col = line.strip().split('\t', 1)
        cmudict[col[0]] = col[1]
    fp.close()

    fp = open(inputfile, 'r')
    words = []
    for line in fp.readlines():
        col = line.strip().upper().split()
        for word in col:
            words.append(word)
    fp.close()

    words = list(set(words))

    fp = open(outputfile, 'w')
    for word in words:
        isSucceed = False
        try:
            line = word + '\t' + cmudict[word] + '\n'
            fp.write(line)
            isSucceed = True
            for i in range(2, 5):
                key = word + '(' + str(i) + ')'
                line = key + '\t' + cmudict[key] + '\n'
                fp.write(line)
        except:
            pass
        if not isSucceed:
            print("I can't find " + word)
    fp.close()


if __name__ == '__main__':
    main(sys.argv)

