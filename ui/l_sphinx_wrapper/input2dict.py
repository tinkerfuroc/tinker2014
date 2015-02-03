#!/usr/bin/env python
# -*- coding: utf-8 -*-
# file: input2dict.py
# created by bss at 2014-04-30
# Last modified: 2015-02-02, 23:29:17
# 把输入转化为字典
#

import sys
import rospkg
import getopt

def Usage():
    print('input2dict.py usage:')
    print('把输入的句子、单词转化为发音字典')
    print('-h,--help: print help message.')
    print('-t,--task: input the name of the task.')
    print('-i,-o: specify input/out file name.')
    print('')
    print('example:')
    print('python input2dict.py -t whoiswho')

def main(argv):
    if len(argv) <= 1:
        Usage()
        sys.exit(2)
    try:
        opts, args = getopt.getopt(argv[1:], 'ht:i:o:', ['help', 'task='])
    except getopt.GetoptError, err:
        print(str(err))
        Usage()
        sys.exit(2)
    except:
        Usage()
        sys.exit(1)

    inputfile = ''
    outputfile = ''
    for o, a in opts:
        if o in ('-h', '--help'):
            Usage()
            sys.exit(0)
        elif o in ('-t', '--task'):
            processTask(a)
            sys.exit(0)
        elif o in ('-i'):
            inputfile = a
        elif o in ('-o'):
            outputfile = a
    if inputfile != '' and outputfile != '':
        processFile(inputfile, outputfile)

def processTask(task):
    taskdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/launches/tasks/' + task
    inputfile = taskdir + '/sent.txt'
    outputfile = taskdir + '/words.dic'
    processFile(inputfile, outputfile)

def processFile(inputfile, outputfile):
    scriptdir = rospkg.RosPack().get_path('l_sphinx_wrapper') \
            + '/../../../ui/l_sphinx_wrapper'
    fp = open(scriptdir + '/cmudict_SPHINX_40', 'r')
    cmudict = {}
    for line in fp.readlines():
        col = line.strip().upper().split('\t', 1)
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

