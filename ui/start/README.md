tinker启动脚本
=======

使用方法:

```bash
cd $TINKER_WORKSPACE
./start.py
```

如果在远程执行，无法打开新标签页，
请运行

```bash
cd $TINKER_WORKSPACE
./start.py --remote
```

然后根据提示操作。

脚本会解析ui/start/main.tinkerstart文件，
进行一些启动操作。

###需要安装的软件

```bash
sudo apt-get install xdotool    # 切换标签页
sudo apt-get install espeak     # 语音合成
```

##tinkerstart文件格式

下面是tinkerstart文件的一个例子:

```
# -*- coding: utf-8 -*-
# File          : main.tinkerstart
# Module        : tinker
# Author        : bss
# Description   :
#

print "main.tinkerstart for tinker"

speak "hello, please select a task"
switch
    case "answer questions"
        speak "start task answer questions"
        exec "answer_question"
        break
    case "record p c d"
        speak "start task record pcd"
        speak "please input a name"
        gets
        roslaunch d_pcl record_pcds_from_primesense.launch dir:=$(str)
        break
    case "cancel"
        break
end

print "please press enter..."
gets
speak "goodbye"
system echo "The End"
```

####注释
注释以井号(#)开头。
目前不允许行中注释。

####缩进
缩进的意义是美观，
没有强制要求。

####print和speak
使用方法: print/speak+空格+字符串(需要双引号)

print顾名思义。

speak借助d_say播放语音，
同时也会执行print。

```
print "hello"
speak "test"
```

####switch/case语句
用于供用户选择功能。

switch必须与end成对出现，
case必须与break成对出现。
使用方法见示例。

```
switch
    case "situation A"
        print "A"
        break
    case "situation B"
        print "B"
        break
    case "cancel"
    # cancel是可选的
        break
end
```

这段代码执行后，
屏幕会输出类似这样的内容：

```
Please select: 1.(situation A) 2.(situation B) 3.(cancel)
```

如果输入1并按下回车，
就会执行

```
print "A"
```

####gets
用于输入字符串，
供后续使用。
也可用于暂停程序。

代码中的"$(str)"，不含引号，
会被替换成输入的内容。
默认为空。

```
gets
# test
print "Your input is $(str)."
# Your input is test.
```

####system和rosxxx
执行命令。
会打开新标签页。

system+空格+命令(无须双引号)。

rosxxx即为命令。

```
system echo "hello"
rosrun d_say say_node.py
```

####exec
执行另一个脚本文件。
格式为exec+空格+文件名(需要双引号，不含扩展名)。

```
exec "subsystem1"
# 运行 ui/start/scripts/subsystem1.tinkerstart
```

