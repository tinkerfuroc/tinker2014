#l_sphinx_wrapper

Author: bssthu

ROS-Version: indigo

Description: 对pocketsphinx的封装

###使用方法

二选一：

```bash
roslaunch l_sphinx_wrapper TASK_NAME
rosrun l_sphinx_wrapper recognizer.py   # 用这种方法运行时，默认不进行识别，需要一些额外配置
```

###package信息

下面列出的是pocketsphinx的信息：

######nodes
- recognizer.py         # 语音识别节点
- change_task.py        # 改变recognizer.py使用的语法模型
- fake_recognizer.py    # 将输入字符串发布到/recgonizer/output中

######发布的topics
- /recognizer/output

######提供的services
- /recognizer/start ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/stop ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/change_fsg ([l_sphinx_wrapper/ChangeFSG])
- /recognizer/stop ([l_sphinx_wrapper/ChangeTask])

###配置方法

#####安装


```bash
sudo apt-get install sphinxbase-utils    # sphinx_jsgf2fsg命令
```

接下来是pocketsphinx的依赖项：

```bash
sudo apt-get install python-gst0.10 gstreamer0.10-pocketsphinx
sudo apt-get install gstreamer0.10-gconf    # 应该已经自动安装了
```

#####初始化任务

请参考ui/l_sphinx_wrapper/README.md
