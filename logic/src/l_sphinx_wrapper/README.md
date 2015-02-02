#l_sphinx_wrapper

Author: bssthu

ROS-Version: indigo

Description: 对pocketsphinx的封装

###使用方法

```bash
roslaunch l_sphinx_wrapper TASK_NAME
```

###package信息

下面列出的是pocketsphinx的信息：

######nodes
- recognizer.py

######发布的topics
- /recognizer/output

######提供的services
- /recognizer/start ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/stop ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))

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
