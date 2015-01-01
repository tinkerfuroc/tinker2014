#answer_questions

Author: bssthu

ROS-Version: indigo

Description: 回答问题功能的决策节点。

###使用方法

```bash
rosrun answer_questions answer_node.py
```

###package信息

######nodes
- answer_node.py

######订阅的topics
- /recognizer/output ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

######发布的topics
- /say/sentence ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

######调用的services
- /recognizer/start ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/stop ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))

######提供的services
- /answer_questions/start ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /answer_questions/stop ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))

######命令行参数
- -h --help 帮助
- -i 忽略/answer_questions/start与/anwer_questions/stop的存在，始终工作

###配置方法
根据具体问题，编辑 resource 下的 questions.txt,answers.txt 即可。

在联网情况下，可以运行以下代码，下载 google 发音：

```bash
    cd $TINKER_WORKSPACE/ui/answer_questions/
    python fetch_google.py
```

注意： questions.txt 中请勿使用标点符号。
