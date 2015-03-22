#d_say

Author: bssthu

ROS-Version: indigo

Description: 说话

如果在$TINKER_WORKSPACE/share/d_say/sounds中找到同名的.mp3文件，则用mplayer播放；
否则，用espeak合成语音并用aplay播放。

###使用方法

```bash
rosrun d_say say_node.py
```

###package信息

######nodes
- say_node.py

######订阅的topics
- /say/sentence  ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

###配置方法

###### 安装必要的软件包：

```bash
sudo apt-get install espeak mplayer
```
