#l_sphinx_wrapper

Author: bssthu

ROS-Version: indigo

Description: 对 pocketsphinx 的封装

### 使用方法

二选一：

```bash
roslaunch l_sphinx_wrapper TASK_NAME
rosrun l_sphinx_wrapper recognizer.py   # 用这种方法运行时，默认不进行识别，需要一些额外配置
```

### package 信息

下面列出的是 pocketsphinx 的信息：

###### nodes
- recognizer.py         # 语音识别节点
- change_task.py        # 改变recognizer.py使用的语法模型
- fake_recognizer.py    # 将输入字符串发布到 /recgonizer/output 中

###### 发布的 topics
- /recognizer/output

###### 提供的 services
- /recognizer/start ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/stop ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- /recognizer/change_fsg ([l_sphinx_wrapper/ChangeFSG])
- /recognizer/stop ([l_sphinx_wrapper/ChangeTask])

###### recognizer.py 接受的 ros param
- fsg 使用的fsg语法模型
- dict 词库

选择语言时：
- hmm hmm模型文件所在路径
- lm_init lm模型文件路径
- dict_init 词库（完整版）


### 配置方法

##### 安装（无中文）


```bash
sudo apt-get install sphinxbase-utils    # sphinx_jsgf2fsg命令
```

接下来是pocketsphinx的依赖项：

```bash
sudo apt-get install python-gst0.10 gstreamer0.10-pocketsphinx
sudo apt-get install gstreamer0.10-gconf    # 应该已经自动安装了
```

##### 安装（有中文）

使用 apt-get install 安装的是 pocketsphinx 的 0.8 版本，
最多只支持 64 种不同的音素，
因此无法处理汉语。
如果之前用 apt-get 装了
sphinxbase, pocketsphinx, libsphinxbase-dev, libpocketsphinx-dev
等软件包，
请用 apt-get remove 卸载。

修改方法如下：

- 获取源码

```bash
apt-get source sphinxbase
apt-get source gstreamer0.10-pocketsphinx
tar -xzvf sphinxbase_0.8.orig.tar.gz
tar -xzvf pocketsphinx_0.8.0+real.orig.tar.gz
```

如果无法下载，
可以到
https://drive.google.com/open?id=0B-kQc2-wuHntfnVYUFRjcEZuSE5SX0pVeGY3UkxRVW83LVNDY2dxVHNLYjhBMFB5OS04clk&authuser=0
下载 Ubuntu 14.04 (Trusty) 版本可用的源码备份。

- 修改 pocketsphinx-0.8.0+real/src/libpocketsphinx/fsg_lextree.h, 
将在第94行处 define 的 FSG_PNODE_CTXT_BVSZ 由 2 改为 4. 

如果还是太小，
可以改成更大的数。

- 编译安装

```bash
cd sphinxbase-0.8/
./autogen.sh
./configure
# 检查输出，安装缺少的软件包，重复上一步，直到一切正常
make
sudo make install

# pocketsphinx 的编译安装方法同上
cd ../pocketsphinx-0.8.0+real/
./autogen.sh
./configure
# 检查输出，安装缺少的软件包，重复上一步，直到一切正常
make
sudo make install
```

- 安装其余需要的软件

```bash
# 汉语模型
sudo apt-get install pocketsphinx-hmm-zh-tdt pocketsphinx-lm-zh-hans-gigatdt
# python 库
sudo apt-get install python-pocketsphinx
```

- 最后，由于源码编译安装的默认路径在 /usr/local 而非 /usr，
所以需要 export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-0.10 。
在 .launch 文件中设置即可。

##### 初始化任务

请参考 ui/l_sphinx_wrapper/README.md

