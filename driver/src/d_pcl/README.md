#d_pcl
Author: bssthu

ROS-Version: indigo

Description: PCL驱动

将点云数据从 ros 转换到 pcl 的库。
使用方法请参考 ./test/show_xyzrgb.cpp

###使用方法

```bash
roslaunch d_pcl primesense_to_pcl.launch            # 启动转换程序
roslaunch d_pcl record_pcds_from_primesense.launch dir:=test1  # 从primesense录制一批pcd文件
```

```bash
rosrun d_pcl test_show_xyzrgb     # 用图形界面显示xyzrgb
rosrun d_pcl test_sub_xyz         # 命令行显示xyz
rosrun d_pcl test_sub_xyzrgb      # 命令行显示xyzrgb
rosrun d_pcl test_reading_pcd xx  # 读取pcd文件,从share/d_pcl下查找
rosrun d_pcl test_reading_pcds xx # 从share/d_pcl/xx/下读取pcd文件
rosrun d_pcl test_writing_pcds xx # 将pcd文件写入share/d_pcl/xx/下
```

######primesense_to_pcl.launch
从primesense获取信息，
发布点云到/pcl/points2和/pcl/points2_xyz

需要先启动openni2_camera:
```bash
rosrun openni2_camera openni2_camera_node    # 不是用launch
```

######record_pcds_from_primesense.launch
从primesense获取信息，转换为点云，
写入到share/d_pcl/下的xx文件夹中

需要先启动openni2_camera和primesense_to_pcl.launch:
```bash
rosrun openni2_camera openni2_camera_node
roslaunch d_pcl primesense_to_pcl.launch
```

######test_show_xyzrgb
将/pcl/points2的内容显示在图形界面中

######test_reading_pcd
读取share/d_pcl/下的xx文件，
并发送到/pcl/points2

参数：
- -h,--help: 显示帮助
- -r,--rate: 发送频率
- xx: 要发送的文件名

######test_reading_pcds
读取share/d_pcl/xx/下的文件，
文件名依次为pcd0.pcd,pcd1.pcd,...，
并发送到/pcl/points2

可以认为xx代表一组数据的名字。

参数：
- -h,--help: 显示帮助
- -r,--rate: 发送频率（好像不能太高，按需要决定是否改进）
- xx: 要使用的文件夹名

######test_writing_pcds
订阅/pcl/points2，
并将内容保存为pcd文件，存到share/d_pcl/xx/下，
文件名依次为pcd0.pcd,pcd1.pcd,...

参数：
- -h,--help: 显示帮助
- -r,--rate: 保存
- -f: 指定发送完后按什么规则重复：
  - always: 一直重复，连续发送
  - delay times: 延迟times帧后重复，如delay 10表示延迟10帧后重复
  - key: 按q键+回车退出，按其它键继续
  - 其他: 不重复
- xx: 要使用的文件夹名

###package信息

######nodes
- test_show_xyzrgb
- test_sub_xyz
- test_sub_xyzrgb
- test_reading_pcd

######订阅的topics (launch文件)
- /depth/image_raw  ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
- /depth/image
- /depth/image_metric
- /rgb/image
- /rgb/camera_info


###发布的topic
- /pcl/points2  ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
- /pcl/points2_xyz  ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

###配置方法

需要安装ros indigo，pcl 1.7，以及openni和primesense驱动。

**注意：请先安装ros，再安装pcl，否则容易装错pcl版本导致ros无法安装。**

##### PCL

- 方案A

```bash
sudo apt-get install libpcl-1.7-all-dev  # 如果版本太旧，请自行编译
```

- 方案B

下载源码，按照源码版本库中的说明安装。（流量预警）

  - https://github.com/PointCloudLibrary/pcl

##### Primesense驱动

- 方案A

下载事先编译好的二进制包并安装：

  - https://drive.google.com/open?id=0B-kQc2-wuHntfjdXYm5EV042SnVtcnZXeUtuUTNYM3c1MlJ1LWhMdGtpV3ZYQ1g2OWZvdGc&authuser=0

要装openni和sensor。

如果版本太旧或无法下载到二进制包，请采用方案B。

- 方案B

下载源码，按照源码版本库中的说明安装

  - OpenNI https://github.com/OpenNI/OpenNI

  - Primesense驱动 https://github.com/PrimeSense/Sensor

这两个版本库已被Fork到组织账户下，以防万一。

##### 安装必要的ros packages：

```bash
sudo apt-get install ros-indigo-openni2-launch
```
