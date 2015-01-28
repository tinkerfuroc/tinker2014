#d_pcl
Author: bssthu

ROS-Version: indigo

Description: PCL驱动

将点云数据从 ros 转换到 pcl 的库。
使用方法请参考 ./test/show_xyzrgb.cpp

###使用方法

```bash
roslaunch d_pcl primesense_to_pcl.launch    # 启动转换程序
```

```bash
rosrun test_show_xyzrgb     # 用图形界面显示xyzrgb
rosrun test_sub_xyz         # 命令行显示xyz
rosrun test_sub_xyzrgb      # 命令行显示xyzrgb
rosrun test_reading_pcd     # 读取pcd文件,从share/d_pcl下查找
```

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

