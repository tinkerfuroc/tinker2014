#d_ros_to_cv

Author: bssthu

ROS-Version: indigo

Description: 将ros的Image转换为OpenCV支持的格式


###使用方法

测试primesence摄像头:


```bash
rosrun d_ros_to_cv test_reading_image
```

###package信息

######nodes
- test_reading_image

######订阅的topics
- /rgb/image  ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

- /depth/image_raw  ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

###配置方法

安装必要的软件包：

```bash
sudo apt-get install ros-indigo-openni2-launch
```

可能需要的软件：

- https://github.com/PrimeSense/Sensor

- https://github.com/PrimeSense/Sensor
