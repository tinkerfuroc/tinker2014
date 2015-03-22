#d_cv

Author: bssthu

ROS-Version: indigo

Description: 将ros的Image转换为OpenCV支持的格式


###使用方法

测试primesense摄像头:


```bash
rosrun d_cv test_reading_image
```

保存一张primesense拍摄的照片:


```bash
rosrun d_cv test_writing_image xx   # 保存为share/d_cv/xx.png
```

可能需要启动的package:

```bash
rosrun openni2_camera openni2_camera_node
```

###package信息

######nodes
- test_reading_image
- test_writing_image

######订阅的topics
- /rgb/image  ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

- /depth/image_raw  ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

###配置方法

##### OpenCV

- 方案A

```bash
sudo apt-get install libopencv-dev  # 如果版本太旧，请自行编译
```

- 方案B

编译最新版OpenCV。
之前在OpenCV 2.4.8下可以正常使用。

  - http://opencv.org/

##### Primesense驱动

- 方案A

下载事先编译好的二进制包并安装：

  - https://drive.google.com/open?id=0B-kQc2-wuHntfjdXYm5EV042SnVtcnZXeUtuUTNYM3c1MlJ1LWhMdGtpV3ZYQ1g2OWZvdGc&authuser=0

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

