# mythesis_2022  
## Intro  
A distributed and cloud-end collaborated object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同目标检测框架  
## Progress  
developing...  
~2022/03/06: 修复已有系统bug，除了最后的显示窗口，系统框架能完整运行  
~2022/03/10: 系统能够完整运行，同时修改系统框架设计，新增network模块  
## Framework    
单机情境（Single Client-Server）：
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  
分布式情境（Distributed: Multi-Client Multi-Server）：
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic2.png)  
## Prerequisites
OS：Ubuntu20.04(VM is not recommended) / Win10(Not testing yet)  
ROS2：Galactic/Foxy  
Python：3.8  
OpenCV：4.5+  
CVbridge：2.2  
## TO RUN  
首先将basic_pipeline和bspipeline_interfaces复制到工作空间的src目录下  
除源文件package.xml标出的依赖之外，需要额外进行准备配置的有[opencv](https://docs.opencv.org/4.x/index.html)、 [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)、 [yolov4](https://github.com/Tianxiaomo/pytorch-YOLOv4)，可以参照网上教程进行配置，保证源代码import时能找到相应模块  
项目中使用了读写锁，由于python3的threading模块没有自带的读写锁，python3中也没有任何包含读写锁的官方模块，因此使用了第三方实现的读写锁：[readerwriterlock 1.0.9](https://pypi.org/project/readerwriterlock/)
```
python3 -m pip install -U readerwriterlock
```
运行前编译：  
```
cd your_ros2_workspace
colcon build
. install/local_setup.bash
```
客户端运行：  
为防止意外错误发生，最好按框架图中的逆拓扑顺序来启动各结点（等其它客户端结点init完毕后，Camera在最后才启动就行）  
```
cd your_ros2_workspace
. install/local_setup.bash
ros2 run basic_pipeline displayer [client_name]
ros2 run basic_pipeline collector [client_name]
ros2 run basic_pipeline tracker [client_name]
ros2 run basic_pipeline detector [client_name]
ros2 run basic_pipeline scheduler [client_name]
-----------------------------------------------
ros2 run basic_pipeline camera [client_name] [frame_rate]
```
  
服务器运行：  
```
cd your_ros2_workspace
. install/local_setup.bash
ros2 run basic_pipeline server [server_name]
```
