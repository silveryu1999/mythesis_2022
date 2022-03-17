# mythesis_2022  

## Intro  
A distributed and cloud-end collaborated object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同目标检测框架  

## Progress  
developing...  
~2022/03/06: 修复已有系统bug，除了最后的显示窗口，系统框架能完整运行  
~2022/03/10: 系统能够完整运行，同时修改系统框架设计，新增network模块  

## Framework    
Single Client-Server  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  
Distributed: Multi-Client Multi-Server  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic2.png)  

## Prerequisites
* OS：Ubuntu20.04（不推荐使用虚拟机） / Win10（还没测试过）  
* ROS2：Galactic/Foxy  
* Python：3.8  
* OpenCV：4.5+  
* CVbridge：2.2  

## TO RUN  
首先将basic_pipeline和bspipeline_interfaces复制到工作空间的src目录下  
除源文件package.xml标出的依赖之外，需要额外进行准备配置的有 [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)（含[opencv](https://docs.opencv.org/4.x/index.html)）、 [yolov4](https://github.com/Tianxiaomo/pytorch-YOLOv4)，可以参照网上教程进行配置，保证源代码import时能找到相应模块  
项目中使用了读写锁，由于python3的threading模块没有自带的读写锁，python3中也没有任何包含读写锁的官方模块，因此使用了第三方实现的读写锁：[readerwriterlock 1.0.9](https://pypi.org/project/readerwriterlock/)
```
python3 -m pip install -U readerwriterlock
```
创建ros2工作空间：
```
mkdir -p ~/your_ros2_workspace/src
```
复制项目文件，使得工作空间目录结构如下：
```
.
├── build    (build后生成)
├── install    (build后生成)
├── log    (build后生成)
├── src
│   ├── basic_pipeline
│   └── bspipeline_interfaces
├── vision_opencv    (注意参考前面cv_bridge的链接，安装相关依赖并且build完毕)
│   ├── cv_bridge
│   ├── image_geometry
│   ├── opencv_tests
│   ├── README.md
│   └── vision_opencv
└── yolov4
    ├── cfg
    ├── data
    ├── __init__.py
    ├── __pycache__
    ├── tool
    └── weight    (从前面yolov4的链接可以下载权重放到这里)
```
运行前构建并编译：  
```
cd your_ros2_workspace
colcon build
. install/local_setup.bash
``` 
刷新环境：以下两条命令在每次打开一个新的终端都需要输入以刷新，如果觉得麻烦，可以将这个刷新加到你的shell的bashrc文件
```
source /opt/ros/galactic/setup.bash 或
. ~/ros2_galactic/ros2-linux/setup.bash（刷新ros2环境，setup.bash的位置根据你安装ros2的方式（二进制包或源代码）会有所不同）
cd your_ros2_workspace
. install/local_setup.bash（刷新本地环境，否则会找不到我们自己构建的包）
```
客户端运行：  
每个结点都需要打开一个新终端来运行（即每个结点运行在单独的进程中），最好等其它客户端结点init完毕后，在最后才启动Camera 
```
ros2 run basic_pipeline displayer [client_name]
ros2 run basic_pipeline collector [client_name]
ros2 run basic_pipeline tracker [client_name]
ros2 run basic_pipeline detector [client_name]
ros2 run basic_pipeline networker [client_name] [network_type] [throughput_file]
ros2 run basic_pipeline scheduler [client_name]
-----------------------------------------------
ros2 run basic_pipeline camera [client_name] [frame_rate]
```
服务器运行：  
最好在客户端启动前启动，避免开始的请求没被接收到  
```
cd your_ros2_workspace
. install/local_setup.bash
ros2 run basic_pipeline server [server_name]
```  

## TO DO  
- [ ] 传输用于检测的图片时，对图片进行压缩  
- [ ] 新增networker，完成对实际网络吞吐量的仿真  
- [ ] collector对结果进行得分评估，包含准确率(IOU)、召回率、F1-Score三个指标  
- [ ] scheduler根据网络延迟和结果得分评估调整检测和跟踪间隔  
- [ ] networker和server之间的交互从message方式替换成TCP socket，server将不再运行在ros2结点上，同时修改client与server之间的发现方式  
- [ ] client之间的结点通过共享内存方式减少传递帧的开销  

