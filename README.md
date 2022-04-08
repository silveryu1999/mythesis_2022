# mythesis_2022  

## Intro  
A distributed and end-cloud collaborated real-time object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同实时目标检测框架  

## Progress  
developing...  
~2022/03/06: 修复已有系统bug，除了最后的显示窗口，系统框架能完整运行  
~2022/03/10: 系统能够完整运行，同时修改系统框架设计，新增network模块  
~2022/04/05: 完成network模块仿真设计，修改其它模块  
~2022/04/08: 完善用户接口，完善性能信息，添加测试用例和ground truth  

## Framework    
Single Client-Server  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  
Distributed: Multi-Client Multi-Server  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic2.png)  

## Prerequisites
* OS：Ubuntu20.04（不推荐使用虚拟机） / Win10（还没测试过）  
* ROS2：Galactic/Foxy  
* Python：3.8+  
* OpenCV：4.5+  
* CVbridge：2.2+  

## TO RUN  
### 获取相关依赖   
* [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
* [opencv](https://docs.opencv.org/4.x/index.html)
* [yolov4](https://github.com/Tianxiaomo/pytorch-YOLOv4)  
* [readerwriterlock 1.0.9](https://pypi.org/project/readerwriterlock/)  
  
主要参照cv_bridge链接进行配置（这里我已经将yolov4上传，除了权重weights），保证源代码import时能找到相应模块。项目中还使用了读写锁，由于python3的threading模块没有自带的读写锁，python3中也没有任何包含读写锁的官方模块，因此使用了第三方实现的读写锁  
```
python3 -m pip install -U readerwriterlock
```
### 项目构建
创建ros2工作空间：
```
mkdir -p ~/your_ros2_workspace/src
```
从项目中将basic_pipeline和bspipeline_interfaces文件夹复制到工作空间的src目录下，复制yolov4文件夹到工作空间下，从上面cv_bridge链接获取vision_opencv文件夹到工作空间下。此时，工作空间目录结构如下：
```
.
├── build    (build后生成)
├── install    (build后生成)
├── log    (build后生成)
├── ground_truth
│   ├── ...
│   └── ...
├── videos
│   ├── xxx.mp4
│   └── ...
├── network
│   ├── xxx.txt
│   └── ...
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
    ├── tool
    └── weight    (从前面yolov4的链接可以下载权重放到这里)
```
运行前构建并编译：  
编译过程中可能遇到一些警告，可以忽略  
```
cd ~/your_ros2_workspace
colcon build
``` 
刷新环境：  
注：每打开一个新终端都要输入以下命令刷新环境，麻烦的话可以将这两条命令加到你的shell对应的bashrc文件中
```
// 刷新ros2环境，以使用ros2相关命令，setup.bash的位置根据你安装ros2的方式（二进制包或源代码）会有所不同
source /opt/ros/galactic/setup.bash    // 二进制包方式
. ~/ros2_galactic/ros2-linux/setup.bash    // 源代码方式

// 刷新本地环境，否则会找不到我们自己构建的包
. ~/your_ros2_workspace/install/local_setup.bash
```
### 项目运行
客户端运行：  
每个结点都需要打开一个新终端来运行（即每个结点运行在单独的进程中），建议等其它客户端结点init完毕后，最后再启动Camera  
注意一些需要提供路径的结点，检查当前运行路径和所需文件路径是否正确  
#### Camera:  
```
# Command:
ros2 run basic_pipeline camera [client_name] [frame_rate] [video_path]
# Example:
ros2 run basic_pipeline camera client1 10 ./video.mp4
# Arguments:
# (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
# (Argument types: optional or necessary)
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
# frame_rate: optional, value: the frame rate of capturing, if not set, 10 will be default.
# video_path: necessary, value: a specific video file path or 0 (aka the default webcam of the computer).
```
#### Scheduler:  
```
# Command:
ros2 run basic_pipeline scheduler [client_name]
# Example:
ros2 run basic_pipeline scheduler client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
#### Detector:  
```
# Command:
ros2 run basic_pipeline detector [client_name]
# Example:
ros2 run basic_pipeline detector client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
#### Networker:  
```
# Command:
ros2 run basic_pipeline networker [client_name] [bandwidth_file_path]
# Example:
ros2 run basic_pipeline networker client1 ./bandwidth.txt
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
# bandwidth_file_path: necessary, value: a specific bandwidth file path or 0 (not simulating network delay).
```
#### Tracker:  
```
# Command:
ros2 run basic_pipeline tracker [client_name]
# Example:
ros2 run basic_pipeline tracker client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
#### Collector:  
```
# Command:
ros2 run basic_pipeline collector [client_name] [ground_truth_directory]
# Example:
ros2 run basic_pipeline collector client1 ./ground_truth/
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
# ground_truth_directory: necessary, value: directory of ground truth files or 0 (do not have ground truth or not calculating the performance).
```
#### Displayer:
```
# Command:
ros2 run basic_pipeline displayer [client_name]
# Example:
ros2 run basic_pipeline displayer client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
服务器运行：  
最好在客户端启动前启动，避免开始的请求没被接收到  
#### Server:  
```
# Command:
ros2 run basic_pipeline server [server_name]
# Example:
ros2 run basic_pipeline server server1
# Arguments:
# server_name: optional, value: the server name, if not set, 'anony_server' will be default.
```  

## TO DO  
- [ ] 传输用于检测的图片时，对图片进行压缩  
- [x] 新增networker，完成对实际网络吞吐量的仿真  
- [x] collector对结果进行得分评估，包含准确率(IOU)、召回率、F1-Score三个指标  
- [ ] scheduler根据网络延迟和结果得分评估调整检测和跟踪间隔  
- [ ] networker和server之间的交互从message方式替换成TCP socket，server将不再运行在ros2结点上
- [ ] 完成上一点的基础上，修改client与server之间的发现方式  
- [ ] client之间的结点通过共享内存方式减少传递帧的开销  
- [ ] 在系统能够正常运行同时提高camera帧率  
- [ ] 用roscpp重构客户端部分以提高性能  

