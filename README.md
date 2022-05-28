# mythesis_2022  

## Intro  
A distributed and end-cloud collaborated real-time object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同实时目标检测框架  

## Demo  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/demo.gif)  

## Progress  
developing...  
~2022/03/06: 修复已有系统bug，除了最后的显示窗口，系统框架能完整运行  
~2022/03/10: 系统能够完整运行，同时修改系统框架设计，新增network模块  
~2022/04/05: 完成networker模块仿真设计，修改其它模块  
~2022/04/08: 完善用户接口，完善性能信息，添加测试用例和ground truth  
~2022/04/12: 新增sanic server，同时添加websocket通信方式，调整了相关模块代码  
~2022/04/20: 添加自适应的检测和跟踪任务调度，添加性能监视模块，修改其它模块  
~2022/04/25: 添加仿真server，修正tracker的跟踪方式(使用上一次的track result进行跟踪，或收到检测结果后在cache上跳帧跟踪到最新帧)，修改其它模块  
~2022/04/30: 添加glimpse版本的scheduler和detector，修正其它模块的一些错误  
~2022/05/14: 完成答辩，提交最终版本  

## Framework     
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  

## Prerequisites
* OS：Ubuntu20.04（不推荐使用虚拟机） / Win10（还没测试过）  
* ROS2：Galactic/Foxy  
* Python：3.8+  
* OpenCV：4.5+  
* CVbridge：2.2+  
* Readerwriterlock： 1.0.9+  
* Sanic： 22.3.0+  
* Websockets： 10.2+（for sanic）  
* Websocket-client： 1.3.2+（for client node）  
## TO RUN  
### 获取相关依赖   
* [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
* [opencv](https://docs.opencv.org/4.x/index.html)
* [yolov4](https://github.com/Tianxiaomo/pytorch-YOLOv4)  
* [readerwriterlock 1.0.9](https://pypi.org/project/readerwriterlock/)  
* [Sanic](https://sanic.readthedocs.io/en/stable/index.html)  
* [websockets](https://websockets.readthedocs.io/en/stable/index.html)  
* [websocket-client](https://pypi.org/project/websocket-client/)  
  
一些注意事项：cv_bridge注意参照链接进行配置；yolov4可以直接用这里的代码，除了权重weights另需下载；由于python3的threading模块没有自带的读写锁，python3中也没有任何包含读写锁的官方模块，因此使用了第三方实现的读写锁；有两个websocket相关包分别被sanic和client使用；获取以上依赖之后，保证源代码在import时能找到相应模块即可。  
### 项目构建
创建ros2工作空间：
```
mkdir -p ~/your_ros2_workspace/src
```
从上述项目中将各目录文件加载到工作空间下，此时，工作空间目录结构应如下（ground_truth、ideos、network可以放置在其它的地方，只需在启动相应的client node时指明相应路径即可）：
```
.
├── build    (build后生成)
├── install    (build后生成)
├── log    (build后生成)
├── ground_truth
│   ├── xxx
│   │   ├── 1.txt
│   │   └── ...
│   └── ...
├── videos
│   ├── xxx.mp4
│   └── ...
├── network
│   ├── xxx.txt
│   └── ...
├── src
│   ├── basic_pipeline
│   │   ├── basic_pipeline
│   │   │   ├── camera.py
│   │   │   ├── collector.py
│   │   │   ├── detector.py
│   │   │   ├── detector_glimpse.py
│   │   │   ├── displayer.py
│   │   │   ├── monitor.py
│   │   │   ├── networker.py    （使用websocket通信方式的networker）
│   │   │   ├── networker_ros2.py    (使用ros2 topic通信方式的networker)
│   │   │   ├── scheduler.py
│   │   │   ├── scheduler_glimpse.py
│   │   │   ├── server_ros2.py    (构建在ros2 node之上的server)
│   │   │   ├── tracker.py
│   │   │   └── ...
│   │   └── ...
│   └── bspipeline_interfaces
│       ├── msg
│       │   ├── xxx.msg
│       │   └── ...
│       └── ...
├── server.py    (构建在Sanic之上的server)
├── simulate_server.py    (构建在Sanic之上的仿真server)
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
如果按以上树形目录结构部署项目，那么建议客户端结点都运行在 ~/your_ros2_workspace 路径下  
#### Camera:  
```
# Command:
ros2 run basic_pipeline camera [client_name] [frame_rate] [video_path]
# Example:
ros2 run basic_pipeline camera client1 10 ./videos/xxx.mp4
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
#### Scheduler (Glimpse Version):  
```
# Command:
ros2 run basic_pipeline scheduler [client_name] [pixel_diff] [pixel_count_rate]
# Example:
ros2 run basic_pipeline scheduler client1 25 0.1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
# pixel_diff: necessary, value: the threshold of the abs value difference of two pixels, this should be an int between 0 and 255
# pixel_count_rate: necessary, value: the threshold of the 'different' pixels' rate, this should be a float between 0 and 1
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
#### Detector (Glimpse Version):  
```
# Command:
ros2 run basic_pipeline detector_glimpse [client_name]
# Example:
ros2 run basic_pipeline detector_glimpse client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
#### Networker (Combined with server):  
```
# Command:
ros2 run basic_pipeline networker [client_name] [bandwidth_file_path]
# Example:
ros2 run basic_pipeline networker client1 ./network/xxx.txt
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
# bandwidth_file_path: necessary, value: a specific bandwidth file path or 0 (not simulating network delay).
```
#### Networker_ros2 (Combined with server_ros2):  
```
# Command:
ros2 run basic_pipeline networker_ros2 [client_name] [bandwidth_file_path]
# Example:
ros2 run basic_pipeline networker_ros2 client1 ./network/xxx.txt
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
ros2 run basic_pipeline collector client1 ./ground_truth/xxx/
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
#### Monitor:
```
# Command:
ros2 run basic_pipeline monitor [client_name]
# Example:
ros2 run basic_pipeline monitor client1
# Arguments:
# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
```
服务器运行：  
最好在客户端启动前启动，等待模型加载完毕，避免开始的请求没被接收到  
同样建议服务器结点都运行在 ~/your_ros2_workspace 路径下  
#### Server (Sanic server):  
```
# Command:
sanic server.app --port=12345 --workers=1 --no-access-logs
or more simply:
python3 server.py
# Note:
# Currently it only supports worker=1, it means that the server use one main process to handle the requests.
# The port can be modified, change it in both network and server.
```  
#### Simulate Server (Sanic server):  
```
# Command:
sanic simulate_server.app --port=12345 --workers=1 --no-access-logs
or more simply:
python3 simulate_server.py
# Note:
# Currently it only supports worker=1, it means that the server use one main process to handle the requests.
# The port can be modified, change it in both network and server.
```  
#### Server_ros2 (Ros2 node server):  
```
# Command:
ros2 run basic_pipeline server_ros2 [server_name]
# Example:
ros2 run basic_pipeline server server1
# Arguments:
# server_name: optional, value: the server name, if not set, 'anony_server' will be default.
```  

## TO DO  
- [x] 传输用于检测的图片时，对图片进行压缩  
- [x] 新增networker，完成对实际网络吞吐量的仿真  
- [x] collector对结果进行得分评估，包含准确率(IOU)、召回率、F1-Score三个指标  
- [x] scheduler根据网络延迟和跟踪延迟评估，调整检测和跟踪间隔  
- [x] networker和server之间的通信方式从ros2 topic方式替换成websocket，server将不再运行在ros2结点上  
- [x] 在多样网络环境下进行测试，获取更多测试结果  
- [ ] 提高sanic server的性能  
- [ ] 完成tcp socket通信方式下client与server之间的发现机制，为分布式框架多对多框架提供基础  
- [ ] client之间的结点通过共享内存方式减少传递帧的开销  
- [ ] 在系统能够正常运行同时提高camera帧率  
- [ ] 用roscpp重构客户端部分以提高性能  

