# mythesis_2022
A distributed and cloud-end collaborated object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同目标检测框架  
****
进度/Progression：开发中.../developing...  
~2022/03/06: 修复已有系统bug，除了最后的显示窗口，系统框架能完整运行  
****  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic2.png)  
****  
ROS2版本：Galactic  
Python版本：3.8  
****  
TO RUN:  
首先将basic_pipeline和bspipeline_interfaces复制到工作空间的src目录下  
此外，还需要准备好opencv, cv_bridge, yolov4，保证源代码import时能找到相应模块  
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
