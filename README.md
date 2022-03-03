# mythesis_2022
A distributed and cloud-end collaborated object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同目标检测框架  
进度/Progression：开发中.../developing...  
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
项目中使用了读写锁，由于python3的threading模块没有自带的读写锁，因此使用了第三方实现的读写锁：[readerwriterlock 1.0.9](https://pypi.org/project/readerwriterlock/)(注意，在高Camera帧率导致高并发时，该读写锁底层可能会出现一些错误，如需要满足更高Camera帧率的安全使用，可以考虑将tracker部分用C++改写，而读写锁改用pthread中的实现)  
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
为防止意外错误发生，最好按框架图中的逆拓扑顺序来启动各结点  
```
cd your_ros2_workspace
. install/local_setup.bash
ros2 run basic_pipeline displayer [client_name]
ros2 run basic_pipeline tracker [client_name]
ros2 run basic_pipeline detector [client_name]
ros2 run basic_pipeline scheduler [client_name]
ros2 run basic_pipeline camera [client_name] [frame_rate]
```
  
服务器运行：  
```
cd your_ros2_workspace
. install/local_setup.bash
ros2 run basic_pipeline server [server_name]
```
