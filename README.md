# mythesis_2022
A distributed and cloud-end collaborated object detection framework based on ROS2  
一个基于ROS2实现的分布式端云协同目标检测框架  
****  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic1.png)  
![image text](https://github.com/sysu18364109/mythesis_2022/blob/main/pic2.png)  
****  
ROS2版本：Galactic  
Python版本：3.8  
****  
客户端运行：  
为防止意外错误发生，最好按框架图中的逆拓扑顺序来启动各结点  
```
ros2 run basic_pipeline displayer [client_name]
ros2 run basic_pipeline tracker [client_name]
ros2 run basic_pipeline detector [client_name]
ros2 run basic_pipeline scheduler [client_name]
ros2 run basic_pipeline camera [client_name] [frame_rate]
```
  
服务器运行：  
```
ros2 run basic_pipeline server [server_name]
```
