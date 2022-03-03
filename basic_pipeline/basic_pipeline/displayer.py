import sys
import time
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Displayer_Node(Node):

    def __init__(self):
        return



def main(args=None):
    rclpy.init(args=args)

    try:
        displayer_node = Displayer_Node()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(displayer_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            displayer_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()