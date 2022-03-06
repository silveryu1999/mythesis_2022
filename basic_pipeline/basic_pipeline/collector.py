import sys
import time
import os
import threading
import math
import numpy as np
import multiprocessing

from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
from bspipeline_interfaces.msg import DisplayResult
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Collector_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_collector')
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group3)
        self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.group1)
        self.track_result_listener = self.create_subscription(TrackResult, self.name + '_track_result', self.track_result_callback, 10, callback_group=self.group2)
        self.display_result_publisher = self.create_publisher(DisplayResult, self.name + '_display_result', 10, callback_group=self.group3)

        self.br = CvBridge()

        # 0: detect
        # 1: track
        # 2: camera(origin)
        self.detect_diff_threshold = 6
        self.track_diff_threshold = 10

        self.detect_result_rwlock = rwlock.RWLockRead()
        self.detect_result_rlock = self.detect_result_rwlock.gen_rlock()
        self.detect_result_wlock = self.detect_result_rwlock.gen_wlock()
        self.detect_result_is_init = False
        self.detect_result_boxes = None
        self.detect_result_frame_id = 0
        self.detect_result_total_time = 0
        self.detect_result_process_time = 0
        self.detect_result_flight_time = 0
        self.detect_result_server_name = None

        self.track_result_rwlock = rwlock.RWLockRead()
        self.track_result_rlock = self.track_result_rwlock.gen_rlock()
        self.track_result_wlock = self.track_result_rwlock.gen_wlock()
        self.track_result_is_init = False
        self.track_result_boxes = None
        self.track_result_frame_id = 0
        self.track_result_process_time = 0
        self.track_result_last_detect_update_id = 0

        self.get_logger().info('Collector init done.')

    def camera_callback(self, msg):
        current_frame_id = msg.frame_id
        detect_show_flag = False
        track_show_flag = False

        result = DisplayResult()
        result.frame = msg.frame
        result.current_camera_frame_id = current_frame_id

        # show detect result
        with self.detect_result_rlock:
            if(self.detect_result_is_init == True and current_frame_id - self.detect_result_frame_id <= self.detect_diff_threshold):
                detect_show_flag = True
                result.method = 0
                result.display_result_frame_id = self.detect_result_frame_id
                result.result_boxes = self.detect_result_boxes
                result.total_time = self.detect_result_total_time
                result.server_detect_time = self.detect_result_process_time

        # show track result
        if(detect_show_flag == False):
            with self.track_result_rlock:
                if(self.track_result_is_init == True and current_frame_id - self.track_result_frame_id <= self.track_diff_threshold):
                    track_show_flag = True
                    result.method = 1
                    result.display_result_frame_id = self.track_result_frame_id
                    result.result_boxes = self.track_result_boxes
                    result.local_tracking_time = self.track_result_process_time
                    result.tracking_from_frame = self.track_result_last_detect_update_id

        # show origin result
        if(detect_show_flag == False and track_show_flag == False):
            result.method = 2

        self.display_result_publisher.publish(result)
        self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))

    def detect_result_callback(self, msg):
        with self.detect_result_wlock:
            if(self.detect_result_is_init == False):
                self.detect_result_is_init = True
            
            if(msg.frame_id > self.detect_result_frame_id):
                self.detect_result_boxes = msg.result_boxes
                self.detect_result_frame_id = msg.frame_id
                self.detect_result_total_time = msg.total_time
                self.detect_result_process_time = msg.process_time
                self.detect_result_flight_time = msg.flight_time
                self.detect_result_server_name = msg.server_name
    
    def track_result_callback(self, msg):
        with self.track_result_wlock:
            if(self.track_result_is_init == False):
                self.track_result_is_init = True

            if(msg.frame_id > self.track_result_frame_id):
                self.track_result_boxes = msg.result_boxes
                self.track_result_frame_id = msg.frame_id
                self.track_result_process_time = msg.process_time
                self.track_result_last_detect_update_id = msg.last_detect_result_frame_id


def main(args=None):
    rclpy.init(args=args)

    try:
        collector_node = Collector_Node()
        executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
        executor.add_node(collector_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            collector_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()