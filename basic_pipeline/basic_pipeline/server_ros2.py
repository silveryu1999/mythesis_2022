import sys
import os
import time
import threading
import torch
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import AbsBoxes
from bspipeline_interfaces.msg import DetectRequest
from bspipeline_interfaces.msg import DetectResponse
from bspipeline_interfaces.msg import Srvinfo
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node
sys.path.append(os.getcwd()) # adding the current directory to the path, so as to import yolov4
# print(sys.path)
from yolov4 import *


class DetectService(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline server [server_name]
        # Example:
        # ros2 run basic_pipeline server server1
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # server_name: optional, value: the server name, if not set, 'anony_server' will be default.

        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anony_server'
        super().__init__(self.name)

        self.srv_name = self.name + '_bspip'
        self.group = ReentrantCallbackGroup()
        self.req_listener = self.create_subscription(DetectRequest, self.name + '_detect_request', self.frames_callback, 10, callback_group=self.group)
        self.pub = self.create_publisher(Srvinfo, 'server_info', 10, callback_group=self.group)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # self.get_logger().info('cwd: %s' % (os.getcwd()))
        # NewDarknet(Use_tiny, Use_cuda)
        self.use_cuda = True
        self.use_tiny = False
        self.darknet, self.class_names = NewDarknet(self.use_tiny, self.use_cuda)

        if (self.use_cuda == True and self.use_tiny == True):
            self.service_type = "yolov4-tiny-gpu"
        elif (self.use_cuda == True and self.use_tiny == False):
            self.service_type = "yolov4-gpu"
        elif (self.use_cuda == False and self.use_tiny == True):
            self.service_type = "yolov4-tiny-cpu"
        else:
            self.service_type = "yolov4-cpu"

        self.last_flag = True
        self.time_span = 10
        self.process_time_list = np.zeros(self.time_span)
        self.process_time_list_index = 0
        self.process_time_list_size = 0
        self.process_time_list_lock = threading.Lock()
        self.process_time_count = 0
        self.process_time_last_avg_time = 0

        self.srv_timer = self.create_timer(1.0, self.srvinfo_publisher, callback_group=self.group)
        self.get_logger().info('Server init done.')

    def frames_callback(self, msg):
        time_start = time.time()

        self.get_logger().info('Incoming: From: %s FrameID: %d' % (msg.client_name, msg.frame_id))

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg.frame)

        # detect
        sized = cv2.resize(current_frame, (self.darknet.width, self.darknet.height))
        sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

        boxes = do_detect(self.darknet, sized, 0.4, 0.6, self.use_cuda)
        boxes_zero = np.array(boxes[0]).tolist()

        width = current_frame.shape[1]
        height = current_frame.shape[0]

        torch.cuda.empty_cache()

        response = DetectResponse()

        for i in range(len(boxes_zero)):
            absbox = AbsBoxes()
            absbox.x1 = int(boxes_zero[i][0] * width)
            absbox.y1 = int(boxes_zero[i][1] * height)
            absbox.x2 = int(boxes_zero[i][2] * width)
            absbox.y2 = int(boxes_zero[i][3] * height)
            absbox.conf = boxes_zero[i][5]
            absbox.name = self.class_names[int(boxes_zero[i][6])]
            response.boxes.append(absbox)

        response.frame_id = msg.frame_id
        response.server_name = self.name
        response.width = width
        response.height = height
        response.network_delay = msg.network_delay
        response.returning_timestamp = msg.sending_timestamp
        time_end = time.time()
        frame_processing_time = time_end - time_start
        response.frame_processing_time = frame_processing_time

        # recording info
        with self.process_time_list_lock:
            self.process_time_list[self.process_time_list_index] = frame_processing_time
            self.process_time_list_index = (self.process_time_list_index + 1) % self.time_span
            if(self.process_time_list_size < self.time_span):
                self.process_time_list_size += 1

        pub = self.create_publisher(DetectResponse, msg.client_name + '_detect_response', 10, callback_group=self.group)
        pub.publish(response)
        del pub

    def srvinfo_publisher(self):
        with self.process_time_list_lock:
            if(self.process_time_list_size != 0):
                total_time = 0
                for i in range(self.process_time_list_size):
                    total_time += self.process_time_list[i]
                avg_process_time = total_time / self.process_time_list_size
                flag = True
            else:
                flag = False

        if(flag == True and self.process_time_last_avg_time == avg_process_time):
            self.process_time_count += 1
        if(self.process_time_count == self.time_span):
            self.process_time_list = np.zeros(self.time_span)
            self.process_time_list_index = 0
            self.process_time_list_size = 0
            self.process_time_last_avg_time = 0
            self.process_time_count = 0
            flag = False

        if(flag == True):
            self.process_time_last_avg_time = avg_process_time
            self.last_flag = True
        else:
            if(self.last_flag == True):
                self.get_logger().info('Server is idle now...')
                self.last_flag = False
        
        srv_info = Srvinfo()
        srv_info.server_name = self.name
        srv_info.service_type = self.service_type
        srv_info.service_name = self.srv_name
        srv_info.avg_detect_time = float(self.process_time_last_avg_time)
        self.pub.publish(srv_info)


def main():
    rclpy.init()

    try:
        detect_service = DetectService()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(detect_service)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            detect_service.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
