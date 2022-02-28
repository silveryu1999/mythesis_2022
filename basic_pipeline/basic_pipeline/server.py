import sys
import os
import time
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.srv import Bspip
from bspipeline_interfaces.msg import Boxes
from bspipeline_interfaces.msg import Srvinfo
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

sys.path.append(os.getcwd())
# print(sys.path)
from yolov4 import *
import numpy as np

import rclpy
from rclpy.node import Node


class DetectService(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anony_server'
        super().__init__(self.name)

        self.srv_name = self.name + '_bspip'
        self.group = ReentrantCallbackGroup()
        self.srv = self.create_service(Bspip, self.srv_name, self.frames_callback, callback_group=self.group)
        self.pub = self.create_publisher(Srvinfo, 'server_info', 30, callback_group=self.group)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # self.get_logger().info('cwd: %s' % (os.getcwd()))
        # NewDarknet(Use_tiny, Use_cuda)
        self.use_cuda = False
        self.use_tiny = True
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


    def frames_callback(self, request, response):
        time_start = time.time()

        self.get_logger().info('Incoming: From: %s FrameID: %d' % (request.client_name,request.frame_id_send))

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(request.frame)

        # detect
        sized = cv2.resize(current_frame, (self.darknet.width, self.darknet.height))
        sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

        boxes = do_detect(self.darknet, sized, 0.4, 0.6, self.use_cuda)
        boxes_zero = np.array(boxes[0]).tolist()

        width = current_frame.shape[1]
        height = current_frame.shape[0]

        for i in range(len(boxes_zero)):
            response_boxes = Boxes()
            response_boxes.x1 = boxes_zero[i][0]
            response_boxes.y1 = boxes_zero[i][1]
            response_boxes.x2 = boxes_zero[i][2]
            response_boxes.y2 = boxes_zero[i][3]
            response_boxes.conf = boxes_zero[i][5]
            response_boxes.name = self.class_names[int(boxes_zero[i][6])]
            response.boxes.append(response_boxes)

        response.frame_id_return = request.frame_id_send
        response.server_name = self.name
        response.width = width
        response.height = height
        response.returning_timestamp = request.sending_timestamp

        # Display image detect result
        # result_img = plot_boxes_cv2(current_frame, boxes[0], savename=None, class_names=self.class_names)
        # cv2.imshow("Frame result", result_img)
        # cv2.waitKey(1)

        time_end = time.time()
        frame_processing_time = time_end - time_start
        response.frame_processing_time = frame_processing_time

        # recording info
        self.process_time_list_lock.acquire()
        self.process_time_list[self.process_time_list_index] = frame_processing_time
        self.process_time_list_index = (self.process_time_list_index + 1) % self.time_span
        if(self.process_time_list_size < self.time_span):
            self.process_time_list_size += 1
        self.process_time_list_lock.release()

        return response

    def srvinfo_publisher(self):
        self.process_time_list_lock.acquire()
        if(self.process_time_list_size != 0):
            total_time = 0
            for i in range(self.process_time_list_size):
                total_time += self.process_time_list[i]
            avg_process_time = total_time / self.process_time_list_size
            flag = True
        else:
            flag = False
        self.process_time_list_lock.release()

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
            # self.get_logger().info('Server avg processing time: %f' % (avg_process_time))
            self.process_time_last_avg_time = avg_process_time
            self.last_flag = True
        else:
            if(self.last_flag == True):
                self.get_logger().info('Server is idle recently...')
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
