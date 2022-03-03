import sys
import time
import os
import threading
import math
import numpy as np
import multiprocessing

import itertools
import struct  # get_image_size
import imghdr  # get_image_size

from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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


def get_display_img(img, boxes, method, savename=None):
    #img = np.copy(self.br.imgmsg_to_cv2(img))
    img = np.copy(img)
    width = img.shape[1]
    height = img.shape[0]

    if(method == 0):
        # detect, red
        rgb = (255, 0, 0)
    else:
        # track, yellow
        rgb = (255, 255, 0)

    for i in range(len(boxes)):
        x1 = boxes[i].x1
        y1 = boxes[i].y1
        x2 = boxes[i].x2
        y2 = boxes[i].y2
        conf = boxes[i].conf
        name = boxes[i].name
        bbox_thick = int(0.6 * (height + width) / 600)

        msg = boxes[i].name + " " + str(round(conf, 3))
        t_size = cv2.getTextSize(msg, 0, 0.7, thickness = bbox_thick // 2)[0]
        c1, c2 = (x1, y1), (x2, y2)
        c3 = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
        cv2.rectangle(img, (x1, y1), (int(np.float32(c3[0])), int(np.float32(c3[1]))), rgb, -1)
        img = cv2.putText(img, msg, (c1[0], int(np.float32(c1[1] - 2))), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), bbox_thick // 2, lineType = cv2.LINE_AA)

        img = cv2.rectangle(img, (x1, y1), (x2, y2), rgb, bbox_thick)
    
    return img


class Displayer_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_displayer')
        
        #self.group = ReentrantCallbackGroup()
        #self.group1 = MutuallyExclusiveCallbackGroup()
        self.group1 = ReentrantCallbackGroup()
        #self.group2 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group1)
        self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.group1)
        self.track_result_listener = self.create_subscription(TrackResult, self.name + '_track_result', self.track_result_callback, 10, callback_group=self.group1)

        self.br = CvBridge()

        # 0: detect
        # 1: track
        #self.last_result_type_lock = threading.Lock()
        #self.last_result_type = 0
        self.detect_diff_threshold = 2
        self.track_diff_threshold = 4

        self.detect_result_rwlock = rwlock.RWLockWrite()
        self.detect_result_rlock = self.detect_result_rwlock.gen_rlock()
        self.detect_result_wlock = self.detect_result_rwlock.gen_wlock()
        self.detect_result_is_init = False
        self.detect_result_boxes = None
        self.detect_result_frame_id = 0
        self.detect_result_total_time = 0
        self.detect_result_process_time = 0
        self.detect_result_flight_time = 0
        self.detect_result_server_name = None

        self.track_result_rwlock = rwlock.RWLockWrite()
        self.track_result_rlock = self.track_result_rwlock.gen_rlock()
        self.track_result_wlock = self.track_result_rwlock.gen_wlock()
        self.track_result_is_init = False
        self.track_result_boxes = None
        self.track_result_frame_id = 0
        self.track_result_process_time = 0
        self.track_result_last_detect_update_id = 0

        self.get_logger().info('Displayer init done.')

    def camera_callback(self, msg):
        self.get_logger().info('Camera callback Frame ID: %d' % (msg.frame_id))
        current_frame = np.copy(self.br.imgmsg_to_cv2(msg.frame))
        current_frame_id = msg.frame_id
        detect_show_flag = False
        track_show_flag = False

        # show detect result
        with self.detect_result_rlock:
            if(self.detect_result_is_init == True and current_frame_id - self.detect_result_frame_id <= self.detect_diff_threshold):
                result_img = get_display_img(current_frame, self.detect_result_boxes, 0, savename=None)
                detect_show_flag = True
                cv2.imwrite('/home/silveryu1999/result/' + str(current_frame_id) + '.jpg', result_img)
                #cv2.imshow("Frame result", result_img)
                #cv2.waitKey(1)
                self.get_logger().info('Current Camera Frame ID: %d | Result Type: %s | Result Frame ID: %d | Frame Diff/Delay: %d | Total Time: %f | Server Detect Time: %f' % (current_frame_id, "Detect", self.detect_result_frame_id, current_frame_id - self.detect_result_frame_id, self.detect_result_total_time, self.detect_result_process_time))

        # show track result
        if(detect_show_flag == False):
            with self.track_result_rlock:
                if(self.track_result_is_init == True and current_frame_id - self.track_result_frame_id <= self.track_diff_threshold):
                    result_img = get_display_img(current_frame, self.track_result_boxes, 1, savename=None)
                    track_show_flag = True
                    cv2.imwrite('/home/silveryu1999/result/' + str(current_frame_id) + '.jpg', result_img)
                    #cv2.imshow("Frame result", result_img)
                    #cv2.waitKey(1)
                    self.get_logger().info('Current Camera Frame ID: %d | Result Type: %s | Result Frame ID: %d | Frame Diff/Delay: %d | Local Tracking Time: %f | Tracking From Frame: %d' % (current_frame_id, "Track", self.track_result_frame_id, current_frame_id - self.track_result_frame_id, self.track_result_process_time, self.track_result_last_detect_update_id))

        # show origin result
        if(detect_show_flag == False and track_show_flag == False):
            cv2.imwrite('/home/silveryu1999/result/' + str(current_frame_id) + '.jpg', current_frame)
            #cv2.imshow("Frame result", current_frame)
            #cv2.waitKey(1)
            self.get_logger().info('Current Camera Frame ID: %d | Result Type: %s' % (current_frame_id, "None"))

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
        displayer_node = Displayer_Node()
        executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
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