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
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import DisplayResult
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


def get_display_img(img, boxes, method):
    #img = np.copy(self.br.imgmsg_to_cv2(img))
    img = np.copy(img)
    width = img.shape[1]
    height = img.shape[0]

    if(method == 0):
        # detect, red
        rgb = (0, 0, 255)
    else:
        # track, yellow
        rgb = (0, 255, 255)

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

        self.group = MutuallyExclusiveCallbackGroup()
        self.display_result_listener = self.create_subscription(DisplayResult, self.name + '_display_result', self.display_result_callback, 10, callback_group=self.group)

        self.br = CvBridge()

        self.get_logger().info('Displayer init done.')

    def display_result_callback(self, msg):
        current_frame = np.copy(self.br.imgmsg_to_cv2(msg.frame))

        if(msg.method == 0):
            # detect
            result_img = get_display_img(current_frame, msg.result_boxes, 0)
            cv2.imshow("Displayer of " + self.name, result_img)
            cv2.waitKey(1)
            #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '.jpg', result_img)
            self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Total Frame Diff/Delay: %d | Total Time: %f | Server Detect Time: %f' % \
            (msg.current_camera_frame_id, "Detect", msg.display_result_frame_id, msg.current_camera_frame_id - msg.display_result_frame_id, msg.total_time, msg.server_detect_time))
        elif(msg.method == 1):
            # track
            result_img = get_display_img(current_frame, msg.result_boxes, 1)
            cv2.imshow("Displayer of " + self.name, result_img)
            cv2.waitKey(1)
            #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '.jpg', result_img)
            self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Total Frame Diff/Delay: %d (result:%d tracking:%d) | Local Tracking Time: %f' % \
            (msg.current_camera_frame_id, "Track", msg.display_result_frame_id, msg.current_camera_frame_id - msg.tracking_from_frame, msg.current_camera_frame_id - msg.display_result_frame_id, msg.display_result_frame_id - msg.tracking_from_frame, msg.local_tracking_time))
        elif(msg.method == 2):
            # origin
            cv2.imshow("Displayer of " + self.name, current_frame)
            cv2.waitKey(1)
            #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '.jpg', current_frame)
            self.get_logger().info('Current Frame ID: %d | Result Type: %s' % (msg.current_camera_frame_id, "Origin"))
        else:
            self.get_logger().info('Displayer get undifined result.')


def main(args=None):
    rclpy.init(args=args)

    try:
        displayer_node = Displayer_Node()
        executor = SingleThreadedExecutor()
        #executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
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