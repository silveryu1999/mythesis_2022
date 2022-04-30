import sys
import math
import time
import numpy as np
from readerwriterlock import rwlock
import multiprocessing
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectDelay
from bspipeline_interfaces.msg import TrackDelay
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Scheduler_Node(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline scheduler [client_name] [pixel_diff] [pixel_count_rate]
        # Example:
        # ros2 run basic_pipeline scheduler client1 35 0.2
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
        # pixel_diff: necessary, value: the threshold of the abs value difference of two pixels, this should be an int between 0 and 255
		# pixel_count_rate: necessary, value: the threshold of the 'different' pixels' rate, this should be a float between 0 and 1

        if(len(sys.argv) == 4):
            self.name = sys.argv[1]
            self.fi = int(sys.argv[2])
            self.fi_motion_rate = float(sys.argv[3])
        else:
            self.fi = int(sys.argv[1])
            self.fi_motion_rate = float(sys.argv[2])
            self.name = 'anonymous_client'
        super().__init__(self.name + '_scheduler')

        self.camera_frame_rate = 0

        self.detect_interval_lock = rwlock.RWLockFair()
        self.detect_interval_rlock = self.detect_interval_lock.gen_rlock()
        self.detect_interval_wlock = self.detect_interval_lock.gen_wlock()
        self.detect_interval = 0
        self.detect_counter = 0

        self.track_interval_lock = rwlock.RWLockFair()
        self.track_interval_rlock = self.track_interval_lock.gen_rlock()
        self.track_interval_wlock = self.track_interval_lock.gen_wlock()
        self.track_interval = 0
        self.track_counter = 0

        self.br = CvBridge()
        self.last_frame_msg = None

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group1)
        self.detect_delay_listener = self.create_subscription(DetectDelay, self.name + '_detect_delay', self.detect_delay_callback, 10, callback_group=self.group2)
        self.track_delay_listener = self.create_subscription(TrackDelay, self.name + '_track_delay', self.track_delay_callback, 10, callback_group=self.group3)
        self.detect_publisher = self.create_publisher(Camera, self.name + '_detect_frame', 10, callback_group=self.group4)
        self.track_publisher = self.create_publisher(Camera, self.name + '_track_frame', 10, callback_group=self.group4)

        self.get_logger().info('Scheduler init done.')

    def camera_callback(self, msg):
        # get the frame rate
        self.camera_frame_rate = msg.frame_rate

        # schedule the detect frame/task
        # identify the trigger frame
        if(msg.frame_id == 1):
            self.detect_publisher.publish(msg)
            self.last_frame_msg = msg
            self.get_logger().info('Frame %d is a trigger frame and has been submitted to the detector.' % (msg.frame_id))
        else:
            cal_start_time = time.time()
            last_frame_gray = cv2.cvtColor(self.br.imgmsg_to_cv2(self.last_frame_msg.frame), cv2.COLOR_BGR2GRAY)
            curr_frame_gray = cv2.cvtColor(self.br.imgmsg_to_cv2(msg.frame), cv2.COLOR_BGR2GRAY)

            last_frame_gray = last_frame_gray.astype(np.int16)
            curr_frame_gray = curr_frame_gray.astype(np.int16)

            frame_diff_gray = curr_frame_gray - last_frame_gray
            frame_diff_gray_abs = np.abs(frame_diff_gray)

            total_pixels = curr_frame_gray.shape[0] * curr_frame_gray.shape[1]
            pixels_diff_count = np.sum(frame_diff_gray_abs > self.fi)

            if(pixels_diff_count / total_pixels > self.fi_motion_rate):
                # this is a trigger frame
                self.detect_publisher.publish(msg)
                self.last_frame_msg = msg
                self.get_logger().info('Frame %d is a trigger frame and has been submitted to the detector. Pixels count: %d | Pixels rate: %f | Calculate time: %fs' % \
                (msg.frame_id, pixels_diff_count, pixels_diff_count / total_pixels, time.time() - cal_start_time))
            else:
                # this is not a trigger frame
                self.last_frame_msg = msg
                self.get_logger().info('Frame %d is not a trigger frame, hence ignore it. Pixels count: %d | Pixels rate: %f | Calculate time: %fs' % \
                (msg.frame_id, pixels_diff_count, pixels_diff_count / total_pixels, time.time() - cal_start_time))
        
        # schedule the track frame/task
        self.track_publisher.publish(msg)
        self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        '''
        if(self.track_counter == 0):
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        with self.track_interval_rlock:
            if(self.track_counter >= self.track_interval):
                self.track_counter = 0
            else:
                self.track_counter += 1
        '''
        
    def detect_delay_callback(self, msg):
        # the frame rate of detect should not exceed the bottlenet/speed of detect
        curr_detect_time = max(msg.network_delay, msg.process_time)
        curr_detect_frame_rate = 1 / curr_detect_time

        # an easy policy of scheduling
        # increase, decrease or keep the interval
        # if the abs of the difference of both frame rate is less than the threshold, keep the interval
        with self.detect_interval_wlock:
            old_interval = self.detect_interval

            if(abs((curr_detect_frame_rate) * (self.detect_interval + 1) - self.camera_frame_rate) > 1):
                if((curr_detect_frame_rate) * (self.detect_interval + 1) - self.camera_frame_rate < 0):
                    # increase the interval
                    self.detect_interval += 1
                else:
                    # decrease the interval
                    self.detect_interval = max(0, self.detect_interval - 1)
            
            if(self.detect_interval > old_interval):
                self.get_logger().info('Interval of detect: Increased. Current value: %d.' % (self.detect_interval))
            elif(self.detect_interval == old_interval):
                self.get_logger().info('Interval of detect: Keeped. Current value: %d.' % (self.detect_interval))
            else:
                self.get_logger().info('Interval of detect: Decreased. Current value: %d.' % (self.detect_interval))

    def track_delay_callback(self, msg):
        # the frame rate of track should not exceed the speed of track
        track_time = msg.tracking_time
        curr_track_frame_rate = 1 / track_time

        # an easy policy of scheduling
        # increase, decrease or keep the interval
        # if the abs of the difference of both frame rate is less than the threshold, keep the interval
        with self.track_interval_wlock:
            old_interval = self.track_interval

            if(abs((curr_track_frame_rate) * (self.track_interval + 1) - self.camera_frame_rate) > 1):
                if((curr_track_frame_rate) * (self.track_interval + 1) - self.camera_frame_rate < 0):
                    # increase the interval
                    self.track_interval += 1
                else:
                    # decrease the interval
                    self.track_interval = max(0, self.track_interval - 1)

            if(self.track_interval > old_interval):
                self.get_logger().info('Interval of track: Increased. Current value: %d.' % (self.track_interval))
            elif(self.track_interval == old_interval):
                self.get_logger().info('Interval of track: Keeped. Current value: %d.' % (self.track_interval))
            else:
                self.get_logger().info('Interval of track: Decreased. Current value: %d.' % (self.track_interval))
            

def main(args=None):
    rclpy.init(args=args)

    try:
        scheduler_node = Scheduler_Node()
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(scheduler_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            scheduler_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()