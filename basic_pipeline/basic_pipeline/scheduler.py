import sys
import time
import threading
import multiprocessing

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import Detect
from bspipeline_interfaces.msg import Srvinfo
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Scheduler_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_scheduler')

        self.target_server = None
        self.last_state_is_no_server = False

        self.interval = 1
        self.counter = 0
        self.server_list = {}
        self.server_list_lock = threading.Lock()

        self.group = ReentrantCallbackGroup()
        self.srvinfo_listener = self.create_subscription(Srvinfo, 'server_info', self.srvinfo_callback, 10, callback_group=self.group)
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group)
        self.detect_publisher = self.create_publisher(Detect, self.name + '_detect_frame', 10, callback_group=self.group)
        self.track_publisher = self.create_publisher(Camera, self.name + '_track_frame', 10, callback_group=self.group)
        self.timer1 = self.create_timer(1.0, self.select_srv_callback, callback_group=self.group)
        self.timer2 = self.create_timer(5.0, self.clear_srv_callback, callback_group=self.group)

        self.get_logger().info('Scheduler init done.')

    def srvinfo_callback(self, msg):
        with self.server_list_lock:
            self.server_list[msg.server_name] = [msg.service_type, msg.service_name, msg.avg_detect_time]

    def camera_callback(self, msg):
        self.get_logger().info('-----------------------------------------')

        if(self.counter == 0):
            if(self.target_server != None):
                detect = Detect()
                detect.camera = msg
                detect.target_server = self.target_server
                detect.target_service_name = self.target_server + '_bspip'
                self.detect_publisher.publish(detect)
                self.get_logger().info('Frame %d has been submitted to the detector, and will be send to %s.' % (msg.frame_id, self.target_server))
            else:
                self.get_logger().info('Frame %d has been dropped since no server is active.' % (msg.frame_id))
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))
        else:
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        if(self.counter >= self.interval):
            self.counter = 0
        else:
            self.counter += 1
    
    def select_srv_callback(self):
        with self.server_list_lock:
            if(len(self.server_list) == 0):
                if(self.last_state_is_no_server == False):
                    self.target_server = None
                    self.last_state_is_no_server = True
                    self.get_logger().info('-----------------------------------------')
                    self.get_logger().info('No server available now.')
            else:
                min_process_time = 1000.0
                min_process_time_server = None
                self.get_logger().info('-----------------------------------------')
                self.get_logger().info('Current server list:')
                for key in self.server_list:
                    self.get_logger().info('Server name: %s | Service type: %s | Service name: %s | Avg processing time: %f' % (key, self.server_list[key][0], self.server_list[key][1], self.server_list[key][2]))
                    if(self.server_list[key][2] < min_process_time):
                        min_process_time = self.server_list[key][2]
                        min_process_time_server = key
                self.target_server = min_process_time_server
                self.last_state_is_no_server = False
                self.get_logger().info('Target server selected: %s' % (min_process_time_server))
    
    def clear_srv_callback(self):
        with self.server_list_lock:
            self.server_list.clear()
            # self.get_logger().info('-----------------------------------------')
            # self.get_logger().info('Server list has been cleared!')
        

def main(args=None):
    rclpy.init(args=args)

    try:
        scheduler_node = Scheduler_Node()
        executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
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