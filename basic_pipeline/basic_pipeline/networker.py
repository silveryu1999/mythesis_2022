import sys
import time
import threading
import numpy as np
import multiprocessing
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Srvinfo
from bspipeline_interfaces.msg import DetectRequest
from bspipeline_interfaces.msg import DetectResponse
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import rclpy
from rclpy.node import Node

class Networker_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 1):
            self.name = 'anonymous_client'
            self.network_on = False
            self.init_flag = True
        elif(len(sys.argv) == 2):
            self.name = sys.argv[1]
            self.network_on = False
            self.init_flag = True
        elif(len(sys.argv) == 3):
            self.name = sys.argv[1]
            self.network_path = sys.argv[2]
            self.network_on = True
            self.init_flag = True
        else:
            self.init_flag = False

        if(self.init_flag == False):
            super().__init__('networker')
            self.get_logger().info('Networker init failed. You should run command: ros2 run basic_pipeline networker [client_name] [throughput_file]')
            self.get_logger().info('Throughput file should be a path name.')
        else:
            super().__init__(self.name + '_networker')

            if(self.network_on == True):
                self.network_throughput = np.loadtxt(self.network_path)
                self.network_throughput_index = 0
                self.network_throughput_index_lock = threading.Lock()
            
            self.server_list = {}
            self.server_list_lock = threading.Lock()
            self.target_server = None
            self.last_state_is_no_server = False

            self.br = CvBridge()

            self.group = ReentrantCallbackGroup()
            self.srvinfo_listener = self.create_subscription(Srvinfo, 'server_info', self.srvinfo_callback, 10, callback_group=self.group)
            self.detect_request_listener = self.create_subscription(DetectRequest, self.name + '_detect_request_network', self.request_callback, 100, callback_group=self.group)
            self.detect_response_listener = self.create_subscription(DetectResponse, self.name + '_detect_response', self.response_callback, 100, callback_group=self.group)
            self.detect_response_publisher = self.create_publisher(DetectResponse, self.name + '_detect_response_network', 10, callback_group=self.group)
            self.timer1 = self.create_timer(1.0, self.select_srv_callback, callback_group=self.group)
            self.timer2 = self.create_timer(5.0, self.clear_srv_callback, callback_group=self.group)

            if(self.network_on == True):
                self.get_logger().info('Networker init done. Using network file: %s' % (self.network_path))
            else:
                self.get_logger().info('Networker init done. Not using network.')

    def request_callback(self, msg):
        if(self.target_server == None):
            self.get_logger().info('Frame %d has been dropped since no server is active.' % (msg.frame_id))
        else:
            if(self.network_on == True):
                current_frame = self.br.imgmsg_to_cv2(msg.frame)
                # frame_bytes = sys.getsizeof(current_frame) # bytes
                frame_bytes = 1393652

                with self.network_throughput_index_lock:
                    current_index = self.network_throughput_index
                    self.network_throughput_index = (self.network_throughput_index + 1) % len(self.network_path)
                
                bandwidth = self.network_throughput[current_index] # bytes per second
                
                # network delay: simulated by transmission delay
                self.get_logger().info('Frame %d: Size: %dB | Bandwidth: %dB/s' % (msg.frame_id, frame_bytes, bandwidth))
                delay = frame_bytes / bandwidth
                msg.network_delay = delay
                time.sleep(delay)

                pub = self.create_publisher(DetectRequest, self.target_server + '_detect_request', 10, callback_group=self.group)
                pub.publish(msg)
                self.get_logger().info('Frame %d network delay: %fs, and has been delivered to server: %s' % (msg.frame_id, delay, self.target_server))
                del pub
            else:
                pub = self.create_publisher(DetectRequest, self.target_server + '_detect_request', 10, callback_group=self.group)
                pub.publish(msg)
                self.get_logger().info('Frame %d has been delivered to server directly: %s' % (msg.frame_id, self.target_server))
                del pub


    def response_callback(self, msg):
        self.detect_response_publisher.publish(msg)

    def srvinfo_callback(self, msg):
        with self.server_list_lock:
            self.server_list[msg.server_name] = [msg.service_type, msg.service_name, msg.avg_detect_time]

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
                    self.get_logger().info('Server name: %s | Service type: %s | Service name: %s | Avg processing time: %fs' % (key, self.server_list[key][0], self.server_list[key][1], self.server_list[key][2]))
                    if(self.server_list[key][2] < min_process_time):
                        min_process_time = self.server_list[key][2]
                        min_process_time_server = key
                self.target_server = min_process_time_server
                self.last_state_is_no_server = False
                self.get_logger().info('Target server selected: %s' % (min_process_time_server))
    
    def clear_srv_callback(self):
        with self.server_list_lock:
            self.server_list.clear()


def main(args=None):
    rclpy.init(args=args)

    networker_node = Networker_Node()

    if(networker_node.init_flag == True):
        executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
        executor.add_node(networker_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            networker_node.destroy_node()
            rclpy.shutdown()
    else:
        networker_node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()