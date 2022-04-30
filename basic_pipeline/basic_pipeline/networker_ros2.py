import os
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
from bspipeline_interfaces.msg import DetectDelay
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import rclpy
from rclpy.node import Node

class Networker_Node(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline networker [client_name] [bandwidth_file_path]
        # Example:
        # ros2 run basic_pipeline networker client1 ./bandwidth.txt
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
        # bandwidth_file_path: necessary, value: a specific bandwidth file path or 0 (not simulating network delay).

        if(len(sys.argv) == 2):
            self.name = 'anonymous_client'
            if(sys.argv[1].isdigit() == True and sys.argv[1] == '0'):
                self.network_on = False
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[1]) == True):
                    self.network_on = True
                    self.network_path = sys.argv[1]
                    self.init_flag = True
                else:
                    self.init_flag = False
        elif(len(sys.argv) == 3):
            self.name = sys.argv[1]
            if(sys.argv[2].isdigit() == True and sys.argv[2] == '0'):
                self.network_on = False
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[2]) == True):
                    self.network_on = True
                    self.network_path = sys.argv[2]
                    self.init_flag = True
                else:
                    self.init_flag = False
        else:
            self.init_flag = False

        if(self.init_flag == True):
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
            self.detect_delay_publisher = self.create_publisher(DetectDelay, self.name + '_detect_delay', 10, callback_group=self.group)
            self.timer1 = self.create_timer(1.0, self.select_srv_callback, callback_group=self.group)
            self.timer2 = self.create_timer(5.0, self.clear_srv_callback, callback_group=self.group)

            if(self.network_on == True):
                self.get_logger().info('Networker init done. Simulating network delay with bandwidth file: %s, length: %d' % (self.network_path, len(self.network_throughput)))
            else:
                self.get_logger().info('Networker init done. Not simulating network delay.')
        else:
            super().__init__('networker')
            self.get_logger().info('Networker init failed. Check arguments and the bandwidth file path.')
            self.get_logger().info('Command: ros2 run basic_pipeline networker [client_name] [bandwidth_file_path]')
            self.get_logger().info('Optional arguments: [client_name] | Necessary arguments: [bandwidth_file_path]')

    def request_callback(self, msg):
        if(self.target_server == None):
            self.get_logger().info('Frame %d has been dropped since no server is active.' % (msg.frame_id))
        else:
            if(self.network_on == True):
                current_frame = self.br.imgmsg_to_cv2(msg.frame)
                # bytes, after compress to jpg and encoding, the size should be about 1/8 ~ 1/9 of the origin size
                frame_bytes = int(sys.getsizeof(np.array(current_frame)) / 8) 

                with self.network_throughput_index_lock:
                    current_index = self.network_throughput_index
                    self.network_throughput_index = (self.network_throughput_index + 1) % len(self.network_throughput)
                
                bandwidth = self.network_throughput[current_index] # bytes per second
                if(bandwidth == 0):
                    # consider the package is lost
                    self.get_logger().info('Frame %d has been lost.')
                    return
                
                # network delay: simulated by transmission delay
                delay = frame_bytes / bandwidth
                msg.network_delay = delay
                time.sleep(delay)

                pub = self.create_publisher(DetectRequest, self.target_server + '_detect_request', 10, callback_group=self.group)
                pub.publish(msg)
                self.get_logger().info('Frame %d: Size: %dB | Bandwidth: %dB/s | Network Delay: %fs, has been delivered to server: %s' % (msg.frame_id, frame_bytes, bandwidth, delay, self.target_server))
                del pub
            else:
                pub = self.create_publisher(DetectRequest, self.target_server + '_detect_request', 10, callback_group=self.group)
                pub.publish(msg)
                self.get_logger().info('Frame %d has been delivered to server: %s directly.' % (msg.frame_id, self.target_server))
                del pub

    def response_callback(self, msg):
        self.detect_response_publisher.publish(msg)

        detectdelay = DetectDelay()
        detectdelay.frame_id = msg.frame_id
        detectdelay.network_delay = msg.network_delay
        detectdelay.process_time = msg.frame_processing_time
        self.detect_delay_publisher.publish(detectdelay)

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