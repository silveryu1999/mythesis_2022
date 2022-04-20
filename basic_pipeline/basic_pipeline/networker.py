import os
import sys
import time
import websocket
import cv2
import json
import base64
import threading
import numpy as np
import multiprocessing
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import AbsBoxes
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
            
            # connect to web socket, single connect
            self.inf_timeout = 2147483647
            self.ws = websocket.WebSocket()
            self.ws.connect("ws://localhost:12345", timeout = self.inf_timeout)

            self.br = CvBridge()

            self.group = ReentrantCallbackGroup()
            self.detect_request_listener = self.create_subscription(DetectRequest, self.name + '_detect_request_network', self.request_callback, 100, callback_group=self.group)
            self.detect_response_publisher = self.create_publisher(DetectResponse, self.name + '_detect_response_network', 10, callback_group=self.group)
            self.detect_delay_publisher = self.create_publisher(DetectDelay, self.name + '_detect_delay', 10, callback_group=self.group)
            self.timer = self.create_timer(0.05, self.socket_callback, callback_group=self.group)

            if(self.network_on == True):
                self.get_logger().info('Networker init done. Simulating network delay with bandwidth file: %s' % (self.network_path))
            else:
                self.get_logger().info('Networker init done. Not simulating network delay.')
        else:
            super().__init__('networker')
            self.get_logger().info('Networker init failed. Check arguments and the bandwidth file path.')
            self.get_logger().info('Command: ros2 run basic_pipeline networker [client_name] [bandwidth_file_path]')
            self.get_logger().info('Optional arguments: [client_name] | Necessary arguments: [bandwidth_file_path]')

    def request_callback(self, msg):
        encode_start_time = time.time()
        # get frame
        frame = self.br.imgmsg_to_cv2(msg.frame)
        # encode
        img_encode = cv2.imencode('.jpg', frame)[1]
        # to np array
        numpy_encode = np.array(img_encode)
        # to base64 (bytes)
        data_encode_64 = base64.b64encode(numpy_encode)
        # to str
        str_encode = str(data_encode_64, 'utf-8')
        encode_time = time.time() - encode_start_time
        # network delay
        if(self.network_on == True):
            with self.network_throughput_index_lock:
                current_index = self.network_throughput_index
                self.network_throughput_index = (self.network_throughput_index + 1) % len(self.network_path)
            frame_bytes = sys.getsizeof(str_encode)
            bandwidth = self.network_throughput[current_index]
            network_delay = frame_bytes / bandwidth
            time.sleep(network_delay)
        else:
            network_delay = 0.0
        # make dict
        # if network is on, network delay = encode delay + frame_bytes / bandwidth
        # if network is off, network delay = encode delay
        data = {
            'frame': str_encode,
            'frame_id': msg.frame_id,
            'client_name': msg.client_name,
            'client_detector_send_time': msg.sending_timestamp,
            'client_networker_send_time': time.time(),
            'network_delay': encode_time + network_delay
        }
        # to json
        str_json = json.dumps(data)
        # sending
        self.ws.send(str_json)
        self.get_logger().info('Frame %d has been delivered to server. Total Delay: %fs | Encoding + Sending time: %fs | Network Delay: %fs' % (msg.frame_id, encode_time + network_delay, encode_time, network_delay))

    def socket_callback(self):
        self.timer.cancel()
        self.get_logger().info('Networker is processing socket callback in a loop.')
        while True:
            recv = self.ws.recv()
            # loads data
            data = json.loads(recv)
            response = DetectResponse()
            response.frame_id = data['frame_id']
            response.frame_processing_time = data['process_time']
            response.returning_timestamp = data['client_detector_send_time']
            response.network_delay = data['network_delay']
            response.server_name = data['server_name']
            # processing boxes
            for i in range(len(data['boxes'])):
                absbox = AbsBoxes()
                absbox.x1 = data['boxes'][i]['x1']
                absbox.y1 = data['boxes'][i]['y1']
                absbox.x2 = data['boxes'][i]['x2']
                absbox.y2 = data['boxes'][i]['y2']
                absbox.conf = data['boxes'][i]['conf']
                absbox.name = data['boxes'][i]['name']
                response.boxes.append(absbox)
            self.detect_response_publisher.publish(response)
            self.get_logger().info('Detect result of frame %d is back from server, deliver it to detector.' % (data['frame_id']))


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