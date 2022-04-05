import sys
import time
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectRequest
from bspipeline_interfaces.msg import DetectResponse
from bspipeline_interfaces.msg import DetectResult
import rclpy
from rclpy.node import Node


class Detector_Node(Node):

	def __init__(self):
		if(len(sys.argv) == 2):
			self.name = sys.argv[1]
		else:
			self.name = 'anonymous_client'
		super().__init__(self.name + '_detector')

		self.group = ReentrantCallbackGroup()
		self.detect_listener = self.create_subscription(Camera, self.name + '_detect_frame', self.detect_callback, 10, callback_group=self.group)
		self.detect_request_publisher = self.create_publisher(DetectRequest, self.name + '_detect_request_network', 10, callback_group=self.group)
		self.detect_response_listener = self.create_subscription(DetectResponse, self.name + '_detect_response_network', self.response_callback, 10, callback_group=self.group)
		self.detect_result_publisher = self.create_publisher(DetectResult, self.name + '_detect_result', 10, callback_group=self.group)

		self.last_detect_frame_id = 0
		self.last_detect_frame_id_lock = threading.Lock()

		self.get_logger().info('Detector init done.')

	def detect_callback(self, msg):
		request = DetectRequest()
		request.frame = msg.frame
		request.frame_id = msg.frame_id
		request.client_name = self.name
		request.sending_timestamp = time.time()
		self.detect_request_publisher.publish(request)
		self.get_logger().info('Frame %d has been delivered to the networker.' % (msg.frame_id))

	def response_callback(self, msg):
		with self.last_detect_frame_id_lock:
			if(msg.frame_id > self.last_detect_frame_id):
				flag = True
				self.last_detect_frame_id = msg.frame_id
			else:
				flag = False
		
		if(flag == True):
			result = DetectResult()
			result.result_boxes = msg.boxes
			result.frame_id = msg.frame_id
			result.server_name = msg.server_name
			total_time = time.time() - msg.returning_timestamp
			result.network_delay = msg.network_delay
			result.process_time = msg.frame_processing_time
			result.total_time = total_time
			result.flight_time = total_time - msg.frame_processing_time - msg.network_delay

			self.detect_result_publisher.publish(result)
			self.get_logger().info('Detect result of frame %d has been published to tracker and collector.' % (msg.frame_id))
		else:
			self.get_logger().info('Detect result of frame %d is outdated, now dropping it.' % (msg.frame_id))


def main(args=None):
    rclpy.init(args=args)

    try:
        detector_node = Detector_Node()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(detector_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            detector_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()