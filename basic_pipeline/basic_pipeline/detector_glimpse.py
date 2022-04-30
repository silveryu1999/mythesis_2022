import sys
import time
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectRequest
from bspipeline_interfaces.msg import DetectResponse
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import DetectDelay
import rclpy
from rclpy.node import Node


class Detector_Node(Node):
	
	def __init__(self):
		# Command:
		# ros2 run basic_pipeline detector [client_name]
		# Example:
		# ros2 run basic_pipeline detector client1
		# Arguments:
		# (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
		# (Argument types: optional or necessary)
		# client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.

		if(len(sys.argv) == 2):
			self.name = sys.argv[1]
		else:
			self.name = 'anonymous_client'
		super().__init__(self.name + '_detector')

		self.group1 = MutuallyExclusiveCallbackGroup()
		self.group2 = MutuallyExclusiveCallbackGroup()
		self.group3 = ReentrantCallbackGroup()
		self.detect_listener = self.create_subscription(Camera, self.name + '_detect_frame', self.detect_callback, 10, callback_group=self.group1)
		self.detect_request_publisher = self.create_publisher(DetectRequest, self.name + '_detect_request_network', 10, callback_group=self.group3)
		self.detect_response_listener = self.create_subscription(DetectResponse, self.name + '_detect_response_network', self.response_callback, 10, callback_group=self.group2)
		self.detect_result_publisher = self.create_publisher(DetectResult, self.name + '_detect_result', 10, callback_group=self.group3)
		self.detect_delay_publisher = self.create_publisher(DetectDelay, self.name + '_detect_delay', 10, callback_group=self.group3)
		self.timer = self.create_timer(0.05, self.trigger_frame_callback, callback_group=self.group3)
		
		self.last_detect_frame_id = 0

		self.trigger_frame_init = False
		self.trigger_frame_in_flight = False
		self.trigger_frame_is_update = False
		self.last_sending_trigger_frame_id = 0

		self.last_trigger_frame_lock = threading.Lock()
		self.last_trigger_frame = None
		self.last_trigger_frame_id = 0
		
		self.get_logger().info('Detector init done.')
	
	def detect_callback(self, msg):
		self.trigger_frame_init = True

		with self.last_trigger_frame_lock:
			self.last_trigger_frame = msg.frame
			self.last_trigger_frame_id = msg.frame_id
			self.trigger_frame_is_update = True

	def trigger_frame_callback(self):
		self.timer.cancel()
		self.get_logger().info('Detector is processing trigger frame callback in a loop.')

		while self.trigger_frame_init == False:
			pass

		while True:
			if(self.trigger_frame_in_flight == False):
				request = DetectRequest()

				with self.last_trigger_frame_lock:
					request.frame = self.last_trigger_frame
					request.frame_id = self.last_trigger_frame_id

				if(request.frame_id > self.last_sending_trigger_frame_id):
					request.client_name = self.name
					request.sending_timestamp = time.time()
					self.last_sending_trigger_frame_id = request.frame_id
					self.detect_request_publisher.publish(request)
					self.trigger_frame_in_flight = True
					self.get_logger().info('Trigger Frame %d has been delivered to the networker.' % (request.frame_id))
				else:
					self.trigger_frame_is_update = False
					while self.trigger_frame_is_update == False:
						pass
		
	def response_callback(self, msg):
		self.trigger_frame_in_flight = False
		
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
			result.bandwidth = msg.bandwidth
			self.detect_result_publisher.publish(result)
			self.get_logger().info('Detect result of frame %d has been published to tracker and collector.' % (msg.frame_id))

			detectdelay = DetectDelay()
			detectdelay.frame_id = msg.frame_id
			detectdelay.total_delay = total_time
			detectdelay.network_delay = msg.network_delay
			detectdelay.process_time = msg.frame_processing_time
			detectdelay.bandwidth = msg.bandwidth
			self.detect_delay_publisher.publish(detectdelay)
		else:
			self.get_logger().info('Detect result of frame %d is outdated, now dropping it.' % (msg.frame_id))


def main(args=None):
	rclpy.init(args=args)
	
	try:
		detector_node = Detector_Node()
		executor = MultiThreadedExecutor(num_threads=6)
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