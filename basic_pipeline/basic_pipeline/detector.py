import sys
import time
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.srv import Bspip
from bspipeline_interfaces.msg import Detect
from bspipeline_interfaces.msg import DetectRequest
from bspipeline_interfaces.msg import DetectResponse
from bspipeline_interfaces.msg import DetectResult
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
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
		self.detect_listener = self.create_subscription(Detect, self.name + '_detect_frame', self.detect_callback, 10, callback_group=self.group)
		self.detect_response_listener = self.create_subscription(DetectResponse, self.name + '_detect_response', self.response_callback, 10, callback_group=self.group)
		self.detect_result_publisher = self.create_publisher(DetectResult, self.name + '_detect_result', 10, callback_group=self.group)

		self.last_detect_frame_id = 0
		self.last_detect_frame_id_lock = threading.Lock()

		self.get_logger().info('Detector init done.')

	def detect_callback(self, msg):
		# self.get_logger().info('Frame %d has been received from scheduler.' % (msg.camera.frame_id))
		request = DetectRequest()
		request.frame = msg.camera.frame
		request.frame_id = msg.camera.frame_id
		request.client_name = self.name
		request.sending_timestamp = time.time()

		pub = self.create_publisher(DetectRequest, msg.target_server + '_detect_request', 10, callback_group=self.group)
		pub.publish(request)
		self.get_logger().info('Frame %d has been delivered to the server: %s.' % (msg.camera.frame_id, msg.target_server))
		del pub

	def response_callback(self, msg):
		self.last_detect_frame_id_lock.acquire()
		if(msg.frame_id > self.last_detect_frame_id):
			self.last_detect_frame_id = msg.frame_id
			self.last_detect_frame_id_lock.release()

			result = DetectResult()
			result.result_boxes = msg.boxes
			result.frame_id = msg.frame_id
			result.server_name = msg.server_name
			total_time = time.time() - msg.returning_timestamp
			result.total_time = total_time
			result.process_time = msg.frame_processing_time
			result.flight_time = total_time - msg.frame_processing_time

			self.detect_result_publisher.publish(result)
			self.get_logger().info('Detect result of frame %d has been published to tracker and displayer.' % (msg.frame_id))
		else:
			self.last_detect_frame_id_lock.release()
			self.get_logger().info('Detect result of frame %d is outdated, now dropping it.' % (msg.frame_id))

	
	'''
	def detect_callback(self, msg):
		#self.get_logger().info('-----------------------------------------')
		#self.get_logger().info('Frame %d has been received from scheduler.' % (msg.camera.frame_id))
		srv_cli = self.create_client(Bspip, msg.target_service_name, callback_group=self.group)
		if not srv_cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('-----------------------------------------')
			self.get_logger().info('Target server not available, detect frame %d has been dropped.' % (msg.camera.frame_id))
		else:
			detect_req = Bspip.Request()
			detect_req.frame = msg.camera.frame
			detect_req.frame_id_send = msg.camera.frame_id
			detect_req.client_name = self.name
			detect_req.sending_timestamp = time.time()
			future = srv_cli.call_async(detect_req)
			self.get_logger().info('-----------------------------------------')
			self.get_logger().info('Frame %d has been delivered to the target server.' % (msg.camera.frame_id))
			while rclpy.ok():
				if future.done():
					try:
						response = future.result()
					except Exception as e:
						self.get_logger().info('Service call failed %r' % (e,))
					else:
						total_time = time.time() - response.returning_timestamp
						process_time = response.frame_processing_time
						flight_time = total_time - response.frame_processing_time
						self.get_logger().info('Detect result: FrameID: %d | Handled by: %s | Total_time:%.4fs Process_time:%.4fs Flight_time:%.4fs' % (response.frame_id_return, response.server_name, total_time, process_time, flight_time))
						#for i in range(len(response.boxes)):
						#	self.get_logger().info('Boxes[%d]: x1:%d y1:%d x2:%d y2:%d conf:%f name:%s' % (i, response.boxes[i].x1, response.boxes[i].y1, response.boxes[i].x2, response.boxes[i].y2, response.boxes[i].conf,response.boxes[i].name))
						result = DetectResult()
						result.result_boxes = response.boxes
						result.frame_id = response.frame_id_return
						result.total_time = total_time
						result.process_time = process_time
						result.flight_time = flight_time
						result.server_name = response.server_name
						self.detect_result_publisher.publish(result)
						self.get_logger().info('Detect result of frame %d has been published to tracker and displayer.' % (msg.camera.frame_id))
					break
	'''
		

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