import sys
import time
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.srv import Bspip
from bspipeline_interfaces.msg import Detect
from bspipeline_interfaces.msg import DetectResult
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Detector_node(Node):

	def __init__(self):
		if(len(sys.argv) == 2):
			self.name = sys.argv[1]
		else:
			self.name = 'anonymous_client'
		super().__init__(self.name + '_detector')

		self.group = ReentrantCallbackGroup()
		self.detect_listener = self.create_subscription(Detect, self.name + '_detect_frame', self.detect_callback, 30, callback_group=self.group)
		self.detect_result_publisher = self.create_publisher(DetectResult, self.name + '_detect_result', 30, callback_group=self.group)

		self.get_logger().info('Detector init done.')

	def detect_callback(self, msg):
		self.get_logger().info('-----------------------------------------')
		self.get_logger().info('Frame %d has been received from scheduler.' % (msg.camera.frame_id))
		srv_cli = self.create_client(Bspip, msg.target_service_name, callback_group=self.group)
		counter = 0
		while not srv_cli.wait_for_service(timeout_sec=1.0) and counter < 3:
			self.get_logger().info('Target server not available, trying again...')
			counter += 1
		if(counter == 3):
			self.get_logger().info('Target server still not available, dropping this detect frame.')
		else:
			detect_req = Bspip.Request()
			detect_req.frame = msg.camera.frame
			detect_req.frame_id_send = msg.camera.frame_id
			detect_req.client_name = self.name
			detect_req.sending_timestamp = time.time()
			future = srv_cli.call_async(detect_req)
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
						self.get_logger().info('Result: FrameID: %d | Handled by: %s | Total_time:%.4fs Process_time:%.4fs Flight_time:%.4fs' % (response.frame_id_return, response.server_name, total_time, process_time, flight_time))
						#for i in range(len(response.boxes)):
						#	self.get_logger().info('Boxes[%d]: x1:%f y1:%f x2:%f y2:%f conf:%f name:%s' % (i, response.boxes[i].x1, response.boxes[i].y1, response.boxes[i].x2, response.boxes[i].y2, response.boxes[i].conf,response.boxes[i].name))
						result = DetectResult()
						result.result_boxes = response.boxes
						result.frame_id = response.frame_id_return
						result.total_time = total_time
						result.process_time = process_time
						result.flight_time = flight_time
						result.server_name = response.server_name
						result.width = response.width
						result.height = response.height
						self.detect_result_publisher.publish(result)
						self.get_logger().info('Detect result of frame %d has been published to tracker and displayer.' % (msg.camera.frame_id))
					break
		del future
		del srv_cli
		

def main(args=None):
    rclpy.init(args=args)

    try:
        detector_node = Detector_node()
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