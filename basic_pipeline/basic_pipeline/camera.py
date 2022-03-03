import sys
import time
from bspipeline_interfaces.msg import Camera
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Camera_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 1):
            self.frame_rate = 15
            self.name = 'anonymous_client'
            self.init_flag = True
        elif(len(sys.argv) == 2):
            if(sys.argv[1].isdigit() == True):
                self.frame_rate = int(sys.argv[1])
                self.name = 'anonymous_client'
            else:
                self.frame_rate = 30
                self.name = sys.argv[1]
            self.init_flag = True
        elif(len(sys.argv) == 3 and sys.argv[2].isdigit() == True):
            self.frame_rate = int(sys.argv[2])
            self.name = sys.argv[1]
            self.init_flag = True
        else:
            self.init_flag = False

        if(self.init_flag == True):
            super().__init__(self.name + '_camera')

            self.camera_topic = self.name + '_camera_frame'
            self.publisher_ = self.create_publisher(Camera, self.camera_topic, 10)

            # Create a VideoCapture object
            # The argument '0' gets the default webcam.
            self.cap = cv2.VideoCapture("/home/silveryu1999/video.mp4")
            # Used to convert between ROS and OpenCV images
            self.br = CvBridge()

            timer_period = 1 / self.frame_rate  # seconds
            self.sending_frame_id = 1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info('Camera init done.')
        else:
            super().__init__('camera')
            self.get_logger().info('Camera init failed: You should specify client name and frame rate through the command line.')
            self.get_logger().info('Command: ros2 run basic_pipeline camera [client name] [frame rate]')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            camera = Camera()
            camera.frame_id = self.sending_frame_id
            camera.frame = self.br.cv2_to_imgmsg(frame)
            camera.frame_rate = self.frame_rate
            camera.timestamp = time.time()
            self.publisher_.publish(camera)
            self.get_logger().info('Client camera sending frame %d' % (self.sending_frame_id))
            self.sending_frame_id += 1


def main(args=None):
    rclpy.init(args=args)

    camera_node = Camera_Node()
    
    if(camera_node.init_flag == True):
        rclpy.spin(camera_node)
        camera_node.destroy_node()
        rclpy.shutdown()
    else:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()