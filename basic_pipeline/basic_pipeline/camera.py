import os
import sys
import time
from bspipeline_interfaces.msg import Camera
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


class Camera_Node(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline camera [client_name] [frame_rate] [video_path]
        # Example:
        # ros2 run basic_pipeline camera client1 10 ./video.mp4
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
        # frame_rate: optional, value: the frame rate of capturing, if not set, 10 will be default.
        # video_path: necessary, value: a specific video file path or 0 (aka the default webcam of the computer).

        if(len(sys.argv) == 2):
            self.frame_rate = 10
            self.name = 'anonymous_client'
            if(sys.argv[1].isdigit() == True and sys.argv[1] == '0'):
                self.video_path = None
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[1]) == True):
                    self.video_path = sys.argv[1]
                    self.init_flag = True
                else:
                    self.init_flag = False
        elif(len(sys.argv) == 3):
            if(sys.argv[1].isdigit() == True):
                self.frame_rate = int(sys.argv[1])
                self.name = 'anonymous_client'
                if(sys.argv[2].isdigit() == True and sys.argv[2] == '0'):
                    self.video_path = None
                    self.init_flag = True
                else:
                    if(os.path.exists(sys.argv[2]) == True):
                        self.video_path = sys.argv[2]
                        self.init_flag = True
                    else:
                        self.init_flag = False
            else:
                self.frame_rate = 10
                self.name = sys.argv[1]
                if(sys.argv[2].isdigit() == True and sys.argv[2] == '0'):
                    self.video_path = None
                    self.init_flag = True
                else:
                    if(os.path.exists(sys.argv[2]) == True):
                        self.video_path = sys.argv[2]
                        self.init_flag = True
                    else:
                        self.init_flag = False
        elif(len(sys.argv) == 4):
            self.frame_rate = int(sys.argv[2])
            self.name = sys.argv[1]
            if(sys.argv[3].isdigit() == True and sys.argv[3] == '0'):
                self.video_path = None
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[3]) == True):
                    self.video_path = sys.argv[3]
                    self.init_flag = True
                else:
                    self.init_flag = False
        else:
            self.init_flag = False

        if(self.init_flag == True):
            super().__init__(self.name + '_camera')
            
            # Create a VideoCapture object
            # The argument '0' gets the default webcam.
            if(self.video_path == None):
                self.cap = cv2.VideoCapture(0)
            else:
                self.cap = cv2.VideoCapture(self.video_path)
            # Used to convert between ROS and OpenCV images
            self.br = CvBridge()

            timer_period = 1 / self.frame_rate  # seconds
            self.sending_frame_id = 1
            self.camera_publisher = self.create_publisher(Camera, self.name + '_camera_frame', 10)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info('Camera init done.')
        else:
            super().__init__('camera')
            self.get_logger().info('Camera init failed. Check arguments and the video file path.')
            self.get_logger().info('Command: ros2 run basic_pipeline camera [client_name] [frame_rate] [video_path]')
            self.get_logger().info('Optional arguments: [client_name] [frame_rate] | Necessary arguments: [video_path]')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            camera = Camera()
            camera.frame_id = self.sending_frame_id
            camera.frame = self.br.cv2_to_imgmsg(frame)
            camera.frame_rate = self.frame_rate
            camera.timestamp = time.time()
            self.camera_publisher.publish(camera)
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