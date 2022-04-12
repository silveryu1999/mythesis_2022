import sys
import time
import numpy as np
import multiprocessing
from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import AbsBoxes
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
from bspipeline_interfaces.msg import TrackDelay
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


def Get_mean_shift(old_points, new_points):
    if(len(old_points) == 0):
        return 0, 0
    total_x = 0
    total_y = 0
    for i in range(len(old_points)):
        total_x += new_points[i][0] - old_points[i][0]
        total_y += new_points[i][1] - old_points[i][1]
    mean_x = int(total_x / len(old_points))
    mean_y = int(total_y / len(old_points))
    return mean_x, mean_y
    
def Point_is_in_box(point, box):
    min_x = min(box.x1, box.x2)
    min_y = min(box.y1, box.y2)
    max_x = max(box.x1, box.x2)
    max_y = max(box.y1, box.y2)
    x = int(point[0])
    y = int(point[1])
    if(x >= min_x and x <= max_x and y >= min_y and y <= max_y):
        return True
    else:
        return False


class Tracker_Node(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline tracker [client_name]
        # Example:
        # ros2 run basic_pipeline tracker client1
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.

        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_tracker')

        self.group = ReentrantCallbackGroup()
        self.mutex_group1 = MutuallyExclusiveCallbackGroup()
        self.mutex_group2 = MutuallyExclusiveCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group)
        self.track_listener = self.create_subscription(Camera, self.name + '_track_frame', self.track_callback, 10, callback_group=self.mutex_group1)
        self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.mutex_group2)
        self.track_result_publisher = self.create_publisher(TrackResult, self.name + '_track_result', 10, callback_group=self.group)
        self.track_delay_publisher = self.create_publisher(TrackDelay, self.name + '_track_delay', 10, callback_group=self.group)

        self.br = CvBridge()

        # frame_history: store the frames for tracking
        self.frame_history = {}
        self.frame_history_rwlock = rwlock.RWLockFair()
        self.frame_history_rlock = self.frame_history_rwlock.gen_rlock()
        self.frame_history_wlock = self.frame_history_rwlock.gen_wlock()
        
        # LKtracker: tracking using Lucas-Kanade algorithm
        self.LKtracker_rwlock = rwlock.RWLockFair()
        self.LKtracker_rlock = self.LKtracker_rwlock.gen_rlock()
        self.LKtracker_wlock = self.LKtracker_rwlock.gen_wlock()
        self.LKtracker_is_init = False
        self.LKtracker_previmg = None
        self.LKtracker_prevpoints = None
        self.LKtracker_boxes = None
        self.LKtracker_last_update_frame_id = 0 # frame_id of last detect result that updates the tracker
        # self.LKtracker_feature_params = dict(maxCorners = 200, qualityLevel = 0.001, minDistance = 30)
        self.LKtracker_feature_params = dict(maxCorners = 300, qualityLevel = 0.003, minDistance = 25)
        self.LKtracker_lk_params = dict(winSize = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.get_logger().info('Tracker init done.')
        
    def camera_callback(self, msg):
        with self.frame_history_wlock:
            self.frame_history[msg.frame_id] = self.br.imgmsg_to_cv2(msg.frame)

    def track_callback(self, msg):
        with self.LKtracker_rlock:
            if(self.LKtracker_is_init == True):
                # local tracking
                track_time_start = time.time()
                track_result_msg, id_diff = self.tracking(self.br.imgmsg_to_cv2(msg.frame), msg.frame_id)
                tracking_time = time.time() - track_time_start

                track_result_msg.process_time = tracking_time
                self.track_result_publisher.publish(track_result_msg)

                trackdelay = TrackDelay()
                trackdelay.frame_id = msg.frame_id
                trackdelay.tracking_time = tracking_time
                self.track_delay_publisher.publish(trackdelay)
                
                self.get_logger().info('Track result of frame %d has been published to collector. frame_delay: %d | tracking_time: %fs.' % (msg.frame_id, id_diff, track_result_msg.process_time))
            else:
                self.get_logger().info('Tracker received frame %d from scheduler, but has been dropped due to tracker not init yet.' % (msg.frame_id))

    def detect_result_callback(self, msg):
        with self.LKtracker_wlock:
            if(msg.frame_id <= self.LKtracker_last_update_frame_id):
                self.get_logger().info('Tracker received detect result of frame %d but it is outdated, now dropping it.' % (msg.frame_id))
            else:
                update_time_start = time.time()
                with self.frame_history_rlock:
                    image = self.frame_history[msg.frame_id]
                
                # new tracker init
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                self.LKtracker_previmg = gray_image
                self.LKtracker_prevpoints = cv2.goodFeaturesToTrack(gray_image, mask = None, **self.LKtracker_feature_params)
                self.LKtracker_boxes = msg.result_boxes

                # the 4 corner points and the central point of the boxes will be included in feature points
                for i in range(len(msg.result_boxes)):
                    a1 = np.array([[[msg.result_boxes[i].x1, msg.result_boxes[i].y1]]], dtype=np.float32)
                    a2 = np.array([[[msg.result_boxes[i].x1, msg.result_boxes[i].y2]]], dtype=np.float32)
                    a3 = np.array([[[msg.result_boxes[i].x2, msg.result_boxes[i].y1]]], dtype=np.float32)
                    a4 = np.array([[[msg.result_boxes[i].x2, msg.result_boxes[i].y2]]], dtype=np.float32)
                    a5 = np.array([[[(msg.result_boxes[i].x1 + msg.result_boxes[i].x2) / 2, (msg.result_boxes[i].y1 + msg.result_boxes[i].y2) / 2]]], dtype=np.float32)
                    self.LKtracker_prevpoints = np.concatenate((self.LKtracker_prevpoints, a1, a2, a3, a4, a5), axis = 0)
                
                self.LKtracker_is_init = True
                self.LKtracker_last_update_frame_id = msg.frame_id
                updating_time = time.time() - update_time_start
                self.get_logger().info('Tracker received detect result of frame %d and updated tracker in %f seconds.' % (msg.frame_id, updating_time))

    def tracking(self, frame, frame_id):
        nextimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        nextpoints, st, err = cv2.calcOpticalFlowPyrLK(self.LKtracker_previmg, nextimg, self.LKtracker_prevpoints, None, **self.LKtracker_lk_params)
        good_new = nextpoints[st == 1]
        good_old = self.LKtracker_prevpoints[st == 1]

        points_in_boxes_old = []
        points_in_boxes_new = []
        for i in range(len(self.LKtracker_boxes)):
            points_in_box_old = []
            points_in_box_new = []
            # check the old point if it is in the box
            for j in range(len(good_old)):
                if(Point_is_in_box(good_old[j], self.LKtracker_boxes[i]) == True):
                    points_in_box_old.append(good_old[j])
                    points_in_box_new.append(good_new[j])
            points_in_boxes_old.append(points_in_box_old)
            points_in_boxes_new.append(points_in_box_new)

        track_result = TrackResult()
        for i in range(len(self.LKtracker_boxes)):
            mean_shift_x, mean_shift_y = Get_mean_shift(points_in_boxes_old[i], points_in_boxes_new[i])
            absbox = AbsBoxes()
            absbox.x1 = self.LKtracker_boxes[i].x1 + mean_shift_x
            absbox.x2 = self.LKtracker_boxes[i].x2 + mean_shift_x
            absbox.y1 = self.LKtracker_boxes[i].y1 + mean_shift_y
            absbox.y2 = self.LKtracker_boxes[i].y2 + mean_shift_y
            absbox.conf = self.LKtracker_boxes[i].conf
            absbox.name = self.LKtracker_boxes[i].name
            track_result.result_boxes.append(absbox)
        track_result.frame_id = frame_id

        track_result.last_detect_result_frame_id = self.LKtracker_last_update_frame_id
        diff = frame_id - self.LKtracker_last_update_frame_id
        
        return track_result, diff

        
def main(args=None):
    rclpy.init(args=args)

    try:
        tracker_node = Tracker_Node()
        executor = MultiThreadedExecutor(num_threads=multiprocessing.cpu_count())
        executor.add_node(tracker_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            tracker_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()