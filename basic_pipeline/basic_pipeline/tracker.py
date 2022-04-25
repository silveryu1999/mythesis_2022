import sys
import time
import math
import numpy as np
import multiprocessing
from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
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

        self.br = CvBridge()
        self.camera_frame_rate = 0
        self.tracking_once_time = 0.0

        # frame_history: store the frames for tracking
        self.frame_history_rwlock = rwlock.RWLockFair()
        self.frame_history_rlock = self.frame_history_rwlock.gen_rlock()
        self.frame_history_wlock = self.frame_history_rwlock.gen_wlock()
        self.frame_history = {0: None}
        
        # LKtracker: tracking using Lucas-Kanade algorithm
        self.LKtracker_rwlock = rwlock.RWLockFair()
        self.LKtracker_rlock = self.LKtracker_rwlock.gen_rlock()
        self.LKtracker_wlock = self.LKtracker_rwlock.gen_wlock()
        self.LKtracker_is_init = False
        self.LKtracker_previmg = None
        self.LKtracker_prevpoints = None
        self.LKtracker_prevboxes = None
        self.LKtracker_last_track_task_frame_id = 0
        self.LKtracker_last_update_frame_id = 0 # frame_id of last detect result that updates the tracker
        # self.LKtracker_feature_params = dict(maxCorners = 200, qualityLevel = 0.001, minDistance = 30)
        self.LKtracker_feature_params = dict(maxCorners = 300, qualityLevel = 0.003, minDistance = 25)
        self.LKtracker_lk_params = dict(winSize = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.get_logger().info('Tracker init done.')
        
    def camera_callback(self, msg):
        with self.frame_history_wlock:
            self.frame_history[msg.frame_id] = self.br.imgmsg_to_cv2(msg.frame)
            self.camera_frame_rate = msg.frame_rate

    def track_callback(self, msg):
        with self.LKtracker_wlock:
            if(self.LKtracker_is_init == True):
                # tracking from last track result
                curr_frame = self.br.imgmsg_to_cv2(msg.frame)
                curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

                tracking_once_time_start = time.time()
                new_boxes, new_points = self.tracking_once(self.LKtracker_previmg, curr_gray, self.LKtracker_prevpoints, self.LKtracker_prevboxes)
                self.tracking_once_time = time.time() - tracking_once_time_start

                self.LKtracker_previmg = curr_gray
                self.LKtracker_prevpoints = new_points
                self.LKtracker_prevboxes = new_boxes
                self.LKtracker_last_track_task_frame_id = msg.frame_id

                track_result = TrackResult()
                track_result.result_boxes = new_boxes
                track_result.frame_id = msg.frame_id
                track_result.last_detect_result_frame_id = self.LKtracker_last_update_frame_id
                track_result.process_time = self.tracking_once_time
                self.track_result_publisher.publish(track_result)

                self.get_logger().info('Track result of frame %d has been published to collector. track_from_frame_diff: %d | tracking_once_time: %fs.' % (msg.frame_id, msg.frame_id - self.LKtracker_last_update_frame_id, self.tracking_once_time))
            else:
                # waiting for tracker init
                self.LKtracker_last_track_task_frame_id = msg.frame_id

                self.get_logger().info('Tracker received frame %d from scheduler, but has been dropped due to tracker not init yet.' % (msg.frame_id))

    def detect_result_callback(self, msg):
        with self.LKtracker_wlock:
            if(msg.frame_id > self.LKtracker_last_update_frame_id):
                update_time_start = time.time()

                with self.frame_history_rlock:
                    detect_frame = self.frame_history[msg.frame_id]

                detect_frame_gray = cv2.cvtColor(detect_frame, cv2.COLOR_BGR2GRAY)
                self.LKtracker_previmg = detect_frame_gray
                self.LKtracker_prevpoints = cv2.goodFeaturesToTrack(detect_frame_gray, mask = None, **self.LKtracker_feature_params)
                self.LKtracker_prevboxes = msg.result_boxes

                # additional feature points
                # the 4 corner points and the central point of the boxes will be included in feature points
                for i in range(len(msg.result_boxes)):
                    a1 = np.array([[[msg.result_boxes[i].x1, msg.result_boxes[i].y1]]], dtype=np.float32)
                    a2 = np.array([[[msg.result_boxes[i].x1, msg.result_boxes[i].y2]]], dtype=np.float32)
                    a3 = np.array([[[msg.result_boxes[i].x2, msg.result_boxes[i].y1]]], dtype=np.float32)
                    a4 = np.array([[[msg.result_boxes[i].x2, msg.result_boxes[i].y2]]], dtype=np.float32)
                    a5 = np.array([[[(msg.result_boxes[i].x1 + msg.result_boxes[i].x2) / 2, (msg.result_boxes[i].y1 + msg.result_boxes[i].y2) / 2]]], dtype=np.float32)
                    self.LKtracker_prevpoints = np.concatenate((self.LKtracker_prevpoints, a1, a2, a3, a4, a5), axis = 0)
                
                # catch up with the last track frame
                if(self.LKtracker_last_track_task_frame_id - msg.frame_id > 0):
                    # calculate the interval
                    base_time = self.tracking_once_time
                    frame_time = 1 / self.camera_frame_rate
                    active_cache_size = msg.frame_id - self.LKtracker_last_track_task_frame_id
                    interval = 0
                    while interval + 1 < active_cache_size:
                        times = active_cache_size / (interval + 1)
                        if(times > 1):
                            times_round_up = math.ceil(times)
                            if(times_round_up * base_time <= frame_time):
                                break
                        else:
                            break
                        interval += 1

                    with self.frame_history_rlock:
                        last_track_frame = self.frame_history[self.LKtracker_last_track_task_frame_id]

                    # now tracking on active cache to catch up with the last track frame
                    active_cache_tracking_time_start = time.time()
                    new_boxes, new_points, new_img = self.object_tracking(msg.frame_id, last_track_frame, self.LKtracker_last_track_task_frame_id, interval)
                    active_cache_tracking_time = time.time() - active_cache_tracking_time_start
                    self.LKtracker_previmg = new_img
                    self.LKtracker_prevpoints = new_points
                    self.LKtracker_prevboxes = new_boxes

                    self.get_logger().info('Tracking on active cache to catch up with last track frame, costing %f seconds.' % (active_cache_tracking_time))
                else:
                    # detected frame already catchs up to last track frame, just return the init results
                    self.get_logger().info('Directly inited tracker.')
                    pass

                with self.frame_history_wlock:
                    # delete the old frames
                    for i in range(self.LKtracker_last_update_frame_id, msg.frame_id):
                        del self.frame_history[i]

                self.LKtracker_last_update_frame_id = msg.frame_id
                self.LKtracker_is_init = True

                updating_time = time.time() - update_time_start
                self.get_logger().info('Tracker received detect result of frame %d, and inited/updated tracker in %f seconds.' % (msg.frame_id, updating_time))
            else:
                self.get_logger().info('Tracker received detect result of frame %d but it is outdated, now dropping it.' % (msg.frame_id))

    def object_tracking(self, start_id, curr_frame, curr_frame_id, interval):
        # object tracking after received a detect result
        # tracking from the last detected frame to the current frame
        # this is running on the active cache

        start_frame_id = start_id
        end_frame_id = curr_frame_id
        new_img = None
        new_boxes = None
        good_new_points = None

        # check if interval exceeds the active cache
        if(interval + 1 >= end_frame_id - start_frame_id):
            # in this situation, just track once between the detected frame and the current frame
            curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
            new_boxes, good_new_points = self.tracking_once(self.LKtracker_previmg, curr_gray, self.LKtracker_prevpoints, self.LKtracker_prevboxes)
            new_img = curr_gray
        else:
            # tracking through the active cache by an interval
            index_i = start_frame_id
            index_j = start_frame_id + interval + 1
            flag = True
            first_exceed = True
            while index_j <= end_frame_id and flag == True:
                if(index_i == start_frame_id):
                    previmg = self.LKtracker_previmg
                    prevpoints = self.LKtracker_prevpoints
                    prevboxes = self.LKtracker_prevboxes
                    with self.frame_history_rlock:
                        nextimg = cv2.cvtColor(self.frame_history[index_j], cv2.COLOR_BGR2GRAY)
                    
                # tracking once
                new_boxes, good_new_points = self.tracking_once(previmg, nextimg, prevpoints, prevboxes)
                new_img = nextimg

                # update i, j
                index_i = index_j
                index_j = index_i + interval + 1
                if(index_j >= end_frame_id):
                    if(first_exceed == True):
                        index_j = end_frame_id
                        first_exceed = False
                    else:
                        flag = False
                
                if(flag == True):
                    # update prev info
                    previmg = nextimg
                    prevpoints = good_new_points
                    prevboxes = new_boxes
                    with self.frame_history_rlock:
                        nextimg = cv2.cvtColor(self.frame_history[index_j], cv2.COLOR_BGR2GRAY)
        
        return new_boxes, good_new_points, new_img

    def tracking_once(self, previmg, nextimg, prevpoints, prevboxes):
        # running object tracking once between two frames
        # both previmg and nextimg should be gray images
        # prevpoints should be the origin format that cv2.goodFeaturesToTrack returns

        # select feature points by calculating optical flow
        nextpoints, st, err = cv2.calcOpticalFlowPyrLK(previmg, nextimg, prevpoints, None, **self.LKtracker_lk_params)
        good_new = nextpoints[st == 1]
        good_old = prevpoints[st == 1]

        # find points that are in the boxes
        points_in_boxes_old = []
        points_in_boxes_new = []
        for i in range(len(prevboxes)):
            points_in_box_old = []
            points_in_box_new = []
            # check the old point if it is in the box
            for j in range(len(good_old)):
                if(Point_is_in_box(good_old[j], prevboxes[i]) == True):
                    points_in_box_old.append(good_old[j])
                    points_in_box_new.append(good_new[j])
            points_in_boxes_old.append(points_in_box_old)
            points_in_boxes_new.append(points_in_box_new)

        # moving the boxes according to the shift of points
        newboxes = prevboxes
        for i in range(len(prevboxes)):
            mean_shift_x, mean_shift_y = Get_mean_shift(points_in_boxes_old[i], points_in_boxes_new[i])
            newboxes[i].x1 = prevboxes[i].x1 + mean_shift_x
            newboxes[i].x2 = prevboxes[i].x2 + mean_shift_x
            newboxes[i].y1 = prevboxes[i].y1 + mean_shift_y
            newboxes[i].y2 = prevboxes[i].y2 + mean_shift_y
        
        # adjust to correct format
        good_new_points = good_new.reshape(-1, 1, 2)

        return newboxes, good_new_points
        

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