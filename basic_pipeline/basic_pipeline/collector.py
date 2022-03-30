import sys
import time
import csv
from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
from bspipeline_interfaces.msg import DisplayResult
import rclpy
from rclpy.node import Node


def Cal_IOU(gt_box, tg_box):
    xmin = min(int(gt_box[1]), tg_box.x1)
    xmax = max(int(gt_box[3]), tg_box.x2)
    ymin = min(int(gt_box[2]), tg_box.y1)
    ymax = max(int(gt_box[4]), tg_box.y2)
    w1 = int(gt_box[3]) - int(gt_box[1])
    h1 = int(gt_box[4]) - int(gt_box[2])
    w2 = tg_box.x2 - tg_box.x1
    h2 = tg_box.y2 - tg_box.y1

    uw = xmax - xmin
    uh = ymax - ymin
    cw = w1 + w2 - uw
    ch = h1 + h2 - uh
    carea = 0
    if cw <= 0 or ch <= 0:
        return 0.0
    
    area1 = w1 * h1
    area2 = w2 * h2
    carea = cw * ch
    uarea = area1 + area2 - carea

    return carea / uarea


class Collector_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_collector')
        
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group3)
        self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.group1)
        self.track_result_listener = self.create_subscription(TrackResult, self.name + '_track_result', self.track_result_callback, 10, callback_group=self.group2)
        self.display_result_publisher = self.create_publisher(DisplayResult, self.name + '_display_result', 10, callback_group=self.group3)

        # 0: detect
        # 1: track
        # 2: camera(origin)
        self.detect_diff_threshold = 4
        self.track_diff_threshold = 3

        self.detect_result_rwlock = rwlock.RWLockRead()
        self.detect_result_rlock = self.detect_result_rwlock.gen_rlock()
        self.detect_result_wlock = self.detect_result_rwlock.gen_wlock()
        self.detect_result_is_init = False
        self.detect_result_boxes = None
        self.detect_result_frame_id = 0
        self.detect_result_total_time = 0
        self.detect_result_process_time = 0
        self.detect_result_flight_time = 0
        self.detect_result_server_name = None

        self.track_result_rwlock = rwlock.RWLockRead()
        self.track_result_rlock = self.track_result_rwlock.gen_rlock()
        self.track_result_wlock = self.track_result_rwlock.gen_wlock()
        self.track_result_is_init = False
        self.track_result_boxes = None
        self.track_result_frame_id = 0
        self.track_result_process_time = 0
        self.track_result_last_detect_update_id = 0

        # loading ground truth
        self.ground_truth = []
        self.total_frame_num = 154
        for i in range(self.total_frame_num):
            tmp = []
            with open("/home/silveryu1999/ground_truth/" + str(i+1) + ".txt", encoding="utf-8") as cf:
                lines = csv.reader(cf, delimiter=",")
                for line in lines:
                    tmp.append(line)
            self.ground_truth.append(tmp)

        self.get_logger().info('Collector init done.')

    def camera_callback(self, msg):
        current_frame_id = msg.frame_id
        detect_show_flag = False
        track_show_flag = False

        d_result = DisplayResult()
        t_result = DisplayResult()
        d_result.frame = msg.frame
        d_result.current_camera_frame_id = current_frame_id
        t_result.frame = msg.frame
        t_result.current_camera_frame_id = current_frame_id

        with self.detect_result_rlock:
            if(self.detect_result_is_init == True):
                detect_show_flag = True
                d_result.method = 0
                d_result.display_result_frame_id = self.detect_result_frame_id
                d_result.result_boxes = self.detect_result_boxes
                d_result.total_time = self.detect_result_total_time
                d_result.server_detect_time = self.detect_result_process_time
                d_pre, d_rec, d_f1 = self.cal_performance(current_frame_id, self.detect_result_boxes)

        with self.track_result_rlock:
            if(self.track_result_is_init == True):
                track_show_flag = True
                t_result.method = 1
                t_result.display_result_frame_id = self.track_result_frame_id
                t_result.result_boxes = self.track_result_boxes
                t_result.local_tracking_time = self.track_result_process_time
                t_result.tracking_from_frame = self.track_result_last_detect_update_id
                t_pre, t_rec, t_f1 = self.cal_performance(current_frame_id, self.track_result_boxes)

        if(detect_show_flag == True and track_show_flag == True):
            if(d_f1 >= t_f1):
                d_result.precision = d_pre
                d_result.recall = d_rec
                d_result.f1_score = d_f1
                self.display_result_publisher.publish(d_result)
                self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))
            else:
                t_result.precision = t_pre
                t_result.recall = t_rec
                t_result.f1_score = t_f1
                self.display_result_publisher.publish(t_result)
                self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))
        elif(detect_show_flag == True or track_show_flag == True):
            if(detect_show_flag == True):
                d_result.precision = d_pre
                d_result.recall = d_rec
                d_result.f1_score = d_f1
                self.display_result_publisher.publish(d_result)
                self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))
            else:
                t_result.precision = t_pre
                t_result.recall = t_rec
                t_result.f1_score = t_f1
                self.display_result_publisher.publish(t_result)
                self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))
        else:
            d_result.method = 2
            self.display_result_publisher.publish(d_result)
            self.get_logger().info('Display result of frame %d has been published to the displayer.' % (current_frame_id))

    def detect_result_callback(self, msg):
        with self.detect_result_wlock:
            if(self.detect_result_is_init == False):
                self.detect_result_is_init = True
            
            if(msg.frame_id > self.detect_result_frame_id):
                self.detect_result_boxes = msg.result_boxes
                self.detect_result_frame_id = msg.frame_id
                self.detect_result_total_time = msg.total_time
                self.detect_result_process_time = msg.process_time
                self.detect_result_flight_time = msg.flight_time
                self.detect_result_server_name = msg.server_name
    
    def track_result_callback(self, msg):
        with self.track_result_wlock:
            if(self.track_result_is_init == False):
                self.track_result_is_init = True

            if(msg.frame_id > self.track_result_frame_id):
                self.track_result_boxes = msg.result_boxes
                self.track_result_frame_id = msg.frame_id
                self.track_result_process_time = msg.process_time
                self.track_result_last_detect_update_id = msg.last_detect_result_frame_id

    def cal_performance(self, gt_frame_id, target_boxes):
        # gt_boxes是ground truth中的boxes
        # target_boxes是检测/跟踪到的boxes
        # iou>=0.5且预测类别正确则认为成功检测/跟踪, 否则认为目标丢失
        # 对于每个ground truth只计算一次
        # precision: 成功检测或跟踪的对象数 / 所有预测对象数
        # recall: 成功检测或跟踪的对象数 / 所有ground truth对象数
        # f1_score: 2 * p * r / (p + r)

        gt_boxes = self.ground_truth[gt_frame_id - 1]
        is_track = [0 for i in range(len(gt_boxes))]
        hit = 0
        
        for i in range(len(target_boxes)):
            max_iou = 0
            max_index = 0
            for j in range(len(gt_boxes)):
                iou = Cal_IOU(gt_boxes[j], target_boxes[i])
                if iou > max_iou:
                    max_iou = iou
                    max_index = j
            # if max_iou >= 0.5 and gt_boxes[max_index][0] == target_boxes[i].name and is_track[max_index] == 0:
            if max_iou >= 0.5 and gt_boxes[max_index][0] == target_boxes[i].name and is_track[max_index] == 0:
                hit += 1
                is_track[max_index] = 1

        self.get_logger().info('gt_frame_id: %d hit: %d detect total: %d gt total: %d' % (gt_frame_id, hit, len(target_boxes), len(gt_boxes)))

        if(len(target_boxes) != 0):
            precision = hit / len(target_boxes)
        else:
            precision = 0.0
        if(len(gt_boxes) != 0):
            recall = hit / len(gt_boxes)
        else:
            recall = 0.0
        if precision == 0 and recall == 0:
            f1_score = 0.0
        else:
            f1_score = (2 * precision * recall) / (precision + recall)
        
        return precision, recall, f1_score


def main(args=None):
    rclpy.init(args=args)

    try:
        collector_node = Collector_Node()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(collector_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            collector_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()