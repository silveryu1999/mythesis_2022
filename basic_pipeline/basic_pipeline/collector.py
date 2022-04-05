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
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group1)
        self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.group2)
        self.track_result_listener = self.create_subscription(TrackResult, self.name + '_track_result', self.track_result_callback, 10, callback_group=self.group3)
        self.display_result_publisher = self.create_publisher(DisplayResult, self.name + '_display_result', 10, callback_group=self.group4)

        self.camera_current_frame_lock = rwlock.RWLockFair()
        self.camera_current_frame_rlock = self.camera_current_frame_lock.gen_rlock()
        self.camera_current_frame_wlock = self.camera_current_frame_lock.gen_wlock()
        self.camera_current_frame = None
        self.camera_current_frame_id = 0

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
        result = DisplayResult()
        result.method = 2

        with self.camera_current_frame_wlock:
            self.camera_current_frame = msg.frame
            self.camera_current_frame_id = msg.frame_id
        
        result.frame = msg.frame
        result.current_camera_frame_id = msg.frame_id
        self.display_result_publisher.publish(result)
        self.get_logger().info('Origin frame %d has been published to the displayer.' % (msg.frame_id))

    def detect_result_callback(self, msg):
        result = DisplayResult()
        result.method = 0

        with self.camera_current_frame_rlock:
            result.frame = self.camera_current_frame
            result.current_camera_frame_id = self.camera_current_frame_id

        result.display_result_frame_id = msg.frame_id
        result.result_boxes = msg.result_boxes
        result.total_time = msg.total_time
        result.network_delay = msg.network_delay
        result.server_detect_time = msg.process_time
        result.precision, result.recall, result.f1_score = self.cal_performance(result.current_camera_frame_id, msg.result_boxes)
        self.display_result_publisher.publish(result)
        self.get_logger().info('Detect result of frame %d has been published to the displayer.' % (msg.frame_id))
        
    def track_result_callback(self, msg):
        result = DisplayResult()
        result.method = 1

        with self.camera_current_frame_rlock:
            result.frame = self.camera_current_frame
            result.current_camera_frame_id = self.camera_current_frame_id

        result.display_result_frame_id = msg.frame_id
        result.result_boxes = msg.result_boxes
        result.local_tracking_time = msg.process_time
        result.tracking_from_frame = msg.last_detect_result_frame_id
        result.precision, result.recall, result.f1_score = self.cal_performance(result.current_camera_frame_id, msg.result_boxes)
        self.display_result_publisher.publish(result)
        self.get_logger().info('Track result of frame %d has been published to the displayer.' % (msg.frame_id))

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