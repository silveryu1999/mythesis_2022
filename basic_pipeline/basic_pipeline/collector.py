import os
import sys
import time
import glob
import csv
from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectResult
from bspipeline_interfaces.msg import TrackResult
from bspipeline_interfaces.msg import DisplayResult
from bspipeline_interfaces.msg import ResultDelay
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
        # Command:
        # ros2 run basic_pipeline collector [client_name] [ground_truth_directory]
        # Example:
        # ros2 run basic_pipeline collector client1 ./ground_truth/
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.
        # ground_truth_directory: necessary, value: directory of ground truth files or 0 (do not have ground truth or not calculating the performance).

        if(len(sys.argv) == 2):
            self.name = 'anonymous_client'
            if(sys.argv[1].isdigit() == True and sys.argv[1] == '0'):
                self.load_ground_truth = False
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[1]) == True):
                    self.load_ground_truth = True
                    self.ground_truth_directory = sys.argv[1]
                    self.init_flag = True
                else:
                    self.init_flag = False
        elif(len(sys.argv) == 3):
            self.name = sys.argv[1]
            if(sys.argv[2].isdigit() == True and sys.argv[2] == '0'):
                self.load_ground_truth = False
                self.init_flag = True
            else:
                if(os.path.exists(sys.argv[2]) == True):
                    self.load_ground_truth = True
                    self.ground_truth_directory = sys.argv[2]
                    self.init_flag = True
                else:
                    self.init_flag = False

        if(self.init_flag == True):
            super().__init__(self.name + '_collector')

            # loading ground truth
            if(self.load_ground_truth == True):
                self.ground_truth = []
                self.ground_truth_length = len(glob.glob(pathname = self.ground_truth_directory + '*.txt'))
                for i in range(self.ground_truth_length):
                    tmp = []
                    with open(self.ground_truth_directory + str(i+1) + ".txt", encoding="utf-8") as cf:
                        lines = csv.reader(cf, delimiter=",")
                        for line in lines:
                            tmp.append(line)
                    self.ground_truth.append(tmp)
        
            self.group1 = MutuallyExclusiveCallbackGroup()
            self.group2 = MutuallyExclusiveCallbackGroup()
            self.group3 = MutuallyExclusiveCallbackGroup()
            self.group4 = ReentrantCallbackGroup()
            self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group1)
            self.detect_result_listener = self.create_subscription(DetectResult, self.name + '_detect_result', self.detect_result_callback, 10, callback_group=self.group2)
            self.track_result_listener = self.create_subscription(TrackResult, self.name + '_track_result', self.track_result_callback, 10, callback_group=self.group3)
            self.display_result_publisher = self.create_publisher(DisplayResult, self.name + '_display_result', 10, callback_group=self.group4)
            self.result_delay_publisher = self.create_publisher(ResultDelay, self.name + '_result_delay', 10, callback_group=self.group4)

            self.camera_current_frame_lock = rwlock.RWLockFair()
            self.camera_current_frame_rlock = self.camera_current_frame_lock.gen_rlock()
            self.camera_current_frame_wlock = self.camera_current_frame_lock.gen_wlock()
            self.camera_current_frame = None
            self.camera_current_frame_id = 0

            if(self.load_ground_truth == True):
                self.get_logger().info('Collector init done. Calculating performance with ground truth files in directory: %s' % (self.ground_truth_directory))
            else:
                self.get_logger().info('Collector init done. Not calculating performance with ground truth files.')
        else:
            super().__init__('collector')
            self.get_logger().info('Collector init failed. Check arguments and the ground truth directory.')
            self.get_logger().info('Command: ros2 run basic_pipeline collector [client_name] [ground_truth_directory]')
            self.get_logger().info('Optional arguments: [client_name] | Necessary arguments: [ground_truth_directory]')

    def detect_result_callback(self, msg):
        result = DisplayResult()
        resultdelay = ResultDelay()
        result.method = 0
        resultdelay.method = 0

        with self.camera_current_frame_rlock:
            result.frame = self.camera_current_frame
            result.current_camera_frame_id = self.camera_current_frame_id
            resultdelay.current_frame_id = self.camera_current_frame_id

        result.display_result_frame_id = msg.frame_id
        result.result_boxes = msg.result_boxes
        result.total_time = msg.total_time
        result.network_delay = msg.network_delay
        result.server_detect_time = msg.process_time

        resultdelay.result_frame_id = msg.frame_id
        resultdelay.total_delay = resultdelay.current_frame_id - resultdelay.result_frame_id
        resultdelay.result_delay = resultdelay.total_delay

        if(self.load_ground_truth == True):
            result.performance_is_on = 1
            result.tp, result.tp_and_fp, result.tp_and_fn = self.cal_performance(result.current_camera_frame_id, msg.result_boxes)
        else:
            result.performance_is_on = 0

        self.display_result_publisher.publish(result)
        self.result_delay_publisher.publish(resultdelay)
        self.get_logger().info('Detect result of frame %d has been published to the displayer, and the result delay has been published to the scheduler.' % (msg.frame_id))
        
    def track_result_callback(self, msg):
        result = DisplayResult()
        resultdelay = ResultDelay()
        result.method = 1
        resultdelay.method = 1

        with self.camera_current_frame_rlock:
            result.frame = self.camera_current_frame
            result.current_camera_frame_id = self.camera_current_frame_id
            resultdelay.current_frame_id = self.camera_current_frame_id

        result.display_result_frame_id = msg.frame_id
        result.result_boxes = msg.result_boxes
        result.local_tracking_time = msg.process_time
        result.tracking_from_frame = msg.last_detect_result_frame_id

        resultdelay.result_frame_id = msg.frame_id
        resultdelay.total_delay = resultdelay.current_frame_id - msg.last_detect_result_frame_id
        resultdelay.result_delay = resultdelay.current_frame_id - resultdelay.result_frame_id
        resultdelay.track_delay = resultdelay.result_frame_id - msg.last_detect_result_frame_id

        if(self.load_ground_truth == True):
            result.performance_is_on = 1
            result.tp, result.tp_and_fp, result.tp_and_fn = self.cal_performance(result.current_camera_frame_id, msg.result_boxes)
        else:
            result.performance_is_on = 0
        
        self.display_result_publisher.publish(result)
        self.result_delay_publisher.publish(resultdelay)
        self.get_logger().info('Track result of frame %d has been published to the displayer, and the result delay has been published to the scheduler..' % (msg.frame_id))

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

        '''
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
        '''
        
        return hit, len(target_boxes), len(gt_boxes)


def main(args=None):
    rclpy.init(args=args)

    collector_node = Collector_Node()

    if(collector_node.init_flag == True):
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(collector_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            collector_node.destroy_node()
            rclpy.shutdown()
    else:
        collector_node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()