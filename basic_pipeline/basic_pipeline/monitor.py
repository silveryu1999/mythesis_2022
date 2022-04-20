import sys
import numpy as np
import matplotlib.pyplot as plt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import DetectDelay
from bspipeline_interfaces.msg import TrackDelay
from bspipeline_interfaces.msg import DisplayResult
import rclpy
from rclpy.node import Node


def cal_f1_score(tp, tp_and_fp, tp_and_fn):
    if(tp_and_fp != 0):
        precision = tp / tp_and_fp
    else:
        precision = 0.0

    if(tp_and_fn != 0):
        recall = tp / tp_and_fn
    else:
        recall = 0.0

    if(precision != 0.0 or recall != 0.0):
        f1_score = (2 * precision * recall) / (precision + recall)
    else:
        f1_score = 0.0

    return precision, recall, f1_score


class Monitor_Node(Node):
    def __init__(self):
        # Command:
        # ros2 run basic_pipeline monitor [client_name]
        # Example:
        # ros2 run basic_pipeline monitor client1
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.

        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_displayer')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.display_result_listener = self.create_subscription(DisplayResult, self.name + '_display_result', self.display_result_callback, 10, callback_group=self.group1)
        self.timer = self.create_timer(2, self.figure_callback, callback_group=self.group1)

        self.detect_index = 0
        # subplot1: detect f1_score
        self.detect_precision = np.array([])
        self.detect_recall = np.array([])
        self.detect_f1_score = np.array([])
        # subplot2: bandwidth
        self.bandwidth = np.array([])
        # subplot3: detect time
        self.detect_network_delay = np.array([])
        self.detect_process_time = np.array([])
        # subplot4: detect frame diff
        self.detect_result_delay = np.array([])

        self.track_index = 0
        # subplot5: track f1_score
        self.track_precision = np.array([])
        self.track_recall = np.array([])
        self.track_f1_score = np.array([])
        # subplot6: track time
        self.track_time = np.array([])
        # subplot7: track frame diff
        self.track_result_delay = np.array([])

        plt.ion()

        self.get_logger().info('Monitor init done.')
    
    def display_result_callback(self, msg):
        if(msg.method == 0):
            self.detect_index += 1
            precision, recall, f1_score = cal_f1_score(msg.tp, msg.tp_and_fp, msg.tp_and_fn)
            self.detect_precision = np.append(self.detect_precision, precision)
            self.detect_recall = np.append(self.detect_recall, recall)
            self.detect_f1_score = np.append(self.detect_f1_score, f1_score)
            self.detect_network_delay = np.append(self.detect_network_delay, msg.network_delay)
            self.detect_process_time = np.append(self.detect_process_time, msg.server_detect_time)
            self.detect_result_delay = np.append(self.detect_result_delay, msg.current_camera_frame_id - msg.display_result_frame_id)
            self.bandwidth = np.append(self.bandwidth, msg.bandwidth)
        elif(msg.method == 1):
            self.track_index += 1
            precision, recall, f1_score = cal_f1_score(msg.tp, msg.tp_and_fp, msg.tp_and_fn)
            self.track_precision = np.append(self.track_precision, precision)
            self.track_recall = np.append(self.track_recall, recall)
            self.track_f1_score = np.append(self.track_f1_score, f1_score)
            self.track_time = np.append(self.track_time, msg.local_tracking_time)
            self.track_result_delay = np.append(self.track_result_delay, msg.current_camera_frame_id - msg.tracking_from_frame)
        else:
            return

    def figure_callback(self):
        plt.clf()

        plt.subplot(2,4,1)
        if(self.detect_index > 0):
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_precision, label = 'pre')
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_recall, label = 'rec')
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_f1_score, label = 'f1')
            plt.legend()
            plt.title("Detect F1 Score")

        plt.subplot(2,4,2)
        if(self.detect_index > 0):
            plt.plot(np.array(range(1, self.detect_index + 1)), self.bandwidth)
            plt.ylabel("Bandwidth (Mb/s)")
            plt.title("Bandwidth")
        
        plt.subplot(2,4,3)
        if(self.detect_index > 0):
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_network_delay, label = 'network')
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_process_time, label = 'process')
            plt.ylabel("Seconds")
            plt.legend()
            plt.title("Detect Time Delay")
        
        plt.subplot(2,4,4)
        if(self.detect_index > 0):
            plt.plot(np.array(range(1, self.detect_index + 1)), self.detect_result_delay)
            plt.ylabel("Frame Diff")
            plt.title("Detect Frame Diff")

        plt.subplot(2,4,5)
        if(self.track_index > 0):
            plt.plot(np.array(range(1, self.track_index + 1)), self.track_precision, label = 'pre')
            plt.plot(np.array(range(1, self.track_index + 1)), self.track_recall, label = 'rec')
            plt.plot(np.array(range(1, self.track_index + 1)), self.track_f1_score, label = 'f1')
            plt.legend()
            plt.title("Track F1 Score")

        plt.subplot(2,4,6)
        if(self.track_index > 0):
            plt.plot(np.array(range(1, self.track_index + 1)), self.track_time)
            plt.ylabel("Seconds")
            plt.title("Track Time Delay")
        
        plt.subplot(2,4,7)
        if(self.track_index > 0):
            plt.plot(np.array(range(1, self.track_index + 1)), self.track_result_delay)
            plt.ylabel("Frame Diff")
            plt.title("Track Frame Diff")

        plt.subplot(2,4,8)
        
        plt.pause(0.5)


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor_node = Monitor_Node()
        executor = SingleThreadedExecutor()
        executor.add_node(monitor_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            monitor_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()