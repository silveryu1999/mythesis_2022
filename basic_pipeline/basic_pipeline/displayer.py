import sys
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from bspipeline_interfaces.msg import DisplayResult
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rclpy
from rclpy.node import Node


def get_display_img(img, boxes, method):
    img = np.copy(img)
    width = img.shape[1]
    height = img.shape[0]

    if(method == 0):
        # detect, red
        rgb = (0, 0, 255)
    else:
        # track, yellow
        rgb = (0, 255, 255)

    for i in range(len(boxes)):
        x1 = boxes[i].x1
        y1 = boxes[i].y1
        x2 = boxes[i].x2
        y2 = boxes[i].y2
        conf = boxes[i].conf
        name = boxes[i].name
        bbox_thick = int(0.6 * (height + width) / 600)

        msg = name + " " + str(round(conf, 3))
        t_size = cv2.getTextSize(msg, 0, 0.7, thickness = bbox_thick // 2)[0]
        c1, c2 = (x1, y1), (x2, y2)
        c3 = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
        cv2.rectangle(img, (x1, y1), (int(np.float32(c3[0])), int(np.float32(c3[1]))), rgb, -1)
        img = cv2.putText(img, msg, (c1[0], int(np.float32(c1[1] - 2))), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), bbox_thick // 2, lineType = cv2.LINE_AA)

        img = cv2.rectangle(img, (x1, y1), (x2, y2), rgb, bbox_thick)
    
    return img

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


class Displayer_Node(Node):
    def __init__(self):
        # Command:
        # ros2 run basic_pipeline displayer [client_name]
        # Example:
        # ros2 run basic_pipeline displayer client1
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.

        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_displayer')

        self.group = MutuallyExclusiveCallbackGroup()
        self.display_result_listener = self.create_subscription(DisplayResult, self.name + '_display_result', self.display_result_callback, 10, callback_group=self.group)

        self.br = CvBridge()

        self.detect_total_tp = 0
        self.detect_total_tp_and_fp = 0
        self.detect_total_tp_and_fn = 0
        self.detect_total_result_delay = 0
        self.detect_total_total_time = 0
        self.detect_total_server_detect_time = 0
        self.detect_total_network_delay = 0
        self.detect_count = 0

        self.track_total_tp = 0
        self.track_total_tp_and_fp = 0
        self.track_total_tp_and_fn = 0
        self.track_total_result_delay = 0
        self.track_total_track_delay = 0
        self.track_total_track_time = 0
        self.track_count = 0

        self.is_origin = True

        self.get_logger().info('Displayer init done.')

    def display_result_callback(self, msg):
        current_frame = np.copy(self.br.imgmsg_to_cv2(msg.frame))

        if(msg.method == 0):
            # detect
            if(self.is_origin == True):
                self.is_origin = False

            result_img = get_display_img(current_frame, msg.result_boxes, 0)
            cv2.imshow("Displayer of " + self.name, result_img)
            cv2.waitKey(1)
            #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '_detect' + '.jpg', result_img)

            if(msg.performance_is_on == 1):
                precision, recall, f1_score = cal_f1_score(msg.tp, msg.tp_and_fp, msg.tp_and_fn)
                self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Precision: %f | Recall: %f | F1 Score: %f | Total Frame Diff/Delay: %d (Result) | Total Time: %fs | Server Detect Time: %fs | Network Delay: %fs' % \
                (msg.current_camera_frame_id, "Detect", msg.display_result_frame_id, precision, recall, f1_score, msg.current_camera_frame_id - msg.display_result_frame_id, msg.total_time, msg.server_detect_time, msg.network_delay))

                self.detect_total_tp += msg.tp
                self.detect_total_tp_and_fp += msg.tp_and_fp
                self.detect_total_tp_and_fn += msg.tp_and_fn
                self.detect_total_result_delay += msg.current_camera_frame_id - msg.display_result_frame_id
                self.detect_total_total_time += msg.total_time
                self.detect_total_server_detect_time += msg.server_detect_time
                self.detect_total_network_delay += msg.network_delay
                self.detect_count += 1

                avg_pre, avg_rec, avg_f1 = cal_f1_score(self.detect_total_tp, self.detect_total_tp_and_fp, self.detect_total_tp_and_fn)
                self.get_logger().info('Detect Average Performance: Total Frames: %d | Precision: %f | Recall: %f | F1 Score: %f | Total Frame Diff/Delay: %f (Result) | Total Time: %fs | Server Detect Time: %fs | Network Delay: %fs' % \
                (self.detect_count, avg_pre, avg_rec, avg_f1, self.detect_total_result_delay / self.detect_count, self.detect_total_total_time / self.detect_count, self.detect_total_server_detect_time / self.detect_count, self.detect_total_network_delay / self.detect_count))
            else:
                self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Total Frame Diff/Delay: %d (Result) | Total Time: %fs | Server Detect Time: %fs | Network Delay: %fs' % \
                (msg.current_camera_frame_id, "Detect", msg.display_result_frame_id, msg.current_camera_frame_id - msg.display_result_frame_id, msg.total_time, msg.server_detect_time, msg.network_delay))

        elif(msg.method == 1):
            # track
            if(self.is_origin == True):
                self.is_origin = False

            result_img = get_display_img(current_frame, msg.result_boxes, 1)
            cv2.imshow("Displayer of " + self.name, result_img)
            cv2.waitKey(1)
            #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '_track' + '.jpg', result_img)

            if(msg.performance_is_on == 1):
                precision, recall, f1_score = cal_f1_score(msg.tp, msg.tp_and_fp, msg.tp_and_fn)
                self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Precision: %f | Recall: %f | F1 Score: %f | Total Frame Diff/Delay: %d (Result:%d Tracking:%d) | Local Tracking Time: %fs' % \
                (msg.current_camera_frame_id, "Track", msg.display_result_frame_id, precision, recall, f1_score, msg.current_camera_frame_id - msg.tracking_from_frame, msg.current_camera_frame_id - msg.display_result_frame_id, msg.display_result_frame_id - msg.tracking_from_frame, msg.local_tracking_time))
                
                self.track_total_tp += msg.tp
                self.track_total_tp_and_fp += msg.tp_and_fp
                self.track_total_tp_and_fn += msg.tp_and_fn
                self.track_total_result_delay += msg.current_camera_frame_id - msg.display_result_frame_id
                self.track_total_track_delay += msg.display_result_frame_id - msg.tracking_from_frame
                self.track_total_track_time += msg.local_tracking_time
                self.track_count += 1

                avg_pre, avg_rec, avg_f1 = cal_f1_score(self.track_total_tp, self.track_total_tp_and_fp, self.track_total_tp_and_fn)
                self.get_logger().info('Track Average Performance: Total frames: %d | Precision: %f | Recall: %f | F1 Score: %f | Total Frame Diff/Delay: %f (Result:%f Tracking:%f) | Local Tracking Time: %fs' % \
                (self.track_count, avg_pre, avg_rec, avg_f1, (self.track_total_result_delay + self.track_total_track_delay) / self.track_count, self.track_total_result_delay / self.track_count, self.track_total_track_delay / self.track_count, self.track_total_track_time / self.track_count))
            else:
                self.get_logger().info('Current Frame ID: %d | Result Type: %s | Result Frame ID: %d | Total Frame Diff/Delay: %d (Result:%d Tracking:%d) | Local Tracking Time: %fs' % \
                (msg.current_camera_frame_id, "Track", msg.display_result_frame_id, msg.current_camera_frame_id - msg.tracking_from_frame, msg.current_camera_frame_id - msg.display_result_frame_id, msg.display_result_frame_id - msg.tracking_from_frame, msg.local_tracking_time))

        elif(msg.method == 2):
            # origin
            if(self.is_origin == True):
                cv2.imshow("Displayer of " + self.name, current_frame)
                cv2.waitKey(1)
                #cv2.imwrite('/home/silveryu1999/result/' + str(msg.current_camera_frame_id) + '_origin' + '.jpg', current_frame)
                self.get_logger().info('Current Frame ID: %d | Result Type: %s' % (msg.current_camera_frame_id, "Origin"))
            else:
                return
                
        else:
            self.get_logger().info('Displayer get undifined result.')


def main(args=None):
    rclpy.init(args=args)

    try:
        displayer_node = Displayer_Node()
        executor = SingleThreadedExecutor()
        executor.add_node(displayer_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            displayer_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()