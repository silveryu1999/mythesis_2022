import sys
import math
from readerwriterlock import rwlock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
from bspipeline_interfaces.msg import DetectDelay
from bspipeline_interfaces.msg import TrackDelay
import rclpy
from rclpy.node import Node


class Scheduler_Node(Node):

    def __init__(self):
        # Command:
        # ros2 run basic_pipeline scheduler [client_name]
        # Example:
        # ros2 run basic_pipeline scheduler client1
        # Arguments:
        # (Arguments can be skipped if the default value would like to be used, but others must be specified in the order mentioned above.)
        # (Argument types: optional or necessary)
        # client_name: optional, value: the client name, if not set, 'anonymous_client' will be default.

        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_scheduler')

        self.camera_frame_rate = 0

        self.detect_interval_lock = rwlock.RWLockFair()
        self.detect_interval_rlock = self.detect_interval_lock.gen_rlock()
        self.detect_interval_wlock = self.detect_interval_lock.gen_wlock()
        self.detect_interval = 0
        self.detect_counter = 0

        self.track_interval_lock = rwlock.RWLockFair()
        self.track_interval_rlock = self.track_interval_lock.gen_rlock()
        self.track_interval_wlock = self.track_interval_lock.gen_wlock()
        self.track_interval = 0
        self.track_counter = 0

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group4 = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group1)
        self.detect_delay_listener = self.create_subscription(DetectDelay, self.name + '_detect_delay', self.detect_delay_callback, 10, callback_group=self.group2)
        self.track_delay_listener = self.create_subscription(TrackDelay, self.name + '_track_delay', self.track_delay_callback, 10, callback_group=self.group3)
        self.detect_publisher = self.create_publisher(Camera, self.name + '_detect_frame', 10, callback_group=self.group4)
        self.track_publisher = self.create_publisher(Camera, self.name + '_track_frame', 10, callback_group=self.group4)

        self.get_logger().info('Scheduler init done.')

    def camera_callback(self, msg):
        # get the frame rate
        self.camera_frame_rate = msg.frame_rate

        # schedule the detect frame
        if(self.detect_counter == 0):
            self.detect_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the detector.' % (msg.frame_id))
        
        with self.detect_interval_rlock:
            if(self.detect_counter >= self.detect_interval):
                self.detect_counter = 0
            else:
                self.detect_counter += 1
        
        # schedule the track frame
        self.track_publisher.publish(msg)
        self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        '''
        if(self.track_counter == 0):
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        with self.track_interval_rlock:
            if(self.track_counter >= self.track_interval):
                self.track_counter = 0
            else:
                self.track_counter += 1
        '''
        
    def detect_delay_callback(self, msg):
        # the frame rate of detect should not exceed the bottlenet/speed of detect
        curr_detect_time = max(msg.network_delay, msg.process_time)
        curr_detect_frame_rate = 1 / curr_detect_time

        # an easy policy of scheduling
        # increase, decrease or keep the interval
        # if the abs of the difference of both frame rate is less than the threshold, keep the interval
        with self.detect_interval_wlock:
            old_interval = self.detect_interval

            if(abs((curr_detect_frame_rate) * (self.detect_interval + 1) - self.camera_frame_rate) > 1):
                if((curr_detect_frame_rate) * (self.detect_interval + 1) - self.camera_frame_rate < 0):
                    # increase the interval
                    self.detect_interval += 1
                else:
                    # decrease the interval
                    self.detect_interval = max(0, self.detect_interval - 1)
            
            if(self.detect_interval > old_interval):
                self.get_logger().info('Interval of detect: Increased. Current value: %d.' % (self.detect_interval))
            elif(self.detect_interval == old_interval):
                self.get_logger().info('Interval of detect: Keeped. Current value: %d.' % (self.detect_interval))
            else:
                self.get_logger().info('Interval of detect: Decreased. Current value: %d.' % (self.detect_interval))

    def track_delay_callback(self, msg):
        # the frame rate of track should not exceed the speed of track
        track_time = msg.tracking_time
        curr_track_frame_rate = 1 / track_time

        # an easy policy of scheduling
        # increase, decrease or keep the interval
        # if the abs of the difference of both frame rate is less than the threshold, keep the interval
        with self.track_interval_wlock:
            old_interval = self.track_interval

            if(abs((curr_track_frame_rate) * (self.track_interval + 1) - self.camera_frame_rate) > 1):
                if((curr_track_frame_rate) * (self.track_interval + 1) - self.camera_frame_rate < 0):
                    # increase the interval
                    self.track_interval += 1
                else:
                    # decrease the interval
                    self.track_interval = max(0, self.track_interval - 1)

            if(self.track_interval > old_interval):
                self.get_logger().info('Interval of track: Increased. Current value: %d.' % (self.track_interval))
            elif(self.track_interval == old_interval):
                self.get_logger().info('Interval of track: Keeped. Current value: %d.' % (self.track_interval))
            else:
                self.get_logger().info('Interval of track: Decreased. Current value: %d.' % (self.track_interval))
            


def main(args=None):
    rclpy.init(args=args)

    try:
        scheduler_node = Scheduler_Node()
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(scheduler_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            scheduler_node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()