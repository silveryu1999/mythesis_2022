import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bspipeline_interfaces.msg import Camera
import rclpy
from rclpy.node import Node


class Scheduler_Node(Node):

    def __init__(self):
        if(len(sys.argv) == 2):
            self.name = sys.argv[1]
        else:
            self.name = 'anonymous_client'
        super().__init__(self.name + '_scheduler')

        self.interval = 1
        self.counter = 0

        self.group = ReentrantCallbackGroup()
        self.camera_listener = self.create_subscription(Camera, self.name + '_camera_frame', self.camera_callback, 10, callback_group=self.group)
        self.detect_publisher = self.create_publisher(Camera, self.name + '_detect_frame', 10, callback_group=self.group)
        self.track_publisher = self.create_publisher(Camera, self.name + '_track_frame', 10, callback_group=self.group)

        self.get_logger().info('Scheduler init done.')

    def camera_callback(self, msg):
        self.get_logger().info('-----------------------------------------')

        if(self.counter == 0):
            self.detect_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the detector.' % (msg.frame_id))
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))
        else:
            self.track_publisher.publish(msg)
            self.get_logger().info('Frame %d has been submitted to the tracker.' % (msg.frame_id))

        if(self.counter >= self.interval):
            self.counter = 0
        else:
            self.counter += 1
        

def main(args=None):
    rclpy.init(args=args)

    try:
        scheduler_node = Scheduler_Node()
        executor = MultiThreadedExecutor(num_threads=4)
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