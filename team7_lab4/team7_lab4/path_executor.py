# Note: Code generated with the help of copilot. But main logic is ours or other references.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,PoseArray
from nav_msgs.msg import Odometry


class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')
        self.get_object_range_subscriber = self.create_subscription(
            PoseArray,
            '/path',
            self.object_range_callback,
            10
        )
        self.path = []
        self.path_index = 0
        self.velocity_publisher = self.create_publisher(
            Point,
            'target_pos',
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_update,
            10
        )
        self.distance_thres = 0.02
        self.done = True
    def object_range_callback(self, msg):
        print("Got path")
        self.path = msg.poses
        self.path_index = 0
        self.done = False
        self.publish_point()

    def publish_point(self):
        if(self.path_index < len(self.path)):
            self.velocity_publisher.publish(self.path[self.path_index].position)
            self.get_logger().info("Published point:"+str(self.path[self.path_index].position))
            # self.path_index += 1
        else:
            self.get_logger().info("Finished path execution")
    def get_distance(self, p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5
    def odom_update(self,msg):

        if(self.path_index < len(self.path) and not self.done):
            if(self.get_distance(msg.pose.pose.position,self.path[self.path_index].position) < self.distance_thres):
                self.path_index += 1
                if(self.path_index >= len(self.path)):
                    self.done = True
                    self.get_logger().info("Finished path execution")
                else:
                    self.publish_point()

def main():
    rclpy.init()
    node = PathExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

