import rclpy
from rclpy.node import Node
# from std_msgs.msg import YourMessageType  # Replace with your message type
from sensor_msgs.msg import LaserScan
class EchoSpecificFieldNode(Node):
    def __init__(self):
        super().__init__('echo_specific_field_node')
        self.subscription = self.create_subscription(
            LaserScan,   # Replace with your message type
            '/scan',     # Replace with your topic name
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print(msg.angle_min)   # Replace with your desired field name
        print(len(msg.ranges))
        print(type(msg.ranges))

def main(args=None):
    rclpy.init(args=args)
    node = EchoSpecificFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()