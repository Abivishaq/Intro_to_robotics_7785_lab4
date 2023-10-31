'''
Author1: Vriksha Srihari
Author2: Abivishaq Balasubramanian
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point,Pose2D
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class ChaseObjectNode(Node):
    def __init__(self):
        super().__init__('chase_object')
        self.qos_profile = qos_profile = QoSProfile(
           depth=10,  # Queue size
           reliability=QoSReliabilityPolicy.RELIABLE,  # Set reliability to RELIABLE
           history=QoSHistoryPolicy.KEEP_LAST  # Keep only the last 'depth' number of messages
           )
        self.get_object_range_subscriber = self.create_subscription(
            Pose2D,
            '/local_goal',
            self.object_range_callback,
            self.qos_profile
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.linear_vel_max= 0.2
        self.angular_vel_max= 1.5
        # Initialize PID control parameters
        self.kp_linear = 0.6
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.kp_angular = 5
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear_error = 0.0
        self.integral_angular_error = 0.0

        # Distance and angle
        self.desired_distance = 0.5  
        self.desired_angle = 0.0
        
    def limit_linear(self, linear_cmd):
        linear_cmd = max(-self.linear_vel_max, min(linear_cmd, self.linear_vel_max))
        return linear_cmd
    def limit_angular(self, angular_cmd):
        angular_cmd = max(-self.angular_vel_max, min(angular_cmd, self.angular_vel_max))
        return angular_cmd
    def object_range_callback(self, msg):
        # print("In object range callback")
        if((msg.y == -1.0 or abs(msg.x) < 0.01) and not(msg.y>1.0)):
            twist_msg = Twist()
            twist_msg.linear.x = 0.0 #linear_cmd
            twist_msg.angular.z = 0.0 # angular_cmd
            self.velocity_publisher.publish(twist_msg)
        else:
            print("Velocity command published")
            object_range = msg.x

            # Compute error terms
            linear_error =  object_range
            angular_error = msg.theta  

            # Compute PID control commands
            linear_cmd = self.kp_linear * linear_error + \
                self.ki_linear * self.integral_linear_error + \
                self.kd_linear * (linear_error - self.prev_linear_error)

            angular_cmd = self.kp_angular * angular_error + \
                self.ki_angular * self.integral_angular_error + \
                self.kd_angular * (angular_error - self.prev_angular_error)

            # Store current errors for the next iteration
            self.prev_linear_error = linear_error
            self.prev_angular_error = angular_error

            # Publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = self.limit_linear(linear_cmd)
            twist_msg.angular.z = self.limit_angular(angular_cmd)
            self.velocity_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    chase_object_node = ChaseObjectNode()
    rclpy.spin(chase_object_node)
    chase_object_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
