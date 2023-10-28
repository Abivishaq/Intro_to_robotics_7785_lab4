# boilerplate generate by copilot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

class PathTestBed(Node):

    def __init__(self):
        super().__init__('path_test_bed')
        self.publisher_ = self.create_publisher(PoseArray, 'path', 10)
        # self.timer_ = self.create_timer(1.0, self.publish_pose_array)
        self.waypoints = self.get_waypoints('waypoints.txt')
    def get_waypoints(filename):
        f = open(filename,'r')
        data = f.read()
        f.close()
        data.split()
        print(data)
        waypoints = []
        lines = data.split('\n')
        for line in lines[:-1]:
            elms = line.split()
            waypoints.append([float(elms[0]),float(elms[1])])
        return(waypoints)
    def publish_pose_array(self):
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'odom'
        pose_array.poses = []
        
        # Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        # Pose(position=Point(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        # Pose(position=Point(x=1.0, y=1.0, z=0.0), orientation=Quaternion(w=1.0)),
        # Pose(position=Point(x=0.0, y=1.0, z=0.0), orientation=Quaternion(w=1.0)),
        
        self.publisher_.publish(pose_array)
        self.get_logger().info('Published PoseArray message')

def main(args=None):
    rclpy.init(args=args)
    node = PathTestBed()
    node.publish_pose_array()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
