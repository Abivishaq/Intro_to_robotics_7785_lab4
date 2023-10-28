# Note: Code written with the help of github copilot but main logic is ours or reference below:
# references used:
# 1. For tf lookup: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
# 2. ROS2 tutoials for publisher, subscriber and other basics
# 3. tf broadcaster: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
# 4. quad to euler: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, Pose2D

from nav_msgs.msg import Odometry


# imports for TF
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import numpy as np
import math

from team7_mgs_srvs.srv import GoToPointSrv
class goToPose(Node):
    def __init__(self):
        super().__init__('go_to_pos')
        self.pos_odom = None
        self.vel_odom = None
        self.goal_point = None # Global frame
        self.vel_trg = Twist()
        self.pos_err_buffer = 0.1
        self.ang_err_buffer = 0.05
        self.reached_target_publisher = self.create_publisher(Point, 'reached_target', 10)

        self.target_sub = self.create_subscription(
            Point,
            'target_pos',
            self.go_to_point_update,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_update,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.target_sub  # prevent unused variable warning
        self.odom_sub  # prevent unused variable warning
        self.scan_sub  # prevent unused variable warning
        self.non_idle_active = False


        # self.goToPoint_service = self.create_service(GoToPointSrv, 'go_to_point', self.go_to_point_callback)
        # self.get_logger().info('go_to_point service created')

        # self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Pose2D, '/local_goal', 10)
        
        self.target_sub  # prevent unused variable warning
        self.odom_sub  # prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.controller_timer = self.create_timer(1/30.0, self.controller)
        self.last_ang_error = None
        self.stop_move = False
        self.state = "idle"
        self.check_rate = self.create_rate(10)
    
    # def go_to_point_callback(self, request, response):
    #     self.goal_point = request.goal_point
    #     self.publish_goal_frame(self.goal_point)
    #     response.result = False
    #     response.goal_reached = False
    #     self.response = response
    #     if(request.only_angular):
    #         self.state = "rotate"
    #     else:
    #         self.state = "move"
    #     # self.publish_goal_frame(self.goal_point)
    #     while(self.state != "idle"):
    #         # self.controller()
    #         self.check_rate.sleep()
            
        
    #     self.error_pub.publish(Pose2D(x=0.0,theta=0.0))
    #     response = self.response
    #     print("returning response:"+str(response))
    #     return response
    def scan_callback(self, msg):
        self.last_scan = msg
        if(self.last_ang_error is not None):
            ang = self.last_ang_error
            if(ang<0):
                ang = 2*math.pi + ang
            ang_index = int((ang-msg.angle_min)/msg.angle_increment)
            dist_to_goal = msg.ranges[ang_index]
            if(dist_to_goal < 0.2):
                self.state = "idle"

    def go_to_point_update(self, msg):
        if(msg.z==-1.0):
            self.state = "rotate"
            self.non_idle_active = True
        else:
            self.state = "move"
            self.non_idle_active = True
        self.get_logger().info('I heard: ' + str(msg.x))
        self.goal_point = msg
        print("Latest pose:"+str(self.pos_odom))
        self.publish_goal_frame(self.goal_point)
    
    def odom_update(self,msg):
        # self.get_logger().info("Got odom!!!!!!!!!!!!!!!\n"*10)
        self.pos_odom = msg.pose.pose
        self.vel_odom = msg.twist.twist
    def get_error_to_goal(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint',
                'goal_frame',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform odom to base_footprint: {ex}')
            return(0.0,0.0)
        trans = t.transform.translation
        pos= np.array([trans.x,trans.y])
        print(pos)
        pos_error = np.sum(pos**2)**0.5
        ang_error = np.arctan2(pos[1],pos[0]) 
        self.get_logger().info("pos_error:"+str(pos_error))
        self.get_logger().info("ang_error:"+str(ang_error))
        return pos_error,ang_error
    
    def publish_goal_frame(self,goal_point):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'goal_frame'

        t.transform.translation.x = float(goal_point.x)
        t.transform.translation.y = float(goal_point.y)
        t.transform.translation.z = 0.0#float(goal_point.z)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)
    def get_distance_to_goal(self):
        goal = self.goal_point
        pos = self.pos_odom.position
        pos = np.array([pos.x,pos.y])
        goal = np.array([goal.x,goal.y])
        dist = np.sum((pos-goal)**2)**0.5
        return dist
    def controller(self):
        print("state:"+self.state)
        if(self.goal_point is None or self.state == "idle"):
            if(self.non_idle_active):
                self.non_idle_active = False
                self.reached_target_publisher.publish(Point(x=self.pos_odom.position.x,y=self.pos_odom.position.y,z=0.0))
                print("idling: sending 0 vel")
                self.error_pub.publish(Pose2D(x=0.0,theta=0.0))
        elif(self.state == "move"):
            # self.get_logger().info("Will publish vel to go to goal")
            pos_err, ang_error = self.get_error_to_goal()
            print("Got error:"+str(pos_err)+","+str(ang_error))
            self.last_ang_error = ang_error
            self.error_pub.publish(Pose2D(x=pos_err,theta=ang_error))
            # print("Published error:")
            # print('pos_err:'+str(pos_err))
            # print('self.pos_err_buffer:'+str(self.pos_err_buffer))
            if(pos_err < self.pos_err_buffer):
                # print("Entered if")
                self.state = "idle"
                # self.response.result = True
                # dist_to_goal = self.get_distance_to_goal()
                # print("Got here")
                # if(dist_to_goal < self.pos_err_buffer):
                #     self.response.goal_reached = True
                # else:
                #     self.response.goal_reached = False
            else:
                pass
                # print("Entered else")
        elif(self.state == "rotate"):
            pos_err, ang_error = self.get_error_to_goal()
            self.last_ang_error = ang_error 
            if(abs(ang_error) < self.ang_err_buffer):
                # print()
                self.state = "idle"
                # self.response.result = True
                # self.response.goal_reached = False
            self.error_pub.publish(Pose2D(x=0.0,y=2.0,theta=self.last_ang_error))  

def main(args=None):
    rclpy.init(args=args)

    go_to_pose_node = goToPose()

    rclpy.spin(go_to_pose_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    go_to_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()