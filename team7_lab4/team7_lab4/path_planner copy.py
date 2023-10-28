# Note: copilot used for helping in development but main code logic is ours or reference below:
# references:

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from team7_mgs_srvs.srv import GoToPointSrv,GetTargetSrv

class PathPlanner(Node):
    def __init__(self):
        # super().__init__('path_planner')
        # self.scan_subscriber = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10
        # )
        self.goal = None
        self.waypoints = self.get_waypoints('waypoints.txt')

        
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_update,
        #     10
        # )
        # self.distance_thres = 0.02
        # self.done = True

        # creating client for go to point
        self.goTopos_client = self.create_client(GoToPointSrv, 'go_to_point')
        while not self.goTopos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('goToPos service not available, waiting again...')
        self.target_client = self.create_client(GetTargetSrv, 'get_target')
        while not self.target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_target service not available, waiting again...')

    # def scan_callback(self, msg):
    #     self.last_scan = msg
    
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
    def execute_goal(self):
        while(True):
            #rotate to Goal
            # request = GoToPointSrv.Request()
            # request.pose.x = self.goal.x
            # request.pose.y = self.goal.y
            # request.onlyAngular = True
            # future = self.goTopos_client.call_async(request)
            # rclpy.spin_until_future_complete(self, future)
            # response = future.result()
            # if(response.GoalReached):
            #     break

            
            # get_target
            request = GetTargetSrv.Request()
            request.goal = self.goal
            future = self.target_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            target = future.result().target
            # go_to_point
            request = GoToPointSrv.Request()
            request.pose.x = target.x
            request.pose.y = target.y
            request.onlyAngular = False
            future = self.goTopos_client.call_async(request) 
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if(response.GoalReached):
                break
            

        
    def execute_waypoints(self):
        for waypoint in self.waypoints:
            self.goal = waypoint
            self.execute_goal()