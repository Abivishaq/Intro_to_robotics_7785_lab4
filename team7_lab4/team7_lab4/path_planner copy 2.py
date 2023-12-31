# Note: copilot used for helping in development but main code logic is ours or reference below:
# chatGPT used for debugging
# references:

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from team7_mgs_srvs.srv import GoToPointSrv,GetTargetSrv
import math


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        # self.scan_subscriber = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10
        # )
        self.goal = None
        self.waypoints = self.get_waypoints('/home/abivishaq/turtle_ws/src/team7_lab4/team7_lab4/waypoints.txt')

        print(self.waypoints)
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_update,
        #     10
        # )
        # self.distance_thres = 0.02
        # self.done = True

        # creating client for go to point
        # self.goTopos_client = self.create_client(GoToPointSrv, 'go_to_point')
        # while not self.goTopos_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('goToPos service not available, waiting again...')
        self.target_client = self.create_client(GetTargetSrv, 'get_corner_target')
        # while not self.target_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('get_target service not available, waiting again...')
    
        self.target_pub = self.create_publisher(Point, 'target_pos', 10)
        self.reached_trg_sub = self.create_subscription(
            Point,
            'reached_target',
            self.reached_trg_callback,
            10
        )
        self.target_request_pub = self.create_publisher(Point, 'request_corner_point', 10)
        self.rec_target_sub = self.create_subscription(Point, 'corner_point_tpc', self.rec_target_callback, 10)
        self.waiting_for_reached_trg = False
        self.waypoint_reached = True
        self.curr_waypoint_index = -1
        self.last_reached_trg = None
        self.target_reached = True
        self.dist_threshold = 0.05
        self.execute_timer = self.create_timer(1/30, self.execute_waypoints)
        self.IDLE_STATE = 0
        self.ROTATE_STATE = 1
        self.GO_TO_POINT_STATE = 2
        self.GET_TARGET_STATE = 3
        self.state_of_planner = self.IDLE_STATE
        

        
    # def scan_callback(self, msg):
    #     self.last_scan = msg
    def reached_trg_callback(self,msg):
        self.waiting_for_reached_trg = False
        self.last_reached_trg = msg

    def rec_target_callback(self,msg):
        self.latest_target = msg
        self.target_reached = True
    
    def get_waypoints(self,filename):
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
    def check_goal_reached(self):
        pt1 = self.last_reached_trg
        pt2 = self.goal
        dist = math.sqrt((pt1.x-pt2.x)**2+(pt1.y-pt2.y)**2)
        if(dist<self.dist_threshold):
            return True
        else:
            return False
    def execute_goal(self):
        print(self.state_of_planner)
        print(self.waiting_for_service_return)
        if(self.waypoint_reached==False and self.waiting_for_service_return == False):
            if(self.target_reached):
                if(self.state_of_planner == self.IDLE_STATE):
                    self.state_of_planner = self.ROTATE_STATE
                    self.goal.z = -1.0
                    self.target_pub.publish(self.goal)
                    self.target_reached = False
                if(self.state_of_planner == self.ROTATE_STATE):
                    # self.waiting_for_service_return = True
                    # print("waiting for reached trg")
                    # # get_target
                    # request = GetTargetSrv.Request()
                    # request.goal = self.goal
                    # future = self.target_client.call_async(request)
                    # print("")
                    # rclpy.spin_until_future_complete(self, future)
                    # self.waiting_for_service_return = False
                    # print("checking goal:"+str(self.goal))
                    # target = future.result().target
                    # print("target:"+str(target))
                    # # go_to_point
                    # target.z = 0.0
                    # self.target_pub.publish(target)
                    # self.state_of_planner = self.GO_TO_POINT_STATE
                    # self.target_reached = False
                    
                    # Send request
                    self.target_request_pub.publish(self.goal)
                    self.target_reached = False
                    
                    # Transition to GET_TARGET_STATE
                    self.state_of_planner = self.GET_TARGET_STATE
                
                if(self.state_of_planner == self.GET_TARGET_STATE): 
                    self.target_reached = False
                    self.state_of_planner = self.GO_TO_POINT_STATE
                    self.latest_target.z = 0.0
                    self.target_pub.publish(self.latest_target)
                

                if(self.state_of_planner == self.GO_TO_POINT_STATE):
                    # check reached condition
                    if(self.check_goal_reached()):
                        self.waypoint_reached = True
                        self.state_of_planner = self.IDLE_STATE
                    else:
                        self.state_of_planner = self.IDLE_STATE
        
                
        # if(self.waypoint_reached == False and self.target_reached == True):
        #     self.target_reached = False
        #     #rotate to Goal
        #     self.goal.z = -1.0
        #     self.target_pub.publish(self.goal)
        #     # get_target
        #     request = GetTargetSrv.Request()
        #     request.goal = self.goal
        #     future = self.target_client.call_async(request)
        #     rclpy.spin_until_future_complete(self, future)
        #     target = future.result().target
        #     # go_to_point
        #     target.z = 0.0
        #     self.target_pub.publish(target)
        #     # check reached condition
        # if(self.waypoint_reached == False and self.target_reached == False):
        #     if(self.check_goal_reached()):
        #         self.waypoint_reached = True


        
    def execute_waypoints(self):
        if(self.curr_waypoint_index >= len(self.waypoints)):
            print("All waypoints done")
            return
        if(self.waypoint_reached):
            self.waypoint_reached = False
            self.curr_waypoint_index += 1
            print(self.curr_waypoint_index )
            self.goal = Point()
            curr_wp = self.waypoints[self.curr_waypoint_index]
            self.goal.x = curr_wp[0]
            self.goal.y = curr_wp[1]
        else:
            self.execute_goal()

def main():
    rclpy.init()

    path_planner = PathPlanner()

    rclpy.spin(path_planner)