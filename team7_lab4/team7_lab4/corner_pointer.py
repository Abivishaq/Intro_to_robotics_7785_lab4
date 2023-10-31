import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
import math

from team7_mgs_srvs.srv import GetTargetSrv

#tf imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped



class GetCornerPoints(Node):
    def __init__(self):
        super().__init__('get_corner_points')
        # self.det_sub = self.create_subscription(
        #     Point,
        #     'detection_point',
        #     self.calc_det_angle,
        #     10)
        # QoS Best effor for scan subcrber
        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.point_req_sub = self.create_subscription(Point, 'request_corner_point', self.points_topic_callback, 10)
        self.corner_point_pub = self.create_publisher(Point, 'corner_point_tpc', 10)
        self.scan_sub  # prevent unused variable warning
        # self.points_pub = self.create_publisher(Pose2D,'detection_point_3d',10)

        self.last_scan = None
        self.det_angle = None

        self.angle_range_of_interest = [-90,90]
        self.offset = 0.2

        # tf variables
        self.tfBuffer = Buffer()
        self.tf_listener = TransformListener(self.tfBuffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)



         
    def scan_callback(self, msg):
        self.last_scan = msg 
        # threshold = 2
        # for i in len(self.last_scan.ranges):
        #     if self.last_scan.ranges[i] > threshold:
        #         self.last_scan.ranges[i] = np.nan
        

    def ang_to_ind(self, ang,min_ang,increment):
        return int((ang-min_ang)/increment)
    def calc_det_angle(self,msg):
        if(msg.x == -1.0):
            self.det_angle=None
        else:
            self.poi = msg.x
            self.det_angle = self.hfov*self.poi - self.hfov/2
            self.det_angle *= -1
            self.obj_ang = self.det_angle
            if(self.det_angle<0):
                self.det_angle += 2*np.pi
    
    def get_indicies_of_interest(self,min_ang,increment):
        ind1 = int((self.angle_range_of_interest[0]+360 - min_ang)/increment)
        ind2 = int((self.angle_range_of_interest[1]+360 - min_ang)/increment)
        return [ind1,ind2] 
    
    def transform_point_from_local_to_global(self,point):
        # publish target frame
        transformStamped = TransformStamped()
        transformStamped.header.stamp = self.get_clock().now().to_msg()
        transformStamped.header.frame_id = "base_link"
        transformStamped.child_frame_id = "traget_frame_tmp"
        transformStamped.transform.translation.x = point.x
        transformStamped.transform.translation.y = point.y
        transformStamped.transform.translation.z = point.z
        transformStamped.transform.rotation.x = 0.0
        transformStamped.transform.rotation.y = 0.0
        transformStamped.transform.rotation.z = 0.0
        transformStamped.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transformStamped)

        # get local goal in base_link frame
        try:
            transform = self.tfBuffer.lookup_transform('odom', 'traget_frame_tmp', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform point: ' + str(ex))
            return
        target_point = Point()
        target_point.x = transform.transform.translation.x
        target_point.y = transform.transform.translation.y
        target_point.z = transform.transform.translation.z
        return target_point
        # # get transform from odom to base_link
        # try:
        #     transform = self.tfBuffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info('Could not transform point: ' + str(ex))
        #     return
        # # transform point from base_link to odom
        # global_target = Point()
        # global_target.x = point.x + transform.transform.translation.x
        # global_target.y = point.y + transform.transform.translation.y 
        # global_target.z = point.z + transform.transform.translation.z
        # return global_target
    def transform_point_from_global_to_local(self,point):
        # publish goal frame
        transformStamped = TransformStamped()
        transformStamped.header.stamp = self.get_clock().now().to_msg()
        transformStamped.header.frame_id = "odom"
        transformStamped.child_frame_id = "goal_frame_tmp"
        transformStamped.transform.translation.x = point.x
        transformStamped.transform.translation.y = point.y
        transformStamped.transform.translation.z = point.z
        transformStamped.transform.rotation.x = 0.0
        transformStamped.transform.rotation.y = 0.0
        transformStamped.transform.rotation.z = 0.0
        transformStamped.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transformStamped)

        # get local goal in base_link frame
        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'goal_frame_tmp', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform point: ' + str(ex))
            return
        goal_local = Point()
        goal_local.x = transform.transform.translation.x
        goal_local.y = transform.transform.translation.y
        goal_local.z = transform.transform.translation.z
        return goal_local
    

        # # get transform from odom to base_link

        # try:
        #     transform = self.tfBuffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info('Could not transform point: ' + str(ex))
        #     return 
        # # transform point from base_link to odom
        # local_target = Point()
        # local_target.x = point.x - transform.transform.translation.x
        # local_target.y = point.y - transform.transform.translation.y 
        # local_target.z = point.z - transform.transform.translation.z
        # return local_target

    def points_topic_callback(self, goal_point):
        print("Got request")
        
        target_point = Point()
        # goal_point = self.transform_point_from_global_to_local(goal_point)
        goal_point_local = self.transform_point_from_global_to_local(goal_point)
        while(goal_point_local is None):
            goal_point.z = -2.0
            self.corner_point_pub.publish(goal_point)
            return
            goal_point_local = self.transform_point_from_global_to_local(goal_point)
            
        print("Goal_local:",goal_point_local)
        
        goal_local = (goal_point_local.x,goal_point_local.y)
        ranges = np.array(self.last_scan.ranges)
        dist_to_goal = math.sqrt(goal_point_local.x**2+goal_point_local.y**2)
        if(not(np.any(ranges[0:10]<dist_to_goal) or np.any(ranges[-10:]<dist_to_goal))):
            target_point = goal_point
            print("returning goal")
            self.corner_point_pub.publish(target_point)
            return
        waypoints = []
        euc_dists = []
        #threshold = 0.5
        # indicies_of_interest = self.get_indicies_of_interest(self.last_scan.angle_min,self.last_scan.angle_increment)
        # range1 = ranges[indicies_of_interest[0]:]
        # range2 = ranges[:indicies_of_interest[1]]
        # ranges = np.concatenate((range1,range2))
        # # add nan to thebeginning and end of ranges
        # ranges = np.concatenate(([np.nan],ranges,[np.nan]))

        # tmp_ang = self.det_angle - self.last_scan.angle_min 
        # print("self.last_scan.angle_min:",self.last_scan.angle_min)
        # print("self.last_scan.angle_increment:",self.last_scan.angle_increment)
        # print("self.last_scan.angle_max:",self.last_scan.angle_max)

        # angle_ind = int(tmp_ang/self.last_scan.angle_increment) 
        # if(angle_ind >= len(ranges)):
        #     angle_ind = len(ranges)-1
        # ranges = ranges[angle_ind]
        out_range_indicies = np.where(ranges>dist_to_goal)
        ranges[out_range_indicies] = np.nan
        print("checking ranges")
        for i, range in enumerate(ranges[:-1]):
            if math.isnan(ranges[i-1]) and not math.isnan(range):
                print(range)
                x = range * math.cos(self.last_scan.angle_min + i * self.last_scan.angle_increment) - self.offset
                y = range * math.sin(self.last_scan.angle_min + i * self.last_scan.angle_increment) - self.offset
                waypoints.append((x, y))
                euc_dists.append(math.dist(goal_local, (x, y)))
            
            elif math.isnan(ranges[i+1]) and not math.isnan(range):
                x = range * math.cos(self.last_scan.angle_min + i * self.last_scan.angle_increment) - self.offset
                y = range * math.sin(self.last_scan.angle_min + i * self.last_scan.angle_increment) + self.offset
                waypoints.append((x, y))
                euc_dists.append(math.dist(goal_local, (x, y)))
                # break
        if(len(euc_dists)>0):
            min_idx = np.argmin(np.array(euc_dists))
            target_point.x, target_point.y = waypoints[min_idx]
        else:
            print("no point of interest found")

        # print("Sending target_point")
        target_point = self.transform_point_from_local_to_global(target_point)
        # print("target_point:",target_point)
        while(target_point is None):
            goal_point.z = -2.0
            self.corner_point_pub.publish(goal_point)
            return
            target_point = self.transform_point_from_local_to_global(target_point)
 
        self.corner_point_pub.publish(target_point)
        return        
      

def main(args=None):
    rclpy.init(args=args)

    get_corner_points = GetCornerPoints()

    rclpy.spin(get_corner_points)

    get_corner_points.destroy_node()
    rclpy.shutdown()