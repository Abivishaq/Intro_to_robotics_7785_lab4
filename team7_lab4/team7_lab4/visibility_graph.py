# Note: Code written with the help of github copilot but main logic is ours or reference below:

# Code Logic references
# 1. https://www1.cs.columbia.edu/~allen/F19/NOTES/lozanogrown.pdf
import numpy as np
class line_toolbox:
    def __init__(self) -> None:
        # self.points =[(0,0),goal]
        # self.lines = []
        pass
    def find_line(self,pt1,pt2)->tuple:
        slope = (pt2[1]-pt1[1])/(pt2[0]-pt1[0])
        c = pt1[1] - slope*pt1[0]
        return (slope,c)
    def get_x_intersection_point(self,line1,line2)->float:
        # line1 = (slope1,c1), line2 = (slope2,c2)
        x = (line2[1]-line1[1])/(line1[0]-line2[0])
        return(x)
    def check_line_segmnt_intersection(self,line1_pts,line2_pts)->bool: 
        # line1_pts - (pt11,pt12), line2_pts - (pt21,pt22)
        line1 = self.find_line(line1_pts[0],line1_pts[1])
        line2 = self.find_line(line2_pts[0],line2_pts[1])
        x = self.get_x_intersection_point(line1,line2)
        # checking x1 lies in both the line segments

        x_in_line1_seg = x>=min(line1_pts[0][0],line1_pts[1][0]) and x<=max(line1_pts[0][0],line1_pts[1][0])  
        x_in_line2_seg = x>=min(line2_pts[0][0],line2_pts[1][0]) and x<=max(line2_pts[0][0],line2_pts[1][0])
        if  x_in_line1_seg and x_in_line2_seg:
            return True
        else:
            return False

class visibilty_graphs:
    def __init__(self,goal)->None:
        self.obstacle_lines = []
        # self.obstacles = []
        self.start = (0,0)
        self.goal = goal
        self.points = [self.start,self.goal]
        self.line_toolbox = line_toolbox()
        self.graph = {}
    def add_obstacle_line(self,line_pts)->None:
        self.obstacle_lines.append(line_pts)
        if(line_pts[0] not in self.points):
            self.points.append(line_pts[0])
        if(line_pts[1] not in self.points):
            self.points.append(line_pts[1])
        
    def check_line(self,line)->bool:
        for obstacle_line in self.obstacle_lines:
            if self.line_toolbox.check_line_segmnt_intersection(line,obstacle_line):
                return False
        return True
    def get_sq_distance_between_points(self,pt1,pt2)->float:
        return ((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
    def generate_graph(self):
        for pt in self.points:
            self.graph[pt] = np.array([])
        for pt1 in self.points:
            for pt2 in self.points:
                if pt1!=pt2:
                    line = (pt1,pt2)
                    if(self.check_line(line)):
                        dist = self.get_sq_distance_between_points(pt1,pt2)
                        self.graph[pt1].append([pt2,dist])
    def calculate_path(self):
        # A* algorithm
        pass
    
    def update(self,obstacles):
        for obstacle_line in obstacles:
            self.add_obstacle_line(obstacle_line)
        self.generate_graph()
        self.calculate_path()
    
    
class a_star:
    def __init__(self) -> None:
        pass
    def calculate_path(self,graph,start,goal):
        # graph is a dictionary with key as point and value as list of points
        frontier = [start]
        