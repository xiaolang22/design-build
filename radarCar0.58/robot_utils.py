import numpy as np
import matplotlib.pyplot as plt

class Robot:
    """
    一个简单的机器人模型，存储其位姿
    """
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta # 弧度

def _get_line_segment_intersection(p1, p2, p3, p4):
    """
    计算两条线段 p1-p2 和 p3-p4 的交点
    返回交点坐标 (x, y) 或 None
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return None # 平行或共线

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den

    if 0 < t and 0 <= u <= 1:
        # 我们只关心射线方向上的交点 (t>0) 且在墙壁线段上 (0<=u<=1)
        intersect_x = x1 + t * (x2 - x1)
        intersect_y = y1 + t * (y2 - y1)
        return (intersect_x, intersect_y)
    
    return None

def simulate_lidar_scan(robot, edges, num_beams=360, max_range=50.0):
    """
    模拟激光雷达扫描 (光线投射法)
    robot: Robot对象
    edges: 迷宫边缘线段列表
    num_beams: 激光束的数量
    max_range: 激光雷达的最大探测距离
    返回: 一个包含 (角度, 距离) 的元组列表
    """
    scan_data = []
    
    for i in range(num_beams):
        # 计算当前光束的角度 (相对于世界坐标系)
        angle = robot.theta + np.deg2rad(i * 360 / num_beams)
        
        # 创建一条长为 max_range 的射线
        ray_end_x = robot.x + max_range * np.cos(angle)
        ray_end_y = robot.y + max_range * np.sin(angle)
        
        closest_dist = max_range
        
        # 遍历所有墙壁，寻找最近的交点
        for edge in edges:
            p1 = (robot.x, robot.y)
            p2 = (ray_end_x, ray_end_y)
            p3 = (edge[0], edge[1])
            p4 = (edge[2], edge[3])
            
            intersection = _get_line_segment_intersection(p1, p2, p3, p4)
            
            if intersection:
                dist = np.sqrt((intersection[0] - robot.x)**2 + (intersection[1] - robot.y)**2)
                if dist < closest_dist:
                    closest_dist = dist
        
        scan_data.append((angle, closest_dist))
        
    return scan_data