#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
用于测试DWA算法跟踪A*规划的路径
"""

import os
import sys
import argparse
import pickle
import numpy as np
import heapq
from collections import deque
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# 实现简单的KD树替代scipy的cKDTree
class SimpleKDTree:
    def __init__(self, points):
        """简单的KD树实现，用于替代scipy的cKDTree
        
        Args:
            points: 点云数据，形状为(n, 2)的numpy数组
        """
        self.points = np.array(points)
        
    def query(self, query_points):
        """查询最近点
        
        Args:
            query_points: 查询点，形状为(m, 2)的numpy数组
            
        Returns:
            dists: 到最近点的距离，形状为(m,)的numpy数组
            indices: 最近点的索引，形状为(m,)的numpy数组
        """
        query_points = np.array(query_points)
        n_points = len(query_points)
        
        # 初始化距离和索引数组
        dists = np.zeros(n_points)
        indices = np.zeros(n_points, dtype=int)
        
        # 对每个查询点，计算到所有点的距离
        for i in range(n_points):
            # 计算点i到所有点的距离
            diff = self.points - query_points[i]
            sq_dists = np.sum(diff**2, axis=1)
            
            # 找到最小距离及其索引
            min_idx = np.argmin(sq_dists)
            dists[i] = np.sqrt(sq_dists[min_idx])
            indices[i] = min_idx
        
        return dists, indices

class Config:
    def __init__(self):
        # 机器人运动参数 - 调整速度参数
        self.max_speed = 6.0  # 增加最大速度 [m/s]
        self.min_speed = 0.2  # 增加最小速度 [m/s]
        self.max_yawrate = 80.0 * math.pi / 180.0  # 增加最大角速度
        self.max_accel = 1.0  # 增加加速度 [m/ss]
        self.max_dyawrate = 80.0 * math.pi / 180.0  # 增加角加速度

        # DWA参数调整
        self.v_reso = 0.1  # 减小速度分辨率以获得更精细的控制 [m/s]
        self.yawrate_reso = 0.2 * math.pi / 180.0  # 减小角速度分辨率
        self.dt = 0.1  # 采样时间 [s]
        self.predict_time = 1.5  # 增加预测时间以更好地规划转弯

        # 代价函数权重
        self.to_goal_cost_gain = 0.15  # 降低到目标的代价权重，避免过度追求目标
        self.speed_cost_gain = 0.1     # 降低速度代价权重
        self.obstacle_cost_gain = 0.65  # 提高避障权重

        self.robot_radius = 0.2  # [m]

        # A*和地图参数
        self.grid_res = 0.15  # 栅格分辨率
        self.obstacle_radius = 0.3  # 障碍物膨胀半径
        self.a_star_step = 2  # A*步长

class GlobalLocalPathPlanner:
    def __init__(self, point_cloud, start, exits):
        self.config = Config()
        self.point_cloud = point_cloud
        self.start = start
        self.exits = exits
        self.initialize_map()
        
    def initialize_map(self):
        # 计算地图边界
        cloud_x, cloud_y = zip(*self.point_cloud)
        all_x = list(cloud_x) + [self.start[0]] + [p[0] for p in self.exits]
        all_y = list(cloud_y) + [self.start[1]] + [p[1] for p in self.exits]
        
        margin = 0.5
        self.map_min_x = min(all_x) - margin
        self.map_max_x = max(all_x) + margin
        self.map_min_y = min(all_y) - margin
        self.map_max_y = max(all_y) + margin

        # 初始化栅格地图
        self.width = int(np.ceil((self.map_max_x - self.map_min_x) / self.config.grid_res))
        self.height = int(np.ceil((self.map_max_y - self.map_min_y) / self.config.grid_res))
        self.grid_map = np.zeros((self.height, self.width), dtype=np.uint8)

        # 障碍物膨胀
        for x, y in self.point_cloud:
            cx = int((x - self.map_min_x) / self.config.grid_res)
            cy = int((y - self.map_min_y) / self.config.grid_res)
            r = int(self.config.obstacle_radius / self.config.grid_res)
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if 0 <= cx+dx < self.width and 0 <= cy+dy < self.height:
                        if (dx**2 + dy**2) * (self.config.grid_res**2) <= self.config.obstacle_radius**2:
                            self.grid_map[cy+dy, cx+dx] = 1
        
        # 边界障碍
        self.grid_map[0, :] = 1
        self.grid_map[-1, :] = 1
        self.grid_map[:, 0] = 1
        self.grid_map[:, -1] = 1

    def coord2grid(self, x, y):
        return (int((x - self.map_min_x) / self.config.grid_res), 
                int((y - self.map_min_y) / self.config.grid_res))

    def grid2coord(self, gx, gy):
        return (gx * self.config.grid_res + self.map_min_x, 
                gy * self.config.grid_res + self.map_min_y)

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def astar_global_path(self):
        start_g = self.coord2grid(*self.start)
        goal_g = self.coord2grid(*self.exits[0]) if self.exits else (self.width-1, self.height-1)

        open_set = []
        heapq.heappush(open_set, (0+self.heuristic(start_g, goal_g), 0, start_g, None))
        came_from = {}
        cost_so_far = {start_g: 0}

        while open_set:
            _, cost, current, parent = heapq.heappop(open_set)
            
            if current == goal_g:
                path = [current]
                while parent:
                    path.append(parent)
                    parent = came_from.get(parent, (None,))[1]
                return [self.grid2coord(x, y) for x, y in path[::-1]]

            if current in came_from:
                continue
            came_from[current] = (cost, parent)

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = current[0]+dx, current[1]+dy
                if 0 <= nx < self.width and 0 <= ny < self.height and self.grid_map[ny, nx] == 0:
                    new_cost = cost + ((dx**2 + dy**2)**0.5)
                    if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                        cost_so_far[(nx, ny)] = new_cost
                        heapq.heappush(open_set, (new_cost+self.heuristic((nx,ny), goal_g), new_cost, (nx,ny), current))
        return None

    def motion(self, x, u, dt):
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_dynamic_window(self, x):
        vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yawrate, self.config.max_yawrate]
        vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_dyawrate * self.config.dt,
              x[4] + self.config.max_dyawrate * self.config.dt]
        vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        return vr

    def calc_trajectory(self, x_init, v, w):
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, w], self.config.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory

    def calc_to_goal_cost(self, trajectory, goal):
        # 计算终点位置与目标的距离
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        goal_dis = math.sqrt(dx ** 2 + dy ** 2)
        
        # 计算方向差异（与path_DWA.py一致）
        goal_angle = math.atan2(dy, dx)
        trajectory_angle = trajectory[-1, 2]
        angle_diff = abs(goal_angle - trajectory_angle) % (2 * math.pi)
        angle_cost = min(angle_diff, 2 * math.pi - angle_diff)
        
        # 综合距离和方向的代价
        return self.config.to_goal_cost_gain * (goal_dis + 0.5 * angle_cost)

    def calc_obstacle_cost(self, traj, ob):
        # 使用自定义的SimpleKDTree替代scipy的cKDTree
        kdtree = SimpleKDTree(ob)
        dists, _ = kdtree.query(traj[:, :2])
        min_dist = np.min(dists)
        if min_dist <= self.config.robot_radius:
            return float("inf")
        # 指数衰减的代价计算
        obstacle_cost = math.exp(-min_dist) * self.config.obstacle_cost_gain
        return obstacle_cost

    def dwa_local_control(self, x, u, goal, ob):
        vr = self.calc_dynamic_window(x)
        min_cost = float('inf')
        best_u = u
        best_trajectory = None

        # 增加采样点数和采样密度
        for v in np.linspace(vr[0], vr[1], 10):  # 增加线速度采样点
            for w in np.linspace(vr[2], vr[3], 12):  # 增加角速度采样点，提高转弯精度
                trajectory = self.calc_trajectory(x, v, w)
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.calc_obstacle_cost(trajectory, ob)
                final_cost = (
                    to_goal_cost +  # to_goal_cost already includes the gain
                    speed_cost +
                    ob_cost
                )
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_trajectory = trajectory

        return best_u, best_trajectory

    def interpolate_waypoints(self, path, max_distance=3.0):
        """
        根据最大距离插值路径点，使相邻点距离不超过max_distance
        """
        if not path or len(path) < 2:
            return path

        interpolated_path = [path[0]]

        for i in range(1, len(path)):
            prev = np.array(interpolated_path[-1])
            curr = np.array(path[i])
            dist = np.linalg.norm(curr - prev)
            if dist > max_distance:
                num_points = int(np.ceil(dist / max_distance))
                for j in range(1, num_points):
                    ratio = j / num_points
                    point = prev + ratio * (curr - prev)
                    interpolated_path.append(point.tolist())
            interpolated_path.append(curr.tolist())

        return interpolated_path

    def plan(self, animation=True, animation_speed=0.01):
        # 全局路径规划
        global_path = self.astar_global_path()
        if not global_path:
            print("No global path found!")
            return None

        # 对全局路径进行插值，使相邻点距离不超过max_distance
        global_path = self.interpolate_waypoints(global_path, max_distance=1.5)  # 减小间距以提高跟踪精度

        # 计算初始朝向
        if len(global_path) > 1:
            dx = global_path[1][0] - global_path[0][0]
            dy = global_path[1][1] - global_path[0][1]
            init_yaw = math.atan2(dy, dx)
        else:
            init_yaw = math.pi/2

        # 初始化机器人状态
        x = np.array([self.start[0], self.start[1], init_yaw, 0.0, 0.0])  # 初始速度为0
        u = np.array([0.0, 0.0])

        # 定义障碍物数组
        obstacles = np.array(self.point_cloud)

        # 存储轨迹
        trajectory = [x[:3]]

        # 如果不需要动画，直接返回全局路径
        if not animation:
            return global_path

        # ================= matplotlib 可视化 =================
        plt.figure(figsize=(10, 8))
        ax = plt.gca()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title("Global + Local Path Planning")
        
        # 设置坐标轴范围
        margin = 1.5
        all_x = [p[0] for p in self.point_cloud] + [self.start[0]] + [p[0] for p in self.exits]
        all_y = [p[1] for p in self.point_cloud] + [self.start[1]] + [p[1] for p in self.exits]
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        # 绘制点云
        ax.scatter([p[0] for p in self.point_cloud], [p[1] for p in self.point_cloud], s=5, c='gray', alpha=0.5, label='Point Cloud')
        
        # 绘制全局路径
        ax.plot([p[0] for p in global_path], [p[1] for p in global_path], 'r--', linewidth=2, label='Global Path')
        
        # 绘制起点和终点
        ax.plot(self.start[0], self.start[1], 'go', markersize=10, label='Start')
        for exit_point in self.exits:
            ax.plot(exit_point[0], exit_point[1], 'r*', markersize=15, label='Exit')
        
        # 创建机器人圆和轨迹线
        robot_circle = plt.Circle((x[0], x[1]), self.config.robot_radius, color='orange', alpha=0.7)
        ax.add_patch(robot_circle)
        
        trajectory_line, = ax.plot([], [], 'b-', linewidth=2, label='Trajectory')
        
        # 设置图例
        ax.legend()
        
        # 存储轨迹数据
        trajectory_x = [x[0]]
        trajectory_y = [x[1]]

        # 局部路径规划和避障
        for goal in global_path[1:]:
            goal_reached = False
            stuck_count = 0
            while not goal_reached:
                dist_to_goal = np.linalg.norm(x[:2] - goal)
                if dist_to_goal <= self.config.robot_radius * 2.0:  # Increased threshold
                    goal_reached = True
                    break

                u, trajectory_segment = self.dwa_local_control(x, u, goal, obstacles)
                x = self.motion(x, u, self.config.dt)
                trajectory.append(x[:3])

                # 更新轨迹数据
                trajectory_x.append(x[0])
                trajectory_y.append(x[1])
                
                # 每5步刷新一次动画
                if stuck_count % 5 == 0:
                    # 更新轨迹线
                    trajectory_line.set_data(trajectory_x, trajectory_y)
                    
                    # 更新机器人位置
                    robot_circle.center = (x[0], x[1])
                    
                    # 刷新图形
                    plt.pause(animation_speed)

                stuck_count += 1
                if stuck_count > 500:
                    print(f"Stuck at {x[:2]}, moving to next waypoint")
                    break

        plt.show()
        return trajectory

def load_data(filename):
    """
    加载雷达数据和地图信息
    
    Args:
        filename: 数据文件路径，可以是.pkl文件或.txt文件
        
    Returns:
        point_cloud: 点云数据列表 [(x1, y1), (x2, y2), ...]
        start: 起点坐标 (x, y)
        exits: 出口坐标列表 [(x1, y1), (x2, y2), ...]
    """
    # 检查文件是否存在
    if not os.path.exists(filename):
        print(f"错误: 文件 '{filename}' 不存在")
        # 创建测试数据
        print("创建测试数据...")
        point_cloud = [(i, j) for i in range(0, 20, 2) for j in range(0, 20, 2)]
        # 移除中心区域的点，创建一个通道
        point_cloud = [p for p in point_cloud if not (5 < p[0] < 15 and 5 < p[1] < 15)]
        start = (2, 2)
        exits = [(18, 18)]
        return point_cloud, start, exits
    
    # 根据文件扩展名决定如何加载数据
    _, ext = os.path.splitext(filename)
    
    if ext.lower() == '.pkl':
        # 加载pickle文件
        try:
            print(f"尝试加载pickle文件: {filename}")
            with open(filename, 'rb') as f:
                data = pickle.load(f)
                
                # 检查数据格式是否正确
                if not isinstance(data, dict):
                    print(f"错误: pickle文件格式不正确，应为字典类型，实际为 {type(data)}")
                    raise ValueError("数据格式不正确")
                
                required_keys = ['point_cloud', 'start', 'exits']
                missing_keys = [key for key in required_keys if key not in data]
                if missing_keys:
                    print(f"错误: pickle文件缺少必要的键: {missing_keys}")
                    raise ValueError("数据缺少必要的键")
                
                point_cloud = data['point_cloud']
                start = data['start']
                exits = data['exits']
                
                print(f"成功加载pickle数据:")
                print(f"  - 点云数量: {len(point_cloud)}")
                print(f"  - 起点位置: {start}")
                print(f"  - 出口数量: {len(exits)}")
                
                return point_cloud, start, exits
                
        except Exception as e:
            print(f"加载pickle文件失败: {e}")
            # 如果pickle加载失败，尝试加载默认的explore_result_1.pkl
            if filename != 'explore_result_1.pkl':
                print("尝试加载默认的explore_result_1.pkl文件...")
                try:
                    return load_data('explore_result_1.pkl')
                except:
                    pass
    
    # 尝试从雷达数据文件加载
    try:
        print(f"尝试加载雷达数据文件: {filename}")
        with open(filename, 'r') as f:
            lines = f.readlines()
            
            # 读取元数据
            metadata = lines[0].strip().split()
            if len(metadata) >= 3:
                # 格式: grid_size car_radius num_points
                grid_size = float(metadata[0])
                car_radius = float(metadata[1])
                
                # 读取入口位置
                entrance_line = lines[1].strip().split()
                entrance = (float(entrance_line[0]), float(entrance_line[1]))
                
                # 读取出口位置
                exits = []
                exit_count = int(lines[2].strip())
                for i in range(exit_count):
                    exit_line = lines[3+i].strip().split()
                    exits.append((float(exit_line[0]), float(exit_line[1])))
                
                # 读取点云数据
                point_cloud = []
                for i in range(3+exit_count, len(lines)):
                    if lines[i].strip():
                        point = lines[i].strip().split()
                        if len(point) >= 2:
                            point_cloud.append((float(point[0]), float(point[1])))
                
                print(f"成功从雷达数据文件加载:")
                print(f"  - 点云数量: {len(point_cloud)}")
                print(f"  - 起点位置: {entrance}")
                print(f"  - 出口数量: {len(exits)}")
                
                return point_cloud, entrance, exits
            else:
                raise ValueError("雷达数据文件格式不正确")
    except Exception as e:
        print(f"从雷达数据文件加载失败: {e}")
        
    # 如果所有尝试都失败，创建测试数据
    print("所有加载尝试失败，创建测试数据...")
    point_cloud = [(i, j) for i in range(0, 20, 2) for j in range(0, 20, 2)]
    # 移除中心区域的点，创建一个通道
    point_cloud = [p for p in point_cloud if not (5 < p[0] < 15 and 5 < p[1] < 15)]
    start = (2, 2)
    exits = [(18, 18)]
    return point_cloud, start, exits

def main():
    """
    主函数：解析命令行参数并启动DWA路径跟踪
    """
    parser = argparse.ArgumentParser(description='使用DWA算法跟踪A*规划的路径')
    parser.add_argument('input_file', nargs='?', default='1_radar_data.txt', 
                      help='输入文件路径，可以是.pkl文件或.txt文件 (默认: 1_radar_data.txt)')
    parser.add_argument('--no-animation', action='store_true', help='禁用路径规划动画')
    parser.add_argument('--speed', type=float, default=0.01, help='路径规划动画速度 (暂停时间，越小越快，默认0.01)')
    args = parser.parse_args()
    
    # 加载数据
    point_cloud, start, exits = load_data(args.input_file)
    
    # 创建路径规划器
    planner = GlobalLocalPathPlanner(point_cloud, start, exits)
    
    # 执行路径规划和DWA跟踪
    planner.plan(animation=not args.no_animation, animation_speed=args.speed)

if __name__ == "__main__":
    main() 