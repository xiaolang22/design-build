import subprocess
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import argparse
import time
from matplotlib.patches import Circle, Arrow
import pickle
import heapq
from collections import deque

# Add global animation flag and speed - will be set by command line argument
show_animation = True
animation_speed = 0.01  # 默认动画速度（暂停时间，越小越快）
car_animation_speed = 0.05  # 默认小车动画速度
use_dwa = False  # 是否使用DWA控制器

# 导入来自path_DWA.py的DWA相关代码
class Config:
    def __init__(self):
        # 机器人运动参数 - 调整速度参数
        self.max_speed = 6.0  # 增加最大速度 [m/s]
        self.min_speed = 0.2  # 增加最小速度 [m/s]
        self.max_yawrate = 80.0 * math.pi / 180.0  # 增加最大角速度
        self.max_accel = 1.0  # 增加加速度 [m/ss]
        self.max_dyawrate = 80.0 * math.pi / 180.0  # 增加角加速度

        # DWA参数调整
        self.v_reso = 0.2  # 速度分辨率 [m/s]
        self.yawrate_reso = 0.3 * math.pi / 180.0  # 角速度分辨率
        self.dt = 0.1  # 减小采样时间 [s]
        self.predict_time = 1.0  # 预测时间缩短

        # 代价函数权重
        self.to_goal_cost_gain = 2.0  # 增加到目标的代价权重
        self.speed_cost_gain = 1.5
        self.obstacle_cost_gain = 3.0  # 新增障碍物代价权重

        self.robot_radius = 0.2  # [m]

        # A*和地图参数
        self.grid_res = 0.15  # 栅格分辨率
        self.obstacle_radius = 0.3  # 障碍物膨胀半径
        self.a_star_step = 2  # A*步长

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

class DWAController:
    """
    来自path_DWA.py的Dynamic Window Approach控制器
    """
    
    def __init__(self, config=None):
        """
        初始化DWA控制器
        
        Args:
            config: 配置参数，如果为None则使用默认参数
        """
        self.config = config if config else Config()
        self.x = 0.0  # 机器人x坐标 [m]
        self.y = 0.0  # 机器人y坐标 [m]
        self.theta = 0.0  # 机器人朝向 [rad]
        self.v = 0.0  # 机器人线速度 [m/s]
        self.omega = 0.0  # 机器人角速度 [rad/s]
        
        # 跟踪状态
        self.current_waypoint_idx = 0
        self.stuck_count = 0
        self.last_distance = float('inf')
    
    def set_position(self, x, y, theta=None, next_point=None):
        """
        设置机器人位置和朝向
        
        Args:
            x: x坐标
            y: y坐标
            theta: 朝向角度（弧度），如果为None且提供了next_point，则计算朝向
            next_point: 下一个路径点，如果提供则计算朝向
        """
        self.x = x
        self.y = y
        
        # 如果提供了next_point，计算朝向
        if theta is None and next_point is not None:
            dx = next_point[0] - x
            dy = next_point[1] - y
            self.theta = math.atan2(dy, dx)
        else:
            self.theta = theta if theta is not None else 0.0
            
        self.v = 0.0
        self.omega = 0.0
    
    def motion(self, x, u, dt):
        """
        预测机器人运动
        
        Args:
            x: 状态 [x, y, theta, v, omega]
            u: 控制量 [v, omega]
            dt: 时间步长
            
        Returns:
            x_new: 新状态
        """
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x
    
    def calc_dynamic_window(self, x):
        """
        计算动态窗口
        
        Args:
            x: 当前状态 [x, y, theta, v, omega]
            
        Returns:
            vr: 速度范围 [v_min, v_max, omega_min, omega_max]
        """
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
        """
        计算轨迹
        
        Args:
            x_init: 初始状态 [x, y, theta, v, omega]
            v: 线速度
            w: 角速度
            
        Returns:
            trajectory: 轨迹
        """
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, w], self.config.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory
    
    def calc_to_goal_cost(self, trajectory, goal):
        """
        计算到目标的代价
        
        Args:
            trajectory: 轨迹
            goal: 目标点 [x, y]
            
        Returns:
            cost: 代价
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        goal_dis = math.sqrt(dx ** 2 + dy ** 2)
        return self.config.to_goal_cost_gain * goal_dis
    
    def calc_obstacle_cost(self, traj, ob):
        """
        计算障碍物代价
        
        Args:
            traj: 轨迹
            ob: 障碍物列表 [[x1,y1], [x2,y2], ...]
            
        Returns:
            cost: 代价
        """
        if len(ob) == 0:
            return 0.0
        
        # 使用自定义的简单KD树替代scipy的cKDTree
        kdtree = SimpleKDTree(ob)
        dists, _ = kdtree.query(traj[:, :2])
        min_dist = np.min(dists)
        
        if min_dist <= self.config.robot_radius:
            return float("inf")
        
        # 指数衰减的代价计算
        obstacle_cost = math.exp(-min_dist) * self.config.obstacle_cost_gain
        return obstacle_cost
    
    def dwa_control(self, x, u, goal, ob):
        """
        DWA控制算法
        
        Args:
            x: 当前状态 [x, y, theta, v, omega]
            u: 当前控制量 [v, omega]
            goal: 目标点 [x, y]
            ob: 障碍物列表 [[x1,y1], [x2,y2], ...]
            
        Returns:
            u: 最优控制量 [v, omega]
            trajectory: 最优轨迹
        """
        vr = self.calc_dynamic_window(x)
        min_cost = float('inf')
        best_u = u
        best_trajectory = None
        
        # 增加采样点数
        for v in np.linspace(vr[0], vr[1], 8):
            for w in np.linspace(vr[2], vr[3], 8):
                trajectory = self.calc_trajectory(x, v, w)
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.calc_obstacle_cost(trajectory, ob)
                final_cost = (
                    to_goal_cost +
                    speed_cost +
                    ob_cost
                )
                
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_trajectory = trajectory
        
        if best_trajectory is None:
            return [0.0, 0.0], None
        
        return best_u, best_trajectory
    
    def interpolate_waypoints(self, path, max_distance=3.0):
        """
        根据最大距离插值路径点，使相邻点距离不超过max_distance
        
        Args:
            path: 路径点列表 [[x1,y1], [x2,y2], ...]
            max_distance: 最大距离
            
        Returns:
            interpolated_path: 插值后的路径
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
    
    def update(self, v, omega):
        """
        更新机器人状态
        
        Args:
            v: 线速度
            omega: 角速度
        """
        self.x += v * math.cos(self.theta) * self.config.dt
        self.y += v * math.sin(self.theta) * self.config.dt
        self.theta += omega * self.config.dt
        self.v = v
        self.omega = omega
    
    def follow_path(self, path, obstacles=None, ax=None, fig=None, animation_speed=0.05):
        """
        跟踪路径
        
        Args:
            path: 路径点列表 [(x, y), ...]
            obstacles: 障碍物列表 [(x, y), ...]
            ax: matplotlib轴对象，用于可视化
            fig: matplotlib图形对象，用于可视化
            animation_speed: 动画速度，越小越快
            
        Returns:
            success: 是否成功到达目标
        """
        if len(path) < 2:
            print("路径太短，无法跟踪")
            return False
        
        if obstacles is None:
            obstacles = []
        
        # 初始化
        self.current_waypoint_idx = 1  # 从第二个点开始（第一个点是起点）
        self.stuck_count = 0
        self.last_distance = float('inf')
        
        # 设置起点，并计算朝向第一个目标点
        self.set_position(path[0][0], path[0][1], next_point=path[1])
        
        # 创建可视化元素
        if ax is not None and fig is not None:
            # 创建机器人对象
            robot_circle = Circle((self.x, self.y), self.config.robot_radius, 
                                  color='blue', alpha=0.7, label='机器人')
            ax.add_patch(robot_circle)
            
            # 创建方向指示箭头
            arrow_length = self.config.robot_radius * 1.5
            arrow_dx = arrow_length * math.cos(self.theta)
            arrow_dy = arrow_length * math.sin(self.theta)
            arrow = ax.arrow(self.x, self.y, arrow_dx, arrow_dy, 
                            head_width=self.config.robot_radius*0.5, 
                            head_length=self.config.robot_radius*0.8, 
                            fc='red', ec='red', zorder=3)
            
            # 路径跟踪线
            path_trace, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.6, label='实际路径')
            trace_x, trace_y = [self.x], [self.y]
            
            # 当前目标点
            target_point, = ax.plot([], [], 'yo', markersize=6, label='当前目标')
            
            # 预测轨迹
            pred_traj, = ax.plot([], [], 'g--', linewidth=1, alpha=0.7, label='预测轨迹')
            
        # 使用path_DWA.py的方法跟踪路径
        # 计算初始朝向
        if len(path) > 1:
            dx = path[1][0] - path[0][0]
            dy = path[1][1] - path[0][1]
            init_yaw = math.atan2(dy, dx)
        else:
            init_yaw = math.pi/2

        # 初始化机器人状态
        x = np.array([path[0][0], path[0][1], init_yaw, 0.0, 0.0])  # 初始速度为0
        u = np.array([0.0, 0.0])

        # 定义障碍物数组
        obstacles_array = np.array(obstacles)
        
        # 插值路径点，避免距离太远
        path = self.interpolate_waypoints(path, max_distance=2.5)
        
        # 路径跟踪循环
        goal_reached = False
        iter_count = 0
        max_iters = 1000  # 最大迭代次数，防止无限循环
        
        # 局部路径规划和避障
        for i, goal in enumerate(path[1:]):
            goal_reached = False
            stuck_count = 0
            while not goal_reached and stuck_count < 200 and iter_count < max_iters:
                dist_to_goal = np.linalg.norm(x[:2] - goal)
                if dist_to_goal <= self.config.robot_radius * 1.5:
                    goal_reached = True
                    break

                u, trajectory_segment = self.dwa_control(x, u, goal, obstacles_array)
                x = self.motion(x, u, self.config.dt)
                
                # 可视化
                if ax is not None and fig is not None and iter_count % 5 == 0:
                    # 更新机器人位置
                    robot_circle.center = (x[0], x[1])
                    
                    # 更新方向箭头
                    arrow.remove()
                    arrow_dx = arrow_length * math.cos(x[2])
                    arrow_dy = arrow_length * math.sin(x[2])
                    arrow = ax.arrow(x[0], x[1], arrow_dx, arrow_dy, 
                                    head_width=self.config.robot_radius*0.5, 
                                    head_length=self.config.robot_radius*0.8, 
                                    fc='red', ec='red', zorder=3)
                    
                    # 更新路径跟踪线
                    trace_x.append(x[0])
                    trace_y.append(x[1])
                    path_trace.set_data(trace_x, trace_y)
                    
                    # 更新当前目标点
                    target_point.set_data([goal[0]], [goal[1]])
                    
                    # 更新预测轨迹
                    if trajectory_segment is not None and len(trajectory_segment) > 1:
                        pred_traj.set_data(trajectory_segment[:, 0], trajectory_segment[:, 1])
                    else:
                        pred_traj.set_data([], [])
                    
                    # 更新标题
                    progress = (i + 1) / len(path) * 100
                    ax.set_title(f"DWA控制: 完成度 {progress:.1f}%", fontsize=14)
                    
                    # 刷新图形
                    fig.canvas.draw()
                    plt.pause(animation_speed)
                
                stuck_count += 1
                iter_count += 1
        
        # 显示结果
        if ax is not None:
            if goal_reached:
                ax.set_title("DWA控制: 成功到达目标！", fontsize=14)
            else:
                ax.set_title("DWA控制: 未能到达目标", fontsize=14)
            
            if fig is not None:
                fig.canvas.draw()
                plt.pause(animation_speed * 5)  # 暂停一会儿显示结果
        
        return goal_reached

class MapBuilder:
    def __init__(self, filename):
        """
        初始化地图构建器
        
        参数:
        filename: 雷达数据文件路径
        """
        self.filename = filename
        self.grid_size = None
        self.car_radius = None
        self.radar_data = []
        self.entrance = None
        self.exits = []
        self.obstacle_map = None
        self.width = None
        self.height = None
        self.origin_x = None
        self.origin_y = None
        self.fig = None  # 添加图形对象
        self.ax = None   # 添加坐标轴对象
        self.actual_grid_size = None  # 实际使用的栅格大小
        
    def read_radar_data(self):
        """
        读取雷达数据文件
        """
        try:
            with open(self.filename, 'r') as f:
                lines = f.readlines()
                
                # 读取元数据
                metadata = lines[0].strip().split()
                self.grid_size = float(metadata[metadata.index('GRID_SIZE')+1])
                self.car_radius = float(metadata[metadata.index('CAR_RADIUS')+1])
                print(f"元数据: 栅格大小={self.grid_size}m, 小车半径={self.car_radius}m")
                
                # 读取雷达数据
                self.radar_data = []
                for i in range(1, len(lines)-1):
                    values = lines[i].strip().split()
                    car_x, car_y = float(values[0]), float(values[1])
                    scan = [float(v) for v in values[2:]]
                    self.radar_data.append(((car_x, car_y), scan))
                
                # 读取入口和出口
                last_line = lines[-1].strip().split()
                entrance_idx = last_line.index('ENTRANCE')
                self.entrance = (float(last_line[entrance_idx+1]), float(last_line[entrance_idx+2]))
                
                self.exits = []
                i = 0
                while True:
                    exit_tag = f'EXIT{i+1}'
                    if exit_tag in last_line:
                        exit_idx = last_line.index(exit_tag)
                        self.exits.append((float(last_line[exit_idx+1]), float(last_line[exit_idx+2])))
                        i += 1
                    else:
                        break
                
                print(f"读取到 {len(self.radar_data)} 个雷达扫描数据点")
                print(f"入口坐标: {self.entrance}")
                print(f"出口坐标: {self.exits}")
                
                return True
        except Exception as e:
            print(f"读取文件出错: {str(e)}")
            return False
    
    def build_map(self):
        """
        构建障碍物地图
        """
        if not self.radar_data:
            print("没有雷达数据，无法构建地图")
            return False
        
        # 计算地图边界
        all_x, all_y = [], []
        for (car_x, car_y), scan in self.radar_data:
            all_x.append(car_x)
            all_y.append(car_y)
            for angle, dist in enumerate(scan):
                if dist <= 0:  # 跳过无效值
                    continue
                rad_angle = np.deg2rad(angle)
                x = car_x + dist * np.cos(rad_angle)
                y = car_y + dist * np.sin(rad_angle)
                all_x.append(x)
                all_y.append(y)
        
        # 设置地图边界，添加一定边距
        margin = 2.0  # 边距2米
        min_x, max_x = min(all_x) - margin, max(all_x) + margin
        min_y, max_y = min(all_y) - margin, max(all_y) + margin
        
        # 使用4倍的栅格大小（进一步降低分辨率）
        coarse_grid_size = self.grid_size * 4
        print(f"原始栅格大小: {self.grid_size}m, 放大后栅格大小: {coarse_grid_size}m")
        
        # 计算地图尺寸
        self.width = int(np.ceil((max_x - min_x) / coarse_grid_size))
        self.height = int(np.ceil((max_y - min_y) / coarse_grid_size))
        self.origin_x = min_x
        self.origin_y = min_y
        
        print(f"地图范围: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}]")
        print(f"地图尺寸: {self.width} x {self.height} 栅格")
        
        # 创建空地图
        self.obstacle_map = np.zeros((self.width, self.height), dtype=bool)
        
        # 创建可见性掩码（用于美观显示）
        self.visibility_mask = np.zeros((self.width, self.height), dtype=bool)
        
        # 收集所有雷达点云（用于可视化）
        self.all_radar_points = []
        
        # 填充障碍物和可见性掩码
        for (car_x, car_y), scan in self.radar_data:
            # 计算车辆位置的栅格坐标
            car_ix = int((car_x - self.origin_x) / coarse_grid_size)
            car_iy = int((car_y - self.origin_y) / coarse_grid_size)
            
            # 标记车辆位置为可见
            if 0 <= car_ix < self.width and 0 <= car_iy < self.height:
                self.visibility_mask[car_ix, car_iy] = True
            
            for angle, dist in enumerate(scan):
                if dist <= 0:  # 跳过无效值
                    continue
                
                # 计算障碍物点的绝对坐标
                rad_angle = np.deg2rad(angle)
                x = car_x + dist * np.cos(rad_angle)
                y = car_y + dist * np.sin(rad_angle)
                
                # 保存点云数据
                self.all_radar_points.append((x, y))
                
                # 转换为栅格坐标，使用较大的栅格
                ix = int((x - self.origin_x) / coarse_grid_size)
                iy = int((y - self.origin_y) / coarse_grid_size)
                
                # 标记为障碍物及其周围区域(考虑小车半径)
                if 0 <= ix < self.width and 0 <= iy < self.height:
                    # 中心点标记为障碍物
                    self.obstacle_map[ix, iy] = True
                    
                    # 考虑小车半径，在障碍物周围膨胀
                    inflation_radius = int(np.ceil(self.car_radius / coarse_grid_size))
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        for dy in range(-inflation_radius, inflation_radius + 1):
                            # 计算距离
                            distance = math.sqrt(dx**2 + dy**2) * coarse_grid_size
                            if distance <= self.car_radius:
                                nx, ny = ix + dx, iy + dy
                                if 0 <= nx < self.width and 0 <= ny < self.height:
                                    self.obstacle_map[nx, ny] = True
                
                # 更新可见性掩码 - 从车辆到障碍物的路径上所有栅格都是可见的
                # 使用Bresenham算法绘制直线
                if 0 <= car_ix < self.width and 0 <= car_iy < self.height and 0 <= ix < self.width and 0 <= iy < self.height:
                    # 计算直线上的所有点
                    n_points = int(max(abs(ix - car_ix), abs(iy - car_iy)) + 1)
                    if n_points > 1:
                        x_line = np.linspace(car_ix, ix, n_points, dtype=int)
                        y_line = np.linspace(car_iy, iy, n_points, dtype=int)
                        
                        # 标记这些点为可见
                        for x_pt, y_pt in zip(x_line, y_line):
                            if 0 <= x_pt < self.width and 0 <= y_pt < self.height:
                                self.visibility_mask[x_pt, y_pt] = True
        
        # 标记入口和出口以及它们周围的区域为可见且非障碍物
        if self.entrance:
            entrance_ix = int((self.entrance[0] - self.origin_x) / coarse_grid_size)
            entrance_iy = int((self.entrance[1] - self.origin_y) / coarse_grid_size)
            
            # 标记入口周围区域为可见且非障碍物
            radius = 5  # 增大搜索半径
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = entrance_ix + dx, entrance_iy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.visibility_mask[nx, ny] = True
                        # 只有在中心附近才清除障碍物
                        if dx*dx + dy*dy <= 2*2:
                            self.obstacle_map[nx, ny] = False
        
        for exit_point in self.exits:
            exit_ix = int((exit_point[0] - self.origin_x) / coarse_grid_size)
            exit_iy = int((exit_point[1] - self.origin_y) / coarse_grid_size)
            
            # 标记出口周围区域为可见且非障碍物
            radius = 10  # 增大搜索半径
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = exit_ix + dx, exit_iy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.visibility_mask[nx, ny] = True
                        # 只有在中心附近才清除障碍物
                        if dx*dx + dy*dy <= 3*3:
                            self.obstacle_map[nx, ny] = False
        
        # 保存实际使用的栅格大小
        self.actual_grid_size = coarse_grid_size
        print(f"实际使用的栅格大小: {self.actual_grid_size}m")
        print("地图构建完成")
        return True
    
    def plan_path(self, start=None, goal=None, search_callback=None):
        """
        使用A*算法规划路径
        
        参数:
        start: 起点坐标，默认为入口
        goal: 终点坐标，默认为第一个出口
        search_callback: 搜索回调函数，用于动画显示
        
        返回:
        path: 路径点列表 [(x, y), ...]
        """
        if self.obstacle_map is None:
            print("地图未构建，无法规划路径")
            return None
        
        if start is None:
            start = self.entrance
        if goal is None and self.exits:
            goal = self.exits[0]
        
        if start is None or goal is None:
            print("没有有效的起点或终点")
            return None
        
        print(f"规划路径: 从 {start} 到 {goal}")
        
        # 转换为栅格坐标
        start_ix = int((start[0] - self.origin_x) / self.actual_grid_size)
        start_iy = int((start[1] - self.origin_y) / self.actual_grid_size)
        goal_ix = int((goal[0] - self.origin_x) / self.actual_grid_size)
        goal_iy = int((goal[1] - self.origin_y) / self.actual_grid_size)
        
        # 检查起点和终点是否在障碍物上
        if start_ix < 0 or start_ix >= self.width or start_iy < 0 or start_iy >= self.height:
            print("起点超出地图范围")
            return None
        if goal_ix < 0 or goal_ix >= self.width or goal_iy < 0 or goal_iy >= self.height:
            print("终点超出地图范围")
            return None
        
        # 检查起点是否在障碍物上或不可见区域
        if self.obstacle_map[start_ix, start_iy] or not self.visibility_mask[start_ix, start_iy]:
            print("起点在障碍物上或不可见区域，尝试寻找附近的空闲点")
            found = False
            search_radius = 5  # 增大搜索半径
            for radius in range(1, search_radius + 1):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        # 只考虑当前半径上的点
                        if abs(dx) == radius or abs(dy) == radius:
                            nx, ny = start_ix + dx, start_iy + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height and not self.obstacle_map[nx, ny] and self.visibility_mask[nx, ny]:
                                start_ix, start_iy = nx, ny
                                start = (self.origin_x + nx * self.actual_grid_size, self.origin_y + ny * self.actual_grid_size)
                                print(f"使用新起点: {start}")
                                found = True
                                break
                    if found:
                        break
                if found:
                    break
            if not found:
                print("无法找到有效的起点")
                return None
        
        # 检查终点是否在障碍物上或不可见区域
        if self.obstacle_map[goal_ix, goal_iy]:
            print("终点在障碍物上，尝试寻找附近的空闲点")
            found = False
            search_radius = 15  # 大幅增大终点搜索半径
            for radius in range(1, search_radius + 1):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        # 只考虑当前半径上的点
                        if abs(dx) == radius or abs(dy) == radius:
                            nx, ny = goal_ix + dx, goal_iy + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height and not self.obstacle_map[nx, ny]:
                                # 对于终点，我们允许它在未探索区域
                                goal_ix, goal_iy = nx, ny
                                goal = (self.origin_x + nx * self.actual_grid_size, self.origin_y + ny * self.actual_grid_size)
                                print(f"使用新终点: {goal}")
                                found = True
                                break
                    if found:
                        break
                if found:
                    break
            if not found:
                print("无法找到有效的终点")
                return None
        
        # 确保终点附近区域可见
        radius = 5
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = goal_ix + dx, goal_iy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.visibility_mask[nx, ny] = True
        
        # A*路径规划
        path = self.astar(start_ix, start_iy, goal_ix, goal_iy, search_callback)
        
        if path:
            print(f"规划完成，路径长度: {len(path)}个点")
            return path
        else:
            print("无法找到有效路径")
            return None
    
    def astar(self, start_x, start_y, goal_x, goal_y, search_callback=None):
        """
        A*算法实现
        
        参数:
        start_x, start_y: 起点栅格坐标
        goal_x, goal_y: 终点栅格坐标
        search_callback: 搜索回调函数，用于动画显示
        
        返回:
        path: 路径点列表 [(x, y), ...]
        """
        class Node:
            def __init__(self, x, y, cost, parent_index):
                self.x = x
                self.y = y
                self.cost = cost
                self.parent_index = parent_index
            
            def __eq__(self, other):
                return self.x == other.x and self.y == other.y
        
        # 运动模型: 8个方向
        motion = [
            [1, 0, 1],        # 右
            [0, 1, 1],        # 上
            [-1, 0, 1],       # 左
            [0, -1, 1],       # 下
            [1, 1, math.sqrt(2)],    # 右上
            [-1, 1, math.sqrt(2)],   # 左上
            [-1, -1, math.sqrt(2)],  # 左下
            [1, -1, math.sqrt(2)]    # 右下
        ]
        
        # 初始化起点和终点
        start_node = Node(start_x, start_y, 0, -1)
        goal_node = Node(goal_x, goal_y, 0, -1)
        
        # 初始化开集和闭集
        open_set = {}
        closed_set = {}
        
        # 将起点加入开集
        open_set[self.calc_grid_index(start_node)] = start_node
        
        node_count = 0
        
        # 计算终点到起点的距离
        goal_to_start_dist = math.hypot(goal_x - start_x, goal_y - start_y)
        
        while True:
            # 如果开集为空，则无法找到路径
            if len(open_set) == 0:
                print("开集为空，无法找到路径")
                return None
            
            # 找到开集中代价最小的节点
            current_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[o], goal_node))
            current = open_set[current_id]
            
            # 如果提供了搜索回调函数，则调用
            if search_callback:
                should_stop = search_callback(current.x, current.y)
                if should_stop:  # 如果回调函数返回True，则停止搜索
                    return None
            
            # 如果到达终点，则构建路径并返回
            if current.x == goal_node.x and current.y == goal_node.y:
                print(f"找到目标！共探索了{len(closed_set) + 1}个节点")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                
                # 获取最终路径
                path = self.calc_final_path(goal_node, closed_set)
                return path
            
            # 将当前节点从开集移到闭集
            del open_set[current_id]
            closed_set[current_id] = current
            
            # 探索周围的节点
            for dx, dy, cost in motion:
                next_x = current.x + dx
                next_y = current.y + dy
                
                # 检查节点是否在地图范围内
                if next_x < 0 or next_x >= self.width or next_y < 0 or next_y >= self.height:
                    continue
                
                # 检查节点是否为障碍物
                if self.obstacle_map[next_x, next_y]:
                    continue
                
                # 计算当前节点到终点的距离
                dist_to_goal = math.hypot(next_x - goal_x, next_y - goal_y)
                
                # 检查节点是否在已探索区域内（使用可见性掩码）
                # 但如果已经非常接近目标，或者在目标附近，则允许穿过未探索区域
                if not self.visibility_mask[next_x, next_y]:
                    # 如果距离终点很近（小于起点到终点距离的15%）或者在终点附近5个格子内，允许通过
                    if dist_to_goal < goal_to_start_dist * 0.15 or dist_to_goal < 5:
                        pass  # 允许通过未探索区域
                    else:
                        continue  # 否则跳过未探索区域
                
                # 创建新节点
                node = Node(next_x, next_y, current.cost + cost, current_id)
                node_id = self.calc_grid_index(node)
                
                # 如果节点已经在闭集中，则跳过
                if node_id in closed_set:
                    continue
                
                # 如果节点不在开集中，则添加
                if node_id not in open_set:
                    open_set[node_id] = node
                # 如果节点已在开集中，但新路径更好，则更新
                elif open_set[node_id].cost > node.cost:
                    open_set[node_id] = node
    
    def calc_grid_index(self, node):
        """计算节点的唯一索引"""
        return node.y * self.width + node.x
    
    def calc_heuristic(self, node, goal):
        """计算启发式函数值（欧几里得距离）"""
        return math.hypot(node.x - goal.x, node.y - goal.y)
    
    def calc_final_path(self, goal_node, closed_set):
        """计算最终路径"""
        path = [(goal_node.x, goal_node.y)]
        parent = goal_node.parent_index
        
        while parent != -1:
            node = closed_set[parent]
            path.append((node.x, node.y))
            parent = node.parent_index
        
        return path[::-1]  # 反转路径，从起点到终点
    
    def visualize(self, path=None):
        """
        可视化地图和路径
        
        参数:
        path: 路径点列表 [(x, y), ...]
        """
        if self.obstacle_map is None:
            print("地图未构建，无法可视化")
            return
        
        # 创建图形
        plt.figure(figsize=(10, 8))
        ax = plt.gca()
        
        # 设置标题
        plt.title('地图与路径规划', fontsize=16)
        
        # 1. 首先绘制底图 - 灰色为未探索区域，白色为可见区域
        not_visible = ~self.visibility_mask
        cmap = plt.cm.colors.ListedColormap(['white', 'lightgray'])
        ax.imshow(not_visible.T, cmap=cmap, origin='lower', 
                 extent=[self.origin_x, self.origin_x + self.width * self.actual_grid_size,
                        self.origin_y, self.origin_y + self.height * self.actual_grid_size],
                 alpha=0.7, zorder=0)
        
        # 2. 绘制障碍物点云 - 黑色小点
        if hasattr(self, 'all_radar_points') and self.all_radar_points:
            # 采样点云，避免过多点导致图像过于密集
            sampled_points = self.all_radar_points[::10]  # 每10个点取1个
            if sampled_points:
                radar_x, radar_y = zip(*sampled_points)
                ax.scatter(radar_x, radar_y, s=1, c='black', alpha=0.5, label='障碍物点云', zorder=2)
        
        # 3. 绘制入口
        if self.entrance:
            ax.plot(self.entrance[0], self.entrance[1], 'bo', markersize=10, label='入口')
            ax.text(self.entrance[0] + 0.2, self.entrance[1] + 0.2, '入口', fontsize=12)
        
        # 4. 绘制出口
        for i, exit_pos in enumerate(self.exits):
            ax.plot(exit_pos[0], exit_pos[1], 'r*', markersize=10)
            ax.text(exit_pos[0] + 0.2, exit_pos[1] + 0.2, f'出口{i+1}', fontsize=12)
        
        # 5. 绘制路径
        if path:
            # 判断path是否是栅格坐标还是世界坐标
            first_point = path[0]
            if isinstance(first_point, tuple) and len(first_point) == 2:
                # 检查是否是栅格坐标（看第一个点是否小于地图尺寸）
                if isinstance(first_point[0], int) or (first_point[0] < self.width and first_point[1] < self.height):
                    # 是栅格坐标，转换为世界坐标
                    world_path = []
                    for ix, iy in path:
                        x = self.origin_x + (ix + 0.5) * self.actual_grid_size
                        y = self.origin_y + (iy + 0.5) * self.actual_grid_size
                        world_path.append((x, y))
                    path_x, path_y = zip(*world_path)
                else:
                    # 已经是世界坐标
                    path_x, path_y = zip(*path)
            else:
                # 格式不对，直接使用
                path_x, path_y = zip(*path)
                
            ax.plot(path_x, path_y, 'g-', linewidth=2, label='规划路径')
            ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='起点')
            ax.plot(path_x[-1], path_y[-1], 'gx', markersize=8, label='终点')
        
        # 设置轴标签和网格
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        # 保存和显示
        plt.tight_layout()
        plt.savefig('map_and_path.png', dpi=300)
        plt.close()  # 保存后关闭图形，不显示


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='基于雷达数据构建地图并规划路径')
    parser.add_argument('file', nargs='?', help='雷达数据文件路径')
    parser.add_argument('--no-animation', action='store_true', help='禁用路径规划动画')
    parser.add_argument('--speed', type=float, default=0.01, help='动画速度 (暂停时间，越小越快，默认0.01)')
    parser.add_argument('--all-exits', action='store_true', help='为所有出口规划路径并显示最短路径')
    parser.add_argument('--no-car-animation', action='store_true', help='禁用小车移动动画')
    parser.add_argument('--car-speed', type=float, default=0.05, help='小车动画速度 (暂停时间，越小越快，默认0.05)')
    parser.add_argument('--use-dwa', action='store_true', help='使用DWA控制器跟踪路径')
    args = parser.parse_args()
    
    # 设置动画标志和速度
    global show_animation, animation_speed, car_animation_speed, use_dwa
    if args.no_animation:
        show_animation = False
    animation_speed = args.speed
    car_animation_speed = args.car_speed
    use_dwa = args.use_dwa
    
    # 获取文件路径
    if args.file:
        filename = args.file
    else:
        filename = input("请输入雷达数据文件路径: ")
    
    # 初始化地图构建器
    map_builder = MapBuilder(filename)
    
    # 读取雷达数据
    if not map_builder.read_radar_data():
        print("读取雷达数据失败，退出程序")
        return
    
    # 构建地图
    if not map_builder.build_map():
        print("构建地图失败，退出程序")
        return
    
    # 规划到所有出口的路径
    if args.all_exits or len(map_builder.exits) > 1:
        find_best_exit(map_builder, args.no_car_animation)
    else:
        # 如果只有一个出口，也使用动态显示
        if show_animation:
            # 创建单个图形
            fig = plt.figure(figsize=(12, 10))
            ax = plt.gca()  # 获取当前坐标轴
            
            # 设置标题
            fig.suptitle("路径规划过程", fontsize=16)
            
            # 绘制地图背景 - 使用可见性掩码
            not_visible = ~map_builder.visibility_mask
            cmap = plt.cm.colors.ListedColormap(['white', 'lightgray'])
            ax.imshow(not_visible.T, cmap=cmap, origin='lower', 
                     extent=[map_builder.origin_x, map_builder.origin_x + map_builder.width * map_builder.actual_grid_size,
                            map_builder.origin_y, map_builder.origin_y + map_builder.height * map_builder.actual_grid_size],
                     alpha=0.7, zorder=0)
            
            # 绘制障碍物点云
            if hasattr(map_builder, 'all_radar_points') and map_builder.all_radar_points:
                # 采样点云，避免过多点导致图像过于密集
                sampled_points = map_builder.all_radar_points[::10]  # 每10个点取1个
                if sampled_points:
                    radar_x, radar_y = zip(*sampled_points)
                    ax.scatter(radar_x, radar_y, s=1, c='black', alpha=0.5, label='障碍物点云', zorder=2)
            
            # 绘制入口
            if map_builder.entrance:
                ax.plot(map_builder.entrance[0], map_builder.entrance[1], 'bo', markersize=10, label='入口')
                ax.text(map_builder.entrance[0] + 0.2, map_builder.entrance[1] + 0.2, '入口', fontsize=12)
            
            # 绘制出口
            for i, exit_pos in enumerate(map_builder.exits):
                ax.plot(exit_pos[0], exit_pos[1], 'r*', markersize=10)
                ax.text(exit_pos[0] + 0.2, exit_pos[1] + 0.2, f'出口{i+1}', fontsize=12)
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.grid(True)
            
            plt.pause(animation_speed)
            
            # 使用动画规划路径，自定义回调函数显示搜索过程
            search_points = []  # 保存搜索过程中的点对象
            
            def search_callback(x, y):
                # 在地图上绘制搜索点
                world_x = map_builder.origin_x + (x + 0.5) * map_builder.actual_grid_size
                world_y = map_builder.origin_y + (y + 0.5) * map_builder.actual_grid_size
                point = ax.plot(world_x, world_y, '.', color='green', markersize=2, alpha=0.5)[0]
                search_points.append(point)
                if len(search_points) % 50 == 0:  # 每50个点更新一次显示
                    ax.set_title(f"正在规划路径 (已探索: {len(search_points)}个点)", fontsize=14)
                    plt.pause(animation_speed * 0.1)  # 更短的暂停时间以加快显示
                return False  # 继续搜索
            
            path = map_builder.plan_path(search_callback=search_callback)
            
            # 如果找到路径，绘制路径
            if path:
                # 转换为世界坐标
                world_path = []
                for ix, iy in path:
                    x = map_builder.origin_x + (ix + 0.5) * map_builder.actual_grid_size
                    y = map_builder.origin_y + (iy + 0.5) * map_builder.actual_grid_size
                    world_path.append((x, y))
                
                # 计算路径长度
                path_length = 0
                for j in range(1, len(world_path)):
                    dx = world_path[j][0] - world_path[j-1][0]
                    dy = world_path[j][1] - world_path[j-1][1]
                    path_length += math.sqrt(dx*dx + dy*dy)
                
                path_x, path_y = zip(*world_path)
                ax.plot(path_x, path_y, '-', color='green', linewidth=2, label=f'规划路径 ({path_length:.2f}米)')
                ax.plot(path_x[0], path_y[0], 'go', markersize=8, label='起点')
                ax.plot(path_x[-1], path_y[-1], 'gx', markersize=8, label='终点')
                
                ax.set_title(f"路径规划完成，长度: {path_length:.2f}米", fontsize=14)
                ax.legend(loc='upper right')
                subprocess.Popen([sys.executable, "pklinput_window.py"])
                plt.tight_layout()
                plt.savefig('map_and_path.png', dpi=300)
                
                # 展示小车沿路径移动的动画
                if not args.no_car_animation:
                    animate_car_movement(map_builder, path, ax, fig)
                
                plt.show()
            else:
                print("无法找到有效路径")
                plt.savefig('no_valid_path.png', dpi=300)
                plt.close()  # 关闭图形而不是显示
        else:
            # 如果禁用动画，使用原来的方法
            path = map_builder.plan_path()
            map_builder.visualize(path)


def find_best_exit(map_builder, no_car_animation=False):
    """
    为所有出口规划路径，并显示最短路径
    
    参数:
    map_builder: MapBuilder实例
    no_car_animation: 禁用小车移动动画标志
    """
    if not map_builder.exits:
        print("未找到任何出口，无法规划路径")
        return
        
    print(f"\n发现{len(map_builder.exits)}个出口，正在为每个出口规划路径...")
    
    # 存储所有路径
    paths = []
    path_lengths = []
    world_paths = []  # 存储世界坐标路径
    
    # 保存原始动画标志
    global show_animation, animation_speed
    original_animation = show_animation
    
    # 颜色列表用于绘制不同出口的路径
    colors = ['green', 'red', 'purple', 'orange', 'brown', 'pink', 'cyan', 'magenta']
    
    # 创建单个图形窗口
    if show_animation:
        # 创建单个图形
        fig = plt.figure(figsize=(12, 10))
        ax = plt.gca()  # 获取当前坐标轴
        
        # 设置标题
        fig.suptitle("路径规划过程", fontsize=16)
        
        # 绘制地图背景 - 使用可见性掩码
        not_visible = ~map_builder.visibility_mask
        cmap = plt.cm.colors.ListedColormap(['white', 'lightgray'])
        ax.imshow(not_visible.T, cmap=cmap, origin='lower', 
                 extent=[map_builder.origin_x, map_builder.origin_x + map_builder.width * map_builder.actual_grid_size,
                        map_builder.origin_y, map_builder.origin_y + map_builder.height * map_builder.actual_grid_size],
                 alpha=0.7, zorder=0)
        
        # 绘制障碍物点云
        if hasattr(map_builder, 'all_radar_points') and map_builder.all_radar_points:
            # 采样点云，避免过多点导致图像过于密集
            sampled_points = map_builder.all_radar_points[::10]  # 每10个点取1个
            if sampled_points:
                radar_x, radar_y = zip(*sampled_points)
                ax.scatter(radar_x, radar_y, s=1, c='black', alpha=0.5, label='障碍物点云', zorder=2)
        
        # 绘制入口
        if map_builder.entrance:
            ax.plot(map_builder.entrance[0], map_builder.entrance[1], 'bo', markersize=10, label='入口')
            ax.text(map_builder.entrance[0] + 0.2, map_builder.entrance[1] + 0.2, '入口', fontsize=12)
        
        # 绘制所有出口
        for i, exit_pos in enumerate(map_builder.exits):
            color = colors[i % len(colors)]
            ax.plot(exit_pos[0], exit_pos[1], '*', color=color, markersize=10)
            ax.text(exit_pos[0] + 0.2, exit_pos[1] + 0.2, f'出口{i+1}', fontsize=12, color=color)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True)
        
        plt.pause(animation_speed)
    
    # 保存所有路径线对象，以便后面可以删除
    path_lines = []
    search_points = []  # 保存搜索过程中的点对象
    
    # 为每个出口规划路径
    for i, exit_pos in enumerate(map_builder.exits):
        print(f"\n正在规划到出口{i+1}: {exit_pos}的路径")
        
        # 设置当前出口的颜色
        current_color = colors[i % len(colors)]
        
        if show_animation:
            # 清除之前的搜索点（如果有的话）
            for point in search_points:
                if point and point.get_visible():
                    point.remove()
            search_points = []
            
            # 更新标题，显示当前正在规划的出口
            ax.set_title(f"正在规划到出口{i+1}的路径", fontsize=14)
            plt.pause(animation_speed)
        
        # 规划路径
        path = None
        if show_animation:
            # 使用动画规划路径，自定义回调函数显示搜索过程
            def search_callback(x, y):
                # 在地图上绘制搜索点
                world_x = map_builder.origin_x + (x + 0.5) * map_builder.actual_grid_size
                world_y = map_builder.origin_y + (y + 0.5) * map_builder.actual_grid_size
                point = ax.plot(world_x, world_y, '.', color=current_color, markersize=2, alpha=0.5)[0]
                search_points.append(point)
                if len(search_points) % 50 == 0:  # 每50个点更新一次显示
                    ax.set_title(f"正在规划到出口{i+1}的路径 (已探索: {len(search_points)}个点)", fontsize=14)
                    plt.pause(animation_speed * 0.1)  # 更短的暂停时间以加快显示
                return False  # 继续搜索
            
            path = map_builder.plan_path(goal=exit_pos, search_callback=search_callback)
        else:
            path = map_builder.plan_path(goal=exit_pos)
        
        if path:
            # 计算路径长度
            world_path = []
            for ix, iy in path:
                x = map_builder.origin_x + (ix + 0.5) * map_builder.actual_grid_size
                y = map_builder.origin_y + (iy + 0.5) * map_builder.actual_grid_size
                world_path.append((x, y))
            
            world_paths.append(world_path)
            
            # 计算路径总长度
            path_length = 0
            for j in range(1, len(world_path)):
                dx = world_path[j][0] - world_path[j-1][0]
                dy = world_path[j][1] - world_path[j-1][1]
                path_length += math.sqrt(dx*dx + dy*dy)
            
            paths.append(path)
            path_lengths.append(path_length)
            print(f"出口{i+1}路径长度: {path_length:.2f}米 ({len(path)}个点)")
            
            # 如果有动画，在图中绘制当前路径
            if show_animation:
                path_x, path_y = zip(*world_path)
                line = ax.plot(path_x, path_y, '-', color=current_color, linewidth=2, 
                             label=f'到出口{i+1}的路径 ({path_length:.2f}米)')[0]
                path_lines.append(line)
                
                # 绘制路径起点和终点
                start_point = ax.plot(path_x[0], path_y[0], 'o', color=current_color, markersize=6)[0]
                end_point = ax.plot(path_x[-1], path_y[-1], 'x', color=current_color, markersize=6)[0]
                path_lines.extend([start_point, end_point])
                
                ax.set_title(f"到出口{i+1}的路径已规划 (长度: {path_length:.2f}米)")
                ax.legend(loc='upper right')
                plt.pause(animation_speed * 5)  # 显示一会儿当前路径
        else:
            paths.append(None)
            path_lengths.append(float('inf'))
            world_paths.append(None)
            print(f"无法规划到出口{i+1}的路径")
    
    # 找到最短路径
    if not path_lengths or all(length == float('inf') for length in path_lengths):
        print("\n无法规划到任何出口的路径")
        if show_animation:
            plt.savefig('no_valid_path.png', dpi=300)
            plt.close()  # 关闭图形而不是显示
        return
    
    # 找到最短路径的索引
    shortest_idx = path_lengths.index(min(path_lengths))
    shortest_path = paths[shortest_idx]
    shortest_color = colors[shortest_idx % len(colors)]
    
    print(f"\n最短路径是到出口{shortest_idx+1}的路径，长度为{path_lengths[shortest_idx]:.2f}米")
    
    # 如果有动画，先保留所有路径一段时间，然后再只显示最短路径
    if show_animation:
        # 更新标题，显示所有路径已完成
        ax.set_title(f"所有出口路径规划完成，共{len(map_builder.exits)}条路径", fontsize=14)
        
        # 删除所有搜索点
        for point in search_points:
            if point:
                point.remove()
        
        # 保留所有路径一段时间
        print("\n显示所有规划的路径...")
        plt.savefig('all_exit_paths_comparison.png', dpi=300)
        plt.pause(animation_speed * 20)  # 保留所有路径显示一段较长时间
        
        # 删除之前绘制的所有路径
        for line in path_lines:
            if line:
                line.remove()
        path_lines = []
        
        # 重新绘制最短路径
        shortest_world_path = world_paths[shortest_idx]
        path_x, path_y = zip(*shortest_world_path)
        
        # 绘制最短路径
        ax.plot(path_x, path_y, '-', color=shortest_color, linewidth=3, 
                label=f'最短路径 (到出口{shortest_idx+1}, {path_lengths[shortest_idx]:.2f}米)')
        
        # 绘制路径起点和终点
        ax.plot(path_x[0], path_y[0], 'o', color=shortest_color, markersize=8, label='起点')
        ax.plot(path_x[-1], path_y[-1], 'x', color=shortest_color, markersize=8, label='终点')
        
        # 更新标题
        ax.set_title(f"最短路径规划完成：到出口{shortest_idx+1}，长度{path_lengths[shortest_idx]:.2f}米", fontsize=14)
        
        # 更新图例
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig('shortest_path.png', dpi=300)
        
        # 启动DWA寻路窗口
        subprocess.Popen([sys.executable, "pklinput_window.py"])
        
        # 展示小车沿最短路径移动的动画
        if not no_car_animation:
            animate_car_movement(map_builder, shortest_path, ax, fig)
        
        plt.show()  # 只在这里显示一次


def animate_car_movement(map_builder, path, ax, fig):
    """
    动画展示小车沿着规划路径移动
    
    参数:
    map_builder: MapBuilder实例
    path: 路径点列表 [(x, y), ...]，栅格坐标
    ax: 图形坐标轴
    fig: 图形对象
    """
    if not path:
        print("没有有效路径，无法展示小车移动")
        return
    
    # 获取全局小车动画速度
    global car_animation_speed, use_dwa
    
    # 转换为世界坐标
    world_path = []
    for ix, iy in path:
        x = map_builder.origin_x + (ix + 0.5) * map_builder.actual_grid_size
        y = map_builder.origin_y + (iy + 0.5) * map_builder.actual_grid_size
        world_path.append((x, y))
    
    if use_dwa:
        # 使用DWA控制器跟踪路径
        print("使用DWA控制器跟踪路径...")
        
        try:
            # 采样路径点，减少路径点数量
            if len(world_path) > 50:
                sample_step = len(world_path) // 50
                sampled_path = [world_path[i] for i in range(0, len(world_path), sample_step)]
                # 确保终点被包含
                if world_path[-1] not in sampled_path:
                    sampled_path.append(world_path[-1])
                world_path = sampled_path
            
            print(f"路径点数量: {len(world_path)}")
            
            # 创建DWA控制器
            dwa = DWAController()
            
            # 设置机器人半径
            dwa.config.robot_radius = map_builder.car_radius
            print(f"机器人半径: {dwa.config.robot_radius}")
            
            # 如果有障碍物点云，转换为DWA控制器需要的格式
            obstacles = []
            if hasattr(map_builder, 'all_radar_points') and map_builder.all_radar_points:
                # 采样点云，避免过多点导致计算负担
                sample_step = max(1, len(map_builder.all_radar_points) // 1000)
                sampled_points = map_builder.all_radar_points[::sample_step]
                obstacles = [(point[0], point[1]) for point in sampled_points]  # 确保格式为(x, y)元组列表
                print(f"障碍物点数量: {len(obstacles)}")
            
            # 使用DWA控制器跟踪路径
            print("开始DWA路径跟踪...")
            dwa.follow_path(world_path, obstacles, ax, fig, car_animation_speed)
            print("DWA路径跟踪完成")
        except Exception as e:
            print(f"DWA路径跟踪出错: {e}")
            import traceback
            traceback.print_exc()
    else:
        # 原始的路径跟踪方法
        # 创建小车对象（使用圆形表示）
        car_radius = map_builder.car_radius
        car = Circle((world_path[0][0], world_path[0][1]), car_radius, color='blue', alpha=0.7)
        ax.add_patch(car)
        
        # 添加小车前进方向指示（箭头）
        direction_length = car_radius * 1.5
        if len(world_path) > 1:
            dx = world_path[1][0] - world_path[0][0]
            dy = world_path[1][1] - world_path[0][1]
            angle = math.atan2(dy, dx)
            arrow_dx = direction_length * math.cos(angle)
            arrow_dy = direction_length * math.sin(angle)
            arrow = ax.arrow(world_path[0][0], world_path[0][1], arrow_dx, arrow_dy, 
                            head_width=car_radius*0.5, head_length=car_radius*0.8, 
                            fc='red', ec='red', zorder=3)
        else:
            arrow = None
        
        # 添加路径跟踪线
        path_trace, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.6)
        trace_x, trace_y = [], []
        
        # 更新标题
        ax.set_title("小车正在沿路径移动...", fontsize=14)
        
        # 动画展示小车移动
        for i in range(len(world_path)):
            # 更新小车位置
            car.center = (world_path[i][0], world_path[i][1])
            
            # 更新路径跟踪线
            trace_x.append(world_path[i][0])
            trace_y.append(world_path[i][1])
            path_trace.set_data(trace_x, trace_y)
            
            # 更新小车前进方向
            if arrow is not None:
                arrow.remove()
            
            if i < len(world_path) - 1:
                dx = world_path[i+1][0] - world_path[i][0]
                dy = world_path[i+1][1] - world_path[i][1]
                angle = math.atan2(dy, dx)
                arrow_dx = direction_length * math.cos(angle)
                arrow_dy = direction_length * math.sin(angle)
                arrow = ax.arrow(world_path[i][0], world_path[i][1], arrow_dx, arrow_dy, 
                               head_width=car_radius*0.5, head_length=car_radius*0.8, 
                               fc='red', ec='red', zorder=3)
            
            # 更新进度
            progress = (i + 1) / len(world_path) * 100
            ax.set_title(f"小车正在沿路径移动... {progress:.1f}%", fontsize=14)
            
            # 刷新图形
            fig.canvas.draw()
            plt.pause(car_animation_speed)  # 使用小车动画速度
        
        # 移动完成
        ax.set_title("小车已到达目标位置！", fontsize=14)
        fig.canvas.draw()
        plt.pause(car_animation_speed * 5)  # 暂停一段时间，显示完成信息

if __name__ == "__main__":
    # 初始化全局变量
    show_animation = True  # 是否显示动画
    animation_speed = 0.01  # 动画速度（暂停时间，越小越快）
    car_animation_speed = 0.05  # 小车动画速度
    use_dwa = False  # 是否使用DWA控制器
    
    main() 