import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arrow
import math

class Config:
    """DWA算法的配置参数"""
    def __init__(self):
        # 机器人参数
        self.max_speed = 0.5  # 最大速度 [m/s]
        self.min_speed = 0.0  # 最小速度 [m/s]
        self.max_yaw_rate = 50.0 * math.pi / 180.0  # 最大角速度 [rad/s]
        self.max_accel = 0.5  # 最大加速度 [m/s^2]
        self.max_delta_yaw_rate = 50.0 * math.pi / 180.0  # 最大角加速度 [rad/s^2]
        self.v_resolution = 0.05  # 速度分辨率 [m/s]，增大以减少计算量
        self.yaw_rate_resolution = 1.0 * math.pi / 180.0  # 角速度分辨率 [rad/s]，增大以减少计算量
        self.dt = 0.1  # 时间步长 [s]
        self.predict_time = 1.0  # 前向预测时间 [s]
        self.to_goal_cost_gain = 1.0  # 目标权重增益
        self.speed_cost_gain = 0.5  # 速度权重增益
        self.obstacle_cost_gain = 1.0  # 障碍物权重增益
        self.stuck_check_limit = 5  # 判断为卡住的次数限制

        # 路径跟踪参数
        self.robot_radius = 0.3  # 机器人半径 [m]
        self.goal_dist_threshold = 0.3  # 目标距离阈值 [m]
        self.waypoint_dist_threshold = 0.5  # 路径点距离阈值 [m]


class DWAController:
    """Dynamic Window Approach控制器"""
    
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
    
    def calculate_obstacle_cost(self, x_traj, y_traj, obstacles):
        """
        计算轨迹的障碍物代价
        
        Args:
            x_traj: x轨迹点列表
            y_traj: y轨迹点列表
            obstacles: 障碍物列表 [(x, y), ...]
        
        Returns:
            cost: 障碍物代价，越大越危险，无限大表示碰撞
        """
        # 如果没有障碍物，返回0
        if not obstacles:
            return 0.0
        
        min_dist = float('inf')
        
        for x, y in zip(x_traj, y_traj):
            for ox, oy in obstacles:
                dx = x - ox
                dy = y - oy
                dist = math.hypot(dx, dy)
                
                if dist <= self.config.robot_radius:
                    return float('inf')  # 碰撞
                
                if dist < min_dist:
                    min_dist = dist
        
        # 障碍物代价：越近越大
        if min_dist == float('inf'):
            return 0.0  # 没有障碍物
        else:
            return 1.0 / min_dist  # 障碍物代价
    
    def calculate_to_goal_cost(self, x_traj, y_traj, goal_x, goal_y):
        """
        计算轨迹到目标点的代价
        
        Args:
            x_traj: x轨迹点列表
            y_traj: y轨迹点列表
            goal_x: 目标点x坐标
            goal_y: 目标点y坐标
        
        Returns:
            cost: 到目标点的代价，越小越好
        """
        # 计算轨迹终点到目标点的距离
        dx = goal_x - x_traj[-1]
        dy = goal_y - y_traj[-1]
        goal_dist = math.hypot(dx, dy)
        
        # 计算角度差
        goal_angle = math.atan2(dy, dx)
        angle_diff = abs(self.normalize_angle(goal_angle - math.atan2(y_traj[-1] - y_traj[0], x_traj[-1] - x_traj[0])))
        
        # 距离代价 + 角度代价
        cost = goal_dist + angle_diff
        
        return cost
    
    def predict_trajectory(self, v, omega):
        """
        预测给定速度和角速度下的轨迹
        
        Args:
            v: 线速度 [m/s]
            omega: 角速度 [rad/s]
        
        Returns:
            x_traj: x轨迹点列表
            y_traj: y轨迹点列表
            theta_traj: 朝向轨迹点列表
        """
        x_traj = [self.x]
        y_traj = [self.y]
        theta_traj = [self.theta]
        
        time = 0.0
        while time <= self.config.predict_time:
            # 更新位置和朝向
            x = x_traj[-1] + v * math.cos(theta_traj[-1]) * self.config.dt
            y = y_traj[-1] + v * math.sin(theta_traj[-1]) * self.config.dt
            theta = theta_traj[-1] + omega * self.config.dt
            
            # 添加到轨迹
            x_traj.append(x)
            y_traj.append(y)
            theta_traj.append(theta)
            
            time += self.config.dt
        
        return x_traj, y_traj, theta_traj
    
    def calculate_velocity_window(self):
        """
        计算动态窗口
        
        Returns:
            v_min: 最小线速度 [m/s]
            v_max: 最大线速度 [m/s]
            omega_min: 最小角速度 [rad/s]
            omega_max: 最大角速度 [rad/s]
        """
        # 基于机器人动力学约束的窗口
        v_min = max(self.config.min_speed, self.v - self.config.max_accel * self.config.dt)
        v_max = min(self.config.max_speed, self.v + self.config.max_accel * self.config.dt)
        omega_min = max(-self.config.max_yaw_rate, self.omega - self.config.max_delta_yaw_rate * self.config.dt)
        omega_max = min(self.config.max_yaw_rate, self.omega + self.config.max_delta_yaw_rate * self.config.dt)
        
        return v_min, v_max, omega_min, omega_max
    
    def dwa_control(self, goal_x, goal_y, obstacles=None):
        """
        DWA控制算法
        
        Args:
            goal_x: 目标点x坐标
            goal_y: 目标点y坐标
            obstacles: 障碍物列表 [(x, y), ...]
        
        Returns:
            v: 最优线速度 [m/s]
            omega: 最优角速度 [rad/s]
        """
        if obstacles is None:
            obstacles = []
        
        # 计算动态窗口
        v_min, v_max, omega_min, omega_max = self.calculate_velocity_window()
        
        best_cost = float('inf')
        best_v = 0.0
        best_omega = 0.0
        best_trajectory = None
        
        # 穷举所有可能的速度组合
        for v in np.arange(v_min, v_max, self.config.v_resolution):
            for omega in np.arange(omega_min, omega_max, self.config.yaw_rate_resolution):
                # 预测轨迹
                x_traj, y_traj, theta_traj = self.predict_trajectory(v, omega)
                
                # 计算各项代价
                obstacle_cost = self.calculate_obstacle_cost(x_traj, y_traj, obstacles)
                to_goal_cost = self.calculate_to_goal_cost(x_traj, y_traj, goal_x, goal_y)
                speed_cost = self.config.max_speed - v  # 速度代价：希望速度尽可能大
                
                # 如果会碰撞，跳过
                if obstacle_cost == float('inf'):
                    continue
                
                # 总代价 = 各项代价的加权和
                final_cost = (
                    self.config.to_goal_cost_gain * to_goal_cost +
                    self.config.speed_cost_gain * speed_cost +
                    self.config.obstacle_cost_gain * obstacle_cost
                )
                
                # 更新最优解
                if final_cost < best_cost:
                    best_cost = final_cost
                    best_v = v
                    best_omega = omega
                    best_trajectory = (x_traj, y_traj, theta_traj)
        
        # 如果没有找到有效轨迹，返回零速度
        if best_cost == float('inf'):
            return 0.0, 0.0, None
        
        return best_v, best_omega, best_trajectory
    
    def update(self, v, omega):
        """
        更新机器人状态
        
        Args:
            v: 线速度 [m/s]
            omega: 角速度 [rad/s]
        """
        self.x += v * math.cos(self.theta) * self.config.dt
        self.y += v * math.sin(self.theta) * self.config.dt
        self.theta += omega * self.config.dt
        self.theta = self.normalize_angle(self.theta)
        self.v = v
        self.omega = omega
    
    def normalize_angle(self, angle):
        """
        将角度归一化到[-pi, pi]
        
        Args:
            angle: 角度 [rad]
        
        Returns:
            normalized_angle: 归一化后的角度 [rad]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        
        while angle < -np.pi:
            angle += 2.0 * np.pi
        
        return angle
    
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
        
        # 路径跟踪循环
        goal_reached = False
        iter_count = 0
        max_iters = 1000  # 最大迭代次数，防止无限循环
        
        while not goal_reached and iter_count < max_iters:
            # 获取当前目标点
            goal_x, goal_y = path[self.current_waypoint_idx]
            
            # 计算到目标点的距离
            dx = goal_x - self.x
            dy = goal_y - self.y
            dist_to_goal = math.hypot(dx, dy)
            
            # 检查是否达到当前目标点
            if dist_to_goal < self.config.waypoint_dist_threshold:
                # 移动到下一个目标点
                self.current_waypoint_idx += 1
                
                # 检查是否到达终点
                if self.current_waypoint_idx >= len(path):
                    goal_reached = True
                    break
                
                # 更新目标点
                goal_x, goal_y = path[self.current_waypoint_idx]
            
            # 使用DWA计算控制输入
            v, omega, best_trajectory = self.dwa_control(goal_x, goal_y, obstacles)
            
            # 检查是否找到有效轨迹
            if best_trajectory is None:
                print("未找到有效轨迹，尝试原地旋转")
                v = 0.0
                omega = self.config.max_yaw_rate / 2.0
            
            # 更新机器人位置
            self.update(v, omega)
            
            # 检查是否卡住
            dx = goal_x - self.x
            dy = goal_y - self.y
            current_dist = math.hypot(dx, dy)
            
            if abs(current_dist - self.last_distance) < 0.01:
                self.stuck_count += 1
            else:
                self.stuck_count = 0
            
            self.last_distance = current_dist
            
            # 如果卡住，尝试随机移动
            if self.stuck_count > self.config.stuck_check_limit:
                print("检测到卡住，尝试随机移动")
                v = 0.0
                omega = np.random.uniform(-self.config.max_yaw_rate, self.config.max_yaw_rate)
                self.update(v, omega)
                self.stuck_count = 0
            
            # 可视化
            if ax is not None and fig is not None:
                # 更新机器人位置
                robot_circle.center = (self.x, self.y)
                
                # 更新方向箭头
                arrow.remove()
                arrow_dx = arrow_length * math.cos(self.theta)
                arrow_dy = arrow_length * math.sin(self.theta)
                arrow = ax.arrow(self.x, self.y, arrow_dx, arrow_dy, 
                                head_width=self.config.robot_radius*0.5, 
                                head_length=self.config.robot_radius*0.8, 
                                fc='red', ec='red', zorder=3)
                
                # 更新路径跟踪线
                trace_x.append(self.x)
                trace_y.append(self.y)
                path_trace.set_data(trace_x, trace_y)
                
                # 更新当前目标点
                target_point.set_data([goal_x], [goal_y])
                
                # 更新预测轨迹
                if best_trajectory and len(best_trajectory) >= 2:
                    x_traj, y_traj = best_trajectory[0], best_trajectory[1]
                    if len(x_traj) > 0 and len(y_traj) > 0:
                        pred_traj.set_data(x_traj, y_traj)
                else:
                    # 清空预测轨迹
                    pred_traj.set_data([], [])
                
                # 更新标题
                progress = (self.current_waypoint_idx) / len(path) * 100
                ax.set_title(f"DWA控制: 完成度 {progress:.1f}%", fontsize=14)
                
                # 刷新图形
                fig.canvas.draw()
                plt.pause(animation_speed)
            
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