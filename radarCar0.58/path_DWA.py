import pickle
import numpy as np
import heapq
from collections import deque
import math
# 移除pyqtgraph和PyQt5导入
import matplotlib.pyplot as plt
import sys

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
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        goal_dis = math.sqrt(dx ** 2 + dy ** 2)
        return self.config.to_goal_cost_gain * goal_dis

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

        # 增加采样点数
        for v in np.linspace(vr[0], vr[1], 8):
            for w in np.linspace(vr[2], vr[3], 8):
                trajectory = self.calc_trajectory(x, v, w)
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.calc_obstacle_cost(trajectory, ob)
                final_cost = (
                    to_goal_cost * self.config.to_goal_cost_gain +
                    speed_cost +
                    ob_cost * self.config.obstacle_cost_gain
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

    def plan(self):
        # 全局路径规划
        global_path = self.astar_global_path()
        if not global_path:
            print("No global path found!")
            return None

        # 对全局路径进行插值，使相邻点距离不超过max_distance
        global_path = self.interpolate_waypoints(global_path, max_distance=2.5)

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
                if dist_to_goal <= self.config.robot_radius * 1.5:
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
                    plt.pause(0.001)

                stuck_count += 1
                if stuck_count > 500:
                    print(f"Stuck at {x[:2]}, moving to next waypoint")
                    break

        plt.show()
        return trajectory

def main():
    # 读取点云和起点数据
    try:
        with open('explore_result_1.pkl', 'rb') as f:
            data = pickle.load(f)
            point_cloud = data['point_cloud']
            start = data['start']
            exits = data['exits']
    except FileNotFoundError:
        print("Error: File 'explore_result_1.pkl' not found.")
        print("Creating dummy data for testing...")
        # 创建测试数据
        point_cloud = [(i, j) for i in range(0, 20, 2) for j in range(0, 20, 2)]
        # 移除中心区域的点，创建一个通道
        point_cloud = [p for p in point_cloud if not (5 < p[0] < 15 and 5 < p[1] < 15)]
        start = (2, 2)
        exits = [(18, 18)]

    planner = GlobalLocalPathPlanner(point_cloud, start, exits)
    trajectory = planner.plan()

if __name__ == '__main__':
    main()