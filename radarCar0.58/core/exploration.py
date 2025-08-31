import time
from core.slam_node import SLAMNode
from core.frontier import detect_frontiers
from core.global_planner import plan as global_plan
from core.local_planner import VehicleController, DWAController, local_plan
from robot_utils import Robot
import numpy as np
import sys
sys.path.append('.')
from radar_interface import RadarInterface, load_edges
import matplotlib.pyplot as plt
from maze_utils import draw_maze

def pick_reachable_frontier(frontiers):
    """
    选择最近的frontier作为目标
    :param frontiers: [(x, y), ...]
    :return: (x, y) 目标frontier
    """
    if not frontiers:
        return None
    # 这里只取第一个，实际可按距离排序
    return frontiers[0]

class MazeSLAMVisualizer:
    def __init__(self, edges, grid_size, map_size_meters):
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        draw_maze(edges, self.ax)
        self.im = self.ax.imshow(np.zeros((grid_size, grid_size)), cmap='gray', alpha=0.5, origin='lower', extent=[0, map_size_meters, 0, map_size_meters])
        self.trajectory, = self.ax.plot([], [], 'r-', linewidth=2, label='Trajectory')
        self.robot_dot, = self.ax.plot([], [], 'ro', label='Robot', markersize=12)
        self.traj_x, self.traj_y = [], []
        self.ax.legend()
        self.fig.tight_layout()

    def update(self, occ_grid, robot_pose):
        self.im.set_data(occ_grid)
        x, y = robot_pose[0], robot_pose[1]
        self.traj_x.append(x)
        self.traj_y.append(y)
        self.trajectory.set_data(self.traj_x, self.traj_y)
        self.robot_dot.set_data([x], [y])
        self.robot_dot.set_markersize(12)
        self.fig.canvas.draw_idle()
        plt.pause(0.01)

def exploration_main(mode="step", max_iter=10000, max_time=600):
    """
    主循环，支持step/dwa两种模式
    :param mode: "step" 或 "dwa"
    :param max_iter: 最大迭代次数
    :param max_time: 最大仿真时间（秒）
    """
    slam = SLAMNode()
    robot = Robot(x=3, y=0, theta=np.pi/2)
    edges = load_edges('maze_edges.json')
    radar = RadarInterface(robot, edges)
    vehicle = VehicleController()
    dwa_controller = DWAController()
    visualizer = MazeSLAMVisualizer(edges, slam.grid_size, slam.map_size_meters)
    t0 = time.time()
    iter_count = 0
    try:
        while iter_count < max_iter and (time.time() - t0) < max_time:
            scan = radar.get_current_radar_data()
            occ, pose = slam.update(scan)
            visualizer.update(occ, pose)
            fronts = detect_frontiers(occ)
            if not fronts:
                print("无frontier，探索结束！")
                break
            goal = pick_reachable_frontier(fronts)
            if goal is None:
                print("无可达frontier，探索结束！")
                break
            if mode == "step":
                path = global_plan(tuple(robot.xy), goal, occ)
                if not path or len(path) < 2:
                    print("无可达路径，重新探索！")
                    continue
                for nxt in path[1:]:
                    dir_, step = local_plan("step", robot, nxt)
                    if vehicle.move_vehicle(step, dir_):
                        print("预测撞壁，重新规划！")
                        break
                    robot.step_move(dir_, step)
                    visualizer.update(occ, robot.pose)
                    time.sleep(0.01)
            else:  # DWA模式
                vx, vy, w = local_plan("dwa", None, goal, occ, robot, dwa_controller=dwa_controller)
                dwa_controller.update_robot(robot, 0.1)
                time.sleep(0.01)
            iter_count += 1
    except KeyboardInterrupt:
        print("用户中断，清理退出...")
    print("探索完成！")
    plt.show()  # 探索结束后阻塞窗口，方便观察

if __name__ == "__main__":
    # 最小自测
    exploration_main(mode="step", max_iter=10, max_time=5) 