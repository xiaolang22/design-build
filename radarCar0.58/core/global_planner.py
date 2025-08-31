import numpy as np
from PathPlanning.AStar.a_star import AStarPlanner
from common.utils import world_to_grid, grid_to_world, RES, GRID_SIZE, ORIGIN_X, ORIGIN_Y

def occgrid_to_obstacles(occ_grid):
    """
    将占据栅格转为障碍物点坐标列表（世界坐标）
    :param occ_grid: -1:未知, 0:自由, 1:障碍
    :return: ox, oy (障碍物x/y坐标列表，单位:米)
    """
    iy, ix = np.nonzero(occ_grid == 1)
    ox = ORIGIN_X + (ix + 0.5) * RES
    oy = ORIGIN_Y + (iy + 0.5) * RES
    return ox.tolist(), oy.tolist()

def plan(start_xy, goal_xy, occ_grid):
    """
    用AStarPlanner做全局路径规划，机器人视为质点
    :param start_xy: 起点世界坐标(x, y)
    :param goal_xy: 终点世界坐标(x, y)
    :param occ_grid: 占据栅格
    :return: 路径[(x, y), ...]，若无路返回空列表
    """
    ox, oy = occgrid_to_obstacles(occ_grid)
    a_star = AStarPlanner(ox, oy, RES, rr=0.0)
    rx, ry = a_star.planning(start_xy[0], start_xy[1], goal_xy[0], goal_xy[1])
    if len(rx) == 0:
        return []
    return list(zip(rx, ry))

if __name__ == "__main__":
    occ = np.zeros((20, 20), dtype=np.int8)
    occ[10, 5:15] = 1  # 障碍
    start = (0.0, 0.0)
    goal = (0.5, 0.5)
    path = plan(start, goal, occ)
    print(f"A*路径: {path}") 