import numpy as np

# 地图参数
RES = 0.05  # 每格0.05米
GRID_SIZE = 1000  # 原来是800，增加到1000以适配更大的显示范围
ORIGIN_X = -GRID_SIZE * RES / 2
ORIGIN_Y = -GRID_SIZE * RES / 2

def world_to_grid(x, y):
    """
    世界坐标(x, y)转为栅格坐标(ix, iy)
    :param x: 世界坐标x（米）
    :param y: 世界坐标y（米）
    :return: (ix, iy) 栅格索引
    """
    ix = int((x - ORIGIN_X) / RES)
    iy = int((y - ORIGIN_Y) / RES)
    return ix, iy

def grid_to_world(ix, iy):
    """
    栅格坐标(ix, iy)转为世界坐标(x, y)
    :param ix: 栅格x索引
    :param iy: 栅格y索引
    :return: (x, y) 世界坐标（米）
    """
    x = ORIGIN_X + (ix + 0.5) * RES
    y = ORIGIN_Y + (iy + 0.5) * RES
    return x, y

if __name__ == "__main__":
    # 最小自测
    x, y = 0.0, 0.0
    ix, iy = world_to_grid(x, y)
    x2, y2 = grid_to_world(ix, iy)
    print(f"世界坐标: ({x}, {y}) -> 栅格: ({ix}, {iy}) -> 世界坐标: ({x2}, {y2})") 