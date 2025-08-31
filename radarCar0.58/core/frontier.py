import numpy as np
from scipy.ndimage import label, center_of_mass
from common.utils import grid_to_world


def detect_frontiers(occ_grid, min_size=3):
    """
    检测frontier区域，返回所有frontier质心（世界坐标，单位：米）
    :param occ_grid: 占据栅格（-1:未知, 0:自由, 1:障碍）
    :param min_size: 连通块最小像素数
    :return: frontiers [(x, y), ...] 质心坐标
    """
    # 1. frontier点：当前为0且八邻域至少1个为-1
    free = (occ_grid == 0)
    unknown = (occ_grid == -1)
    frontier_mask = np.zeros_like(occ_grid, dtype=bool)
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            shifted = np.roll(np.roll(unknown, dx, axis=0), dy, axis=1)
            frontier_mask |= free & shifted
    # 2. 连通区域筛选
    structure = np.ones((3, 3), dtype=bool)
    labeled, num = label(frontier_mask, structure=structure)
    frontiers = []
    for i in range(1, num + 1):
        inds = np.argwhere(labeled == i)
        if inds.shape[0] >= min_size:
            cy, cx = center_of_mass(labeled == i)
            x, y = grid_to_world(int(cx), int(cy))
            frontiers.append((x, y))
    return frontiers


if __name__ == "__main__":
    # 最小自测
    occ = -np.ones((20, 20), dtype=np.int8)
    occ[8:12, 8:12] = 0  # 自由区
    occ[7, 8:12] = 1     # 障碍
    fronts = detect_frontiers(occ, min_size=1)
    print(f"检测到frontier质心: {fronts}") 