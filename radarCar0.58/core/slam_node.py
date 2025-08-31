import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import XVLidar
from common.utils import RES, GRID_SIZE

class SLAMNode:
    def __init__(self, grid_size=GRID_SIZE, res=RES):
        """
        初始化SLAM节点，使用BreezySLAM
        :param grid_size: 地图格数
        :param res: 地图分辨率（米/格）
        """
        self.grid_size = grid_size
        self.res = res
        self.map_size_meters = grid_size * res
        self.laser = XVLidar()
        self.slam = RMHC_SLAM(self.laser, grid_size, self.map_size_meters)
        self.mapbytes = bytearray(grid_size * grid_size)
        self.pose = np.array([0.0, 0.0, 0.0])  # (x, y, theta)

    def update(self, scan, pose_change=(0, 0, 1)):
        """
        输入激光scan，更新SLAM，输出占据栅格和机器人位姿
        :param scan: 激光雷达距离列表（单位mm，长度360）
        :param pose_change: 里程计变化(前进mm, 旋转deg, 时间s)，无里程计可设(0,0,1)
        :return: occ_grid, pose
        """
        self.slam.update(scan, pose_change)
        self.slam.getmap(self.mapbytes)
        occ_grid = np.array(self.mapbytes, dtype=np.uint8).reshape((self.grid_size, self.grid_size))
        # BreezySLAM: 0=free, 255=occupied, 127=unknown
        occ_grid = np.where(occ_grid == 255, 1, np.where(occ_grid == 0, 0, -1)).astype(np.int8)
        self.pose = np.array(self.slam.getpos())
        return occ_grid, self.pose.copy()

if __name__ == "__main__":
    # 最小自测，模拟一帧全量激光
    scan = [1000] * 360  # 1米全量
    slam = SLAMNode()
    occ, pose = slam.update(scan)
    print(f"位姿: {pose}")
    print(f"地图已知区数量: {(occ != -1).sum()}") 