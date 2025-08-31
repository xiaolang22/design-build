import numpy as np
from typing import List
from robot_utils import simulate_lidar_scan
from maze_utils import load_edges
from datetime import datetime
import random
import time

class RadarInterface:
    """
    雷达数据接口类，只负责获取和保存/加载雷达数据
    """
    def __init__(self, robot, edges):
        self.robot = robot
        self.edges = edges
        self.num_beams = 360

    def get_current_radar_data(self, log_file: str = None) -> List[float]:
        """
        获取当前位置的360条雷达数据，并可按要求附加写入数据文件
        Args:
        log_file (str): 可选，若指定则将本次数据附加写入该文件
        Returns:
        List[float]: 360个距离值，每个角度1度，0度指向正前方，顺时针增加
        """
        scan_data = simulate_lidar_scan(self.robot, self.edges, self.num_beams)
        distances = [0.0] * self.num_beams
        for i, (angle, distance) in enumerate(scan_data):
            relative_angle = (angle - self.robot.theta) % (2 * np.pi)
            relative_angle_deg = int(np.rad2deg(relative_angle)) % 360
            # 保持原始距离
            distances[relative_angle_deg] = distance

        # 附加写入数据文件
        if log_file is not None:
            timestamp = time.time()
            # 随机生成2~24列（23个随机数）
            random_cols = [f"{random.uniform(0, 1):.3f}" for _ in range(23)]
            # 写入时超过4m的距离填0
            write_distances = [d if d <= 4.0 else 0.0 for d in distances]
            # 拼接一行：时间戳, 随机23列, 360个距离
            row = [f"{timestamp:.3f}"] + random_cols + [f"{d:.3f}" for d in write_distances]
            with open(log_file, 'a', encoding='utf-8') as f:
                f.write(' '.join(row) + '\n')

        return distances

    @staticmethod
    def save_radar_data(data: List[float], filename: str = None) -> str:
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"radar_data_{timestamp}.dat"
        with open(filename, 'w', encoding='utf-8') as f:
            for d in data:
                f.write(f"{d}\n")
        return filename

    @staticmethod
    def load_radar_data(filename: str) -> List[float]:
        with open(filename, 'r', encoding='utf-8') as f:
            data = [float(line.strip()) for line in f if line.strip()]
        return data

    def get_min_distance_in_range(self, start_angle: int, end_angle: int) -> float:
        """
        获取指定角度范围内的最小有效雷达距离

        Args:
            start_angle (int): 起始角度（度），范围0~359，0度为车辆正前方，顺时针为正方向
            end_angle (int): 结束角度（度），范围0~359

        Returns:
            float: 指定角度范围内的最小有效距离（大于0的最小值），如果没有有效数据则返回0
        """
        # 获取当前360度雷达数据，每个元素为对应角度的距离
        radar_data = self.get_current_radar_data()

        # 角度归一化到0~359
        start_angle = start_angle % 360
        end_angle = end_angle % 360

        # 根据角度范围提取相关的距离数据
        if start_angle <= end_angle:
            # 情况1：范围不跨越0度（如30~60），直接取[start_angle, end_angle]
            relevant = [radar_data[a] for a in range(start_angle, end_angle + 1)]
        else:
            # 情况2：范围跨越0度（如350~10），分两段拼接
            relevant = [radar_data[a] for a in range(start_angle, 360)] + [radar_data[a] for a in range(0, end_angle + 1)]

        # 只考虑大于0的有效距离，取最小值，若无有效数据则返回0
        min_distance = min([d for d in relevant if d > 0], default=0)
        return min_distance

def create_radar_interface(robot, edges_file: str = 'maze_edges.json') -> RadarInterface:
    """
    创建雷达接口
    """
    edges = load_edges(edges_file)
    return RadarInterface(robot, edges)