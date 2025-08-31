import numpy as np
from typing import Tuple, List
from robot_utils import Robot
from radar_interface import RadarInterface

class VehicleController:
    """
    车辆控制器类，实现预测性碰撞检测
    """
    
    def __init__(self, robot: Robot, radar_interface: RadarInterface):
        """
        初始化车辆控制器
        
        Args:
            robot: 机器人对象，包含当前位置和朝向
            radar_interface: 雷达接口对象
        """
        self.robot = robot
        self.radar_interface = radar_interface
  
        self.safety_margin = 0.1   # 安全裕量（米）
        
        # 方向定义和对应的雷达角度范围
        self.direction_ranges = {
            0: (-15, 15),    # 前：0度为中心，范围 -15度到15度
            1: (75, 105),    # 右：90度为中心，范围75度到105度
            2: (165, 195),   # 后：180度为中心，范围165度到195度
            3: (255, 285)    # 左：270度为中心，范围255度到285度
        }
    
    def move_vehicle(self, distance: float, direction: int) -> bool:
        """
        控制车辆移动的接口（预测性碰撞检测）
        
        Args:
            distance: 移动距离（米），double类型
            direction: 移动方向，int类型
                     0: 前
                     1: 右
                     2: 后
                     3: 左
        
        Returns:
            bool: 是否碰壁
                 True: 预测会碰壁，移动失败
                 False: 预测安全，移动成功
        """
        if not isinstance(distance, (int, float)) or distance <= 0:
            raise ValueError("距离必须是正数")
        
        if direction not in [0, 1, 2, 3]:
            raise ValueError("方向必须是 0(前), 1(右), 2(后), 或 3(左)")
        
        # 1. 预测性碰撞检测
        if self._predict_collision(distance, direction):
            return True  # 预测会碰壁，不执行移动
        
        # 2. 执行移动
        self._execute_movement(distance, direction)
        return False  # 移动成功
    
    def _predict_collision(self, distance: float, direction: int) -> bool:
        """
        预测性碰撞检测（车辆视为质点，不考虑车体半径）
        
        Args:
            distance: 移动距离
            direction: 移动方向
            
        Returns:
            bool: True表示预测会碰撞，False表示预测安全
        """
        # 获取方向对应的雷达角度范围
        start_angle, end_angle = self.direction_ranges[direction]
        
        # 获取该角度范围内的最小雷达距离值
        min_distance = self.radar_interface.get_min_distance_in_range(start_angle, end_angle)
        
        # 如果最小距离为0，说明没有有效数据，保守起见认为会碰撞
        if min_distance <= 0:
            return True
        
        # 只考虑质点：安全距离 = 移动距离 + 安全裕量
        safe_distance = distance + self.safety_margin
        
        # 如果最小距离小于安全距离，预测会碰撞
        return min_distance < safe_distance
    
    def _execute_movement(self, distance: float, direction: int):
        """
        执行车辆移动（以机器人当前朝向为基准）
        
        Args:
            distance: 移动距离
            direction: 移动方向
        """
        # 以当前朝向为基准调整方向
        base_theta = self.robot.theta
        direction_offsets = {
            0: 0,                # 前
            1: -np.pi / 2,       # 右（顺时针）
            2: np.pi,            # 后
            3: np.pi / 2         # 左（逆时针）
        }
        move_theta = base_theta + direction_offsets[direction]
        new_x = self.robot.x + distance * np.cos(move_theta)
        new_y = self.robot.y + distance * np.sin(move_theta)
        self.robot.x = new_x
        self.robot.y = new_y
    

# 便捷函数
def create_vehicle_controller(start_x: float, start_y: float, start_theta: float, 
                             edges_file: str = 'maze_edges.json') -> VehicleController:
    """
    创建车辆控制器的便捷函数
    
    Args:
        start_x, start_y: 起始位置
        start_theta: 起始朝向（弧度）
        edges_file: 迷宫边缘文件路径
    
    Returns:
        VehicleController: 车辆控制器实例
    """
    from radar_interface import create_radar_interface
    
    robot = Robot(start_x, start_y, start_theta)
    radar_interface = create_radar_interface(robot, edges_file)
    return VehicleController(robot, radar_interface) 