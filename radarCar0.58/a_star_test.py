import json
import numpy as np
import matplotlib.pyplot as plt
from a_star import AStarPlanner
from grid_map_utils import visited_to_gridmap, gridmap_to_image

def load_maze_edges(filename):
    """
    从JSON文件加载迷宫边缘数据
    """
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def maze_edges_to_obstacles(maze_data, grid_size=0.1):
    """
    将迷宫边缘转换为障碍点列表
    """
    ox, oy = [], []
    
    # 从每个线段生成密集的障碍点
    for segment in maze_data['segments']:
        start = segment['start']
        end = segment['end']
        
        # 计算线段长度
        length = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        
        # 根据线段长度和网格大小计算采样点数量
        num_points = max(2, int(length / grid_size) + 1)
        
        # 生成线段上的点
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            ox.append(x)
            oy.append(y)
    
    return ox, oy

def test_a_star_with_maze():
    """
    使用迷宫数据测试A*算法：起点(3,0) -> 终点(12,15)
    """
    print("开始A*算法迷宫测试...")
    
    # 加载迷宫数据
    maze_data = load_maze_edges('maze_edges.json')
    print(f"加载了 {len(maze_data['segments'])} 个迷宫边缘段")
    
    # 将迷宫边缘转换为障碍点
    grid_size = 0.1  # 网格大小
    ox, oy = maze_edges_to_obstacles(maze_data, grid_size)
    print(f"生成了 {len(ox)} 个障碍点")
    
    # 设置起点和终点
    sx, sy = 3, 0  # 起点
    gx, gy = 12, 15  # 终点
    
    # 机器人半径
    robot_radius = 0.2
    
    print(f"起点: ({sx}, {sy})")
    print(f"终点: ({gx}, {gy})")
    print(f"网格大小: {grid_size}")
    print(f"机器人半径: {robot_radius}")
    
    # 创建A*规划器
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    
    # 执行路径规划
    print("开始路径规划...")
    rx, ry = a_star.planning(sx, sy, gx, gy)
    
    if rx and ry:
        print(f"找到路径，包含 {len(rx)} 个点")
        
        # 计算路径长度
        path_length = sum(np.sqrt((rx[i+1]-rx[i])**2 + (ry[i+1]-ry[i])**2) 
                         for i in range(len(rx)-1))
        print(f"路径长度: {path_length:.2f}")
        
        # 可视化结果
        plt.figure(figsize=(12, 10))
        
        # 绘制障碍物
        plt.plot(ox, oy, ".k", markersize=2, label='障碍物')
        
        # 绘制起点和终点
        plt.plot(sx, sy, "og", markersize=10, label='起点 (3,0)')
        plt.plot(gx, gy, "xb", markersize=10, label='终点 (12,15)')
        
        # 绘制路径
        plt.plot(rx, ry, "-r", linewidth=3, label='A*路径')
        
        # 绘制迷宫边缘
        for segment in maze_data['segments']:
            start = segment['start']
            end = segment['end']
            plt.plot([start[0], end[0]], [start[1], end[1]], 'b-', linewidth=1, alpha=0.5)
        
        plt.grid(True)
        plt.axis("equal")
        plt.title("A*算法迷宫路径规划测试\n起点(3,0) -> 终点(12,15)")
        plt.legend()
        plt.xlabel("X坐标")
        plt.ylabel("Y坐标")
        
        # 保存结果
        plt.savefig('a_star_maze_result.png', dpi=300, bbox_inches='tight')
        print("结果已保存为 a_star_maze_result.png")
        
        plt.show()
        
        return True
    else:
        print("未找到有效路径")
        return False

if __name__ == '__main__':
    test_a_star_with_maze()
    print("\n测试完成！") 
