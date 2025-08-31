# grid_map_util.py
import numpy as np
import matplotlib.pyplot as plt

def visited_to_gridmap(visited, grid_size=1.0, x_min=0.0, x_max=30.0, y_min=0.0, y_max=30.0):
    """
    将visited点集栅格化为二维numpy数组
    返回：grid_map, ox, oy
    grid_map: 0=可通行, 1=障碍物
    ox, oy: 障碍物点列表（供A*使用）
    """
    # 计算网格尺寸
    width = int(np.ceil((x_max - x_min) / grid_size))
    height = int(np.ceil((y_max - y_min) / grid_size))
    
    # 初始化全可通行地图 (0=可通行)
    grid_map = np.zeros((width, height), dtype=np.int8)
    
    # 标记障碍物
    ox, oy = [], []
    for x, y in visited:
        ix = int(np.floor((x - x_min) / grid_size))
        iy = int(np.floor((y - y_min) / grid_size))
        
        if 0 <= ix < width and 0 <= iy < height:
            grid_map[ix, iy] = 1  # 标记为障碍物
            ox.append(x_min + ix * grid_size)
            oy.append(y_min + iy * grid_size)
    
    return grid_map, ox, oy

def gridmap_to_image(grid_map):
    """
    将栅格地图转换为图像格式
    0=可通行(白色), 1=障碍物(黑色)
    """
    # 反转颜色：0→255(白色), 1→0(黑色)
    image = 255 * (1 - grid_map.astype(np.uint8))
    return image

def save_gridmap(grid_map, filename, grid_size=1.0):
    """
    保存栅格地图到文件
    """
    image = gridmap_to_image(grid_map)
    plt.imsave(filename, image, cmap='gray')
    print(f"地图已保存为 {filename}")