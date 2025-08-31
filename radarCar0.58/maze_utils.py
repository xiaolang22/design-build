import matplotlib.pyplot as plt
import json
import os

def load_edges(filename):
    """
    从文件加载迷宫边缘线段数据，支持txt和json格式。
    txt: 每行格式为 x1 y1 x2 y2
    json: {"segments": [{"start": [x1, y1], "end": [x2, y2]}, ...]}
    返回：[(x1, y1, x2, y2), ...]
    """
    _, ext = os.path.splitext(filename)
    if ext.lower() == '.json':
        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
        edges = []
        for seg in data.get('segments', []):
            x1, y1 = seg['start']
            x2, y2 = seg['end']
            edges.append((x1, y1, x2, y2))
        return edges
    else:
        edges = []
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 4:
                    continue  # 跳过格式不对的行
                x1, y1, x2, y2 = map(float, parts)
                edges.append((x1, y1, x2, y2))
        return edges


def draw_maze(edges, ax=None):
    """
    用matplotlib绘制迷宫边缘线段
    返回: matplotlib的Axes对象
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))
    
    for x1, y1, x2, y2 in edges:
        ax.plot([x1, x2], [y1, y2], 'k-', linewidth=2)
        
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Maze, Robot, and Lidar Scan')
    ax.grid(True, linestyle='--', alpha=0.6)
    return ax
