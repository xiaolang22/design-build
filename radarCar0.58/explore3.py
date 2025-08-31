import glob
import sys
import numpy as np
import matplotlib.pyplot as plt
from vehicle_control import create_vehicle_controller
from radar_interface import create_radar_interface
import matplotlib
import json
from common.utils import RES  # 导入栅格大小常量
from matplotlib.patches import Rectangle
import subprocess  # 添加subprocess模块用于调用其他Python脚本
import pickle
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体
matplotlib.rcParams['axes.unicode_minus'] = False   # 正常显示负号

# 全局车辆控制器实例 - 将在主函数中初始化
vehicle_controller = None
radar_interface = None

# 小车状态
car_pos = [0, 0]  # 初始化为默认值，稍后会更新
history_points = []  # 历史点云
move_distance = 0.1  # 步长
car_init_pos = car_pos.copy()  # 记录初始位置
isnext = False
# 新增：全局变量用于保存所有出口位置
exit_pos_list = []
# 新增：全局可见栅格掩码
visible_grid_mask = None
# 新增：保存所有雷达数据的列表 [((car_x, car_y), radar_data), ...]
all_radar_data = []
# 用于雷达数据去重的位置集合，保存已记录雷达数据的位置坐标（保留小数点后两位）
recorded_positions = set()

# 获取雷达数据
def get_radar():
    vehicle_controller.robot.theta = 0
    radar_data = radar_interface.get_current_radar_data()
    # 保存雷达数据到全局列表
    if radar_data is not None and len(radar_data) == 360:
        global all_radar_data, recorded_positions
        # 当前小车位置
        car_x = vehicle_controller.robot.x
        car_y = vehicle_controller.robot.y
        
        # 对坐标进行四舍五入，保留小数点后两位，用于去重
        rounded_pos = (round(car_x, 2), round(car_y, 2))
        
        # 如果该位置还没有记录过雷达数据，则添加
        if rounded_pos not in recorded_positions:
            current_pos = (car_x, car_y)  # 保存原始精度的坐标
            all_radar_data.append((current_pos, radar_data.copy()))  # 使用copy()避免引用问题
            recorded_positions.add(rounded_pos)
            print(f"记录位置 {rounded_pos} 的雷达数据，当前共 {len(all_radar_data)} 条")
    return radar_data

# 统一方向编号：0=上, 1=右, 2=下, 3=左
def move(direction, distance):
    """
    direction: 0=上，1=右，2=下，3=左
    distance: 移动距离
    返回：False=未碰壁，True=碰壁
    """
    vc_direction = direction  # vehicle_control.py方向编号一致
    return vehicle_controller.move_vehicle(distance, vc_direction)

def move_global(direction, distance):
    """
    direction: 0=上，1=右，2=下，3=左（全局坐标系）
    直接修改robot.x, robot.y
    """
    dx_dy = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    dx, dy = dx_dy[direction]
    vehicle_controller.robot.x += dx * distance
    vehicle_controller.robot.y += dy * distance
    car_pos[0] = vehicle_controller.robot.x
    car_pos[1] = vehicle_controller.robot.y
    return False  # 不检测碰撞，始终返回未碰壁

# 全局变量：完整地图线段
map_segments = []

# 读取完整地图信息
def load_full_map(map_file_path):
    global map_segments
    try:
        with open(map_file_path, 'r') as f:
            map_data = json.load(f)
            if 'segments' in map_data:
                map_segments = map_data['segments']
                print(f"成功加载地图，包含 {len(map_segments)} 条墙体线段")
                return True
            else:
                print("地图文件格式不正确，缺少'segments'字段")
                return False
    except Exception as e:
        print(f"读取地图文件出错: {str(e)}")
        return False

# 绘制完整地图视图（仅墙体、小车位置和轨迹）
def draw_full_map(ax, car_pos):
    ax.cla()
    ax.set_title('完整地图视图')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    # 绘制所有墙体线段
    for segment in map_segments:
        start = segment['start']
        end = segment['end']
        ax.plot([start[0], end[0]], [start[1], end[1]], 'k-', linewidth=2)
    
    # 记录小车位置历史
    global car_positions_history
    if not hasattr(draw_full_map, 'car_positions_history'):
        draw_full_map.car_positions_history = []
    
    # 只有当位置变化超过阈值时才记录，避免重复点
    if not draw_full_map.car_positions_history or np.hypot(
            car_pos[0] - draw_full_map.car_positions_history[-1][0],
            car_pos[1] - draw_full_map.car_positions_history[-1][1]) > 0.1:
        draw_full_map.car_positions_history.append((car_pos[0], car_pos[1]))
    
    # 绘制小车轨迹
    if len(draw_full_map.car_positions_history) > 1:
        traj_x, traj_y = zip(*draw_full_map.car_positions_history)
        ax.plot(traj_x, traj_y, 'b-', alpha=0.7, linewidth=1, label='小车轨迹')
    
    # 绘制小车当前位置
    ax.plot(car_pos[0], car_pos[1], 'ro', markersize=8, label='当前位置')
    
    # 保持坐标轴比例一致，但使用固定的显示范围
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 25)
    
    # 如果轨迹不为空，显示图例
    if len(draw_full_map.car_positions_history) > 1:
        ax.legend(loc='upper right')

def update_plot(ax_explore, ax_full=None):
    global visible_grid_mask
    ax_explore.cla()
    # ====== 新增：栅格化底图（灰色为不可见） ======
    import matplotlib.colors as mcolors
    from common.utils import RES, GRID_SIZE, ORIGIN_X, ORIGIN_Y, world_to_grid
    from grid_map_utils import visited_to_gridmap
    # 设定栅格参数
    grid_size = RES
    x_min = ORIGIN_X
    y_min = ORIGIN_Y
    x_max = ORIGIN_X + GRID_SIZE * RES
    y_max = ORIGIN_Y + GRID_SIZE * RES
    width = int(np.ceil((x_max - x_min) / grid_size))
    height = int(np.ceil((y_max - y_min) / grid_size))
    # 不再在此处初始化visible_grid_mask
    # 统计所有历史点云
    all_points = history_points.copy()
    # 栅格化历史点云（障碍物）
    grid_map, _, _ = visited_to_gridmap(
        all_points,
        grid_size=grid_size,
        x_min=float(x_min),
        x_max=float(x_max),
        y_min=float(y_min),
        y_max=float(y_max)
    )
    # 本帧可见区域栅格
    visible_mask = np.zeros_like(grid_map, dtype=bool)
    radar = get_radar()
    ix0, iy0 = world_to_grid(car_pos[0], car_pos[1])
    for deg, dist in enumerate(radar):
        if dist is None or dist < 0.01:
            continue
        angle = np.deg2rad(deg)
        # 只连线到4m范围
        end_dist = min(dist, 4.0)
        x = car_pos[0] + end_dist * np.cos(angle)
        y = car_pos[1] + end_dist * np.sin(angle)
        ix1, iy1 = world_to_grid(x, y)
        n_points = int(max(abs(ix1 - ix0), abs(iy1 - iy0)) + 1)
        ix_line = np.linspace(ix0, ix1, n_points, dtype=int)
        iy_line = np.linspace(iy0, iy1, n_points, dtype=int)
        for iix, iiy in zip(ix_line, iy_line):
            if 0 <= iix < width and 0 <= iiy < height:
                visible_mask[iiy, iix] = True
                # print(f"可见点: ix={iix}, iy={iiy}, width={width}, height={height}")
    # 小车自身可见
    if 0 <= ix0 < width and 0 <= iy0 < height:
        visible_mask[iy0, ix0] = True
    # 累积可见区域
    visible_grid_mask |= visible_mask
    print(f'可见掩码累计白色格数: {np.sum(visible_grid_mask)}')
    # 构造底图：0=可见，1=不可见
    not_visible = ~visible_grid_mask
    # 画底图（白色为可见，灰色为不可见区域）
    cmap = mcolors.ListedColormap(['white', 'lightgray'])
    ax_explore.imshow(not_visible, cmap=cmap, origin='lower', extent=[x_min, x_max, y_min, y_max], alpha=0.7, zorder=0)
    # ====== 结束栅格化底图 ======
    # 绘制历史点云（黑色）
    if history_points:
        sampled_points = history_points[::10]  # 每十个点取一个
        if sampled_points:
            hx, hy = zip(*sampled_points)
            ax_explore.scatter(hx, hy, s=1, c='black', alpha=0.5, label='历史点云', zorder=2)
    # 绘制当前帧点云（只显示距离在4m以内的点）
    radar_points_x = []
    radar_points_y = []
    # 新增：雷达可视范围边界点
    visibility_boundary_x = []
    visibility_boundary_y = []
    for deg, dist in enumerate(radar):
        if dist is None or dist < 0.01 or dist > 4.0:
            continue
        angle = np.deg2rad(deg)
        x = car_pos[0] + dist * np.cos(angle)
        y = car_pos[1] + dist * np.sin(angle)
        radar_points_x.append(x)
        radar_points_y.append(y)
        history_points.append((x, y))
        # 记录可视范围边界点
        visibility_boundary_x.append(x)
        visibility_boundary_y.append(y)
    # 绘制当前雷达点云
    ax_explore.scatter(radar_points_x, radar_points_y, s=8, c='red', alpha=0.8, label='4m内雷达点', zorder=3)
    # 新增：绘制雷达可视范围
    if visibility_boundary_x and visibility_boundary_y:
        # 按角度排序边界点，形成闭合多边形
        angles = []
        for i, (x, y) in enumerate(zip(visibility_boundary_x, visibility_boundary_y)):
            dx = x - car_pos[0]
            dy = y - car_pos[1]
            angle = np.arctan2(dy, dx)
            angles.append((angle, i))
        angles.sort()
        # 重新排序边界点
        sorted_boundary_x = [visibility_boundary_x[i] for _, i in angles]
        sorted_boundary_y = [visibility_boundary_y[i] for _, i in angles]
        # 添加小车位置作为起始点，形成闭合多边形
        sorted_boundary_x.insert(0, car_pos[0])
        sorted_boundary_y.insert(0, car_pos[1])
        sorted_boundary_x.append(car_pos[0])
        sorted_boundary_y.append(car_pos[1])
        # 绘制可视范围填充区域
        # ax_explore.fill(sorted_boundary_x, sorted_boundary_y, alpha=0.2, color='yellow', label='雷达可视范围', zorder=4)
        # 绘制可视范围边界线
        # ax_explore.plot(sorted_boundary_x, sorted_boundary_y, 'y-', linewidth=1, alpha=0.6, zorder=5)
    # 绘制小车
    ax_explore.plot(car_pos[0], car_pos[1], 'ro', label='小车', zorder=6)
    ax_explore.set_xlabel('X')
    ax_explore.set_ylabel('Y')
    ax_explore.set_title('实时探索视图')
    
    # 将图例移到地图框外，并设置为紧凑布局
    ax_explore.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), borderaxespad=0., 
                    frameon=True, fontsize=9, title="图例")
    
    # 确保图例不会被裁剪
    plt.tight_layout()
    
    # 保持坐标轴比例一致，但使用固定的显示范围
    ax_explore.set_aspect('equal')
    ax_explore.set_xlim(-5, 25)
    ax_explore.set_ylim(-5, 25)
    
    # 如果提供了完整地图视图，则更新
    if ax_full is not None:
        draw_full_map(ax_full, car_pos)
    
    plt.draw()

def can_move_dfs(direction):
    """
    判断某方向是否可通行。direction: 0=上, 1=右, 2=下, 3=左
    雷达数据第0条为正右方，逆时针为正方向。
    方向与角度映射：右=0度，上=90度，左=180度，下=270度。
    """
    vehicle_controller.robot.theta = 0  # 先设置朝向为0
    radar = get_radar()
    # 检测可通行性
    if radar is None or len(radar) < 360:
        print(f"can_move_dfs: 方向{direction} 雷达数据无效，返回False")
        return False
    # 新映射：右=0，上=90，左=180，下=270
    dir_to_angle = {0: 90, 1: 0, 2: 270, 3: 180}
    center_angle = dir_to_angle[direction]
    indices = [(center_angle - 45 + i) % 360 for i in range(90)]
    short_count = sum(1 for idx in indices if radar[idx] is not None and radar[idx] < 0.707)
    result = short_count < 80
    print(f"can_move_dfs: 方向{direction}（中心角{center_angle}°）90条中有{short_count}条<0.707，结果：{'可通行' if result else '不可通行'}")
    return result

def dfs_search(pos, visited, ax=None):
    """
    递归DFS搜索树，pos为当前点，visited为已访问集合，ax为可视化坐标轴
    每次移动1m，分多步调用move_global，每步0.5m。
    方向编号：0=上, 1=右, 2=下, 3=左
    """
    global exit_pos_list
    # 出口条件：有180条雷达距离大于40，认为走出迷宫边界
    radar = get_radar()
    if sum(1 for d in radar if d > 40) >= 180:
        print("检测到出口，已走出迷宫边界，直接返回。")
        exit_pos_list.append(pos.copy())  # 记录出口位置
        return
    key = tuple(pos)  # 直接保存原始浮点坐标
    if key in visited:
        print("?????")
        return
    visited.add(key)
    if ax is not None:
        update_plot(ax[0], ax[1])
        plt.pause(0.00000001)

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 0=上, 1=右, 2=下, 3=左
    for d, (dx, dy) in enumerate(directions):
        # 计算目标点
        next_pos = [pos[0] + dx * 1.0, pos[1] + dy * 1.0]  # 修改为1.0m
        next_key = tuple(next_pos)
        if next_key in visited:
            print("visited!")
            continue
        if can_move_dfs(d):
            # 剪枝：以该方向为中心角，若其180条射线所定位出的点，相邻射线所定位出的点的距离均小于1m，则不探索该方向
            radar = get_radar()
            dir_to_angle = {0: 90, 1: 0, 2: 270, 3: 180}
            center_angle = dir_to_angle[d]
            indices = [(center_angle - 90 + i) % 360 for i in range(180)]
            points = []
            for idx in indices:
                dist = radar[idx]
                angle = np.deg2rad(idx)
                if dist is not None and dist > 0.01:
                    x = car_pos[0] + dist * np.cos(angle)
                    y = car_pos[1] + dist * np.sin(angle)
                    points.append((x, y))
                else:
                    points.append(None)
            all_close = True
            for i in range(1, len(points)):
                if points[i] is not None and points[i-1] is not None:
                    dx = points[i][0] - points[i-1][0]
                    dy = points[i][1] - points[i-1][1]
                    dist = np.hypot(dx, dy)
                    if dist >= 0.5:
                        all_close = False
                        break
            if all_close:
                print(f"方向{d}（中心角{center_angle}°）180条射线相邻点距离均<0.5m，剪枝不探索该方向")
                continue
            total_dist = 1.0  # 修改为1.0m
            step = 1.0        # 步长为1.0m
            steps = int(total_dist / step)
            for _ in range(steps):
                global isnext
                isnext = True
                move_global(d, step)
                print(f"当前位置: x={vehicle_controller.robot.x:.3f}, y={vehicle_controller.robot.y:.3f}")
                if ax is not None:
                    update_plot(ax[0], ax[1])
                    plt.pause(0.0000001)


            if ax is not None:
                update_plot(ax[0], ax[1])
                plt.pause(0.00000001)

            _ = get_radar()
            dfs_search(car_pos.copy(), visited, ax)
            # 回溯：走回原点
            for _ in range(steps):
                move_global((d + 2) % 4, step)
                print(f"回溯: x={vehicle_controller.robot.x:.3f}, y={vehicle_controller.robot.y:.3f}")
                if ax is not None:
                    update_plot(ax[0], ax[1])
                    plt.pause(0.0000001)


def explore_maze(start_x, start_y, map_file_path):
    """
    探索迷宫的主函数
    
    参数:
    start_x, start_y: 小车起始位置坐标
    map_file_path: 地图文件路径
    
    返回:
    valid_exits: 有效出口位置列表 [(x, y), ...]
    history_points_cleaned: 去重后的历史点云数据 [(x, y), ...]，保留一位小数
    """
    global vehicle_controller, radar_interface, car_pos, history_points, exit_pos_list, visible_grid_mask, all_radar_data
    
    print(f"探索迷宫: 使用起始点 ({start_x}, {start_y}), 地图文件: {map_file_path}")
    
    # 清空雷达数据列表和已记录位置集合
    all_radar_data = []
    recorded_positions.clear()
    
    # 加载完整地图信息
    load_full_map(map_file_path)
    
    # 重新初始化车辆控制器和雷达接口
    vehicle_controller = create_vehicle_controller(start_x, start_y, 0, map_file_path)
    radar_interface = create_radar_interface(vehicle_controller.robot, map_file_path)
    
    # 重置全局变量
    car_pos = [vehicle_controller.robot.x, vehicle_controller.robot.y]
    history_points = []
    exit_pos_list = []
    # ====== 只在此处初始化可见掩码 ======
    from common.utils import RES, GRID_SIZE, ORIGIN_X, ORIGIN_Y
    grid_size = RES
    x_min = ORIGIN_X
    y_min = ORIGIN_Y
    x_max = ORIGIN_X + GRID_SIZE * RES
    y_max = ORIGIN_Y + GRID_SIZE * RES
    width = int(np.ceil((x_max - x_min) / grid_size))
    height = int(np.ceil((y_max - y_min) / grid_size))
    visible_grid_mask = np.zeros((width, height), dtype=bool)
    # ====== 结束初始化 ======
    
    plt.ion()
    # 创建分屏视图，增加右侧图的宽度以容纳图例
    fig, (ax_full, ax_explore) = plt.subplots(1, 2, figsize=(20, 9), gridspec_kw={'width_ratios': [1, 1.2]})
    # 增加右侧边距，为图例留出空间
    plt.subplots_adjust(bottom=0.1, wspace=0.3, right=0.85, top=0.9)
    
    # 设置图形的整体标题
    fig.suptitle('迷宫探索过程', fontsize=16)
    
    # 初始化两个视图
    draw_full_map(ax_full, car_pos)  # 左侧显示完整地图
    update_plot(ax_explore, ax_full)  # 右侧显示探索视图
    
    visited = set()
    start_pos = [start_x, start_y]
    car_pos[0], car_pos[1] = start_pos[0], start_pos[1]
    vehicle_controller.robot.x = start_pos[0]
    vehicle_controller.robot.y = start_pos[1]
    radar = get_radar()
    # 起始位置特殊扫描与移动
    # 方向编号：0=上, 1=右, 2=下, 3=左
    # 角度区间：上[0,180)，右[270,360)+[0,90)，下[180,360)，左[90,270)
    print(radar) 
    direction_checks = [
        (0, list(range(0, 180))),  # 上
        (1, list(range(270, 360)) + list(range(0, 90))),  # 右
        (2, list(range(180, 360))),  # 下
        (3, list(range(90, 270)))   # 左
    ]
    for d, indices in direction_checks:
        far_count = sum(1 for idx in indices if radar[idx] is not None and radar[idx] > 30)
        if far_count >= 160:
            if d == 0:  # 上
                print("起始位置检测：上方空旷，向右、向下各移动0.5m")
                move_global(1, 0.5)  # 右
                move_global(2, 0.5)  # 下
            elif d == 1:  # 右
                print("起始位置检测：右方空旷，向左、向下各移动0.5m")
                move_global(3, 0.5)  # 左
                move_global(2, 0.5)  # 下
            elif d == 2:  # 下
                print("起始位置检测：下方空旷，向右、向上各移动0.5m")
                move_global(1, 0.5)  # 右
                move_global(0, 0.5)  # 上
            elif d == 3:  # 左
                print("起始位置检测：左方空旷，向下、向右各移动0.5m")
                move_global(2, 0.5)  # 下
                move_global(1, 0.5)  # 右
            # 只执行一次
            break
    # 可通行性测试
    radar = get_radar()
    direction_names = ['上', '右', '下', '左']
    direction_angles = [0, 90, 180, 270]
    for i, name in enumerate(direction_names):
        center_angle = direction_angles[i]
        indices = [(center_angle - 45 + j) % 360 for j in range(90)]
        short_count = sum(1 for idx in indices if radar[idx] is not None and radar[idx] < 0.707)
        can_pass = short_count < 80
        print(f"测试: 方向{name}（中心角{center_angle}°）90条中有{short_count}条<0.707，可通行性: {can_pass}")
    print(vehicle_controller.robot.x, vehicle_controller.robot.y)
    car_pos[0], car_pos[1] = vehicle_controller.robot.x, vehicle_controller.robot.y
    dfs_search(car_pos.copy(), visited, [ax_explore, ax_full])
    # 筛选有效出口位置
    valid_exits = []
    for pos in exit_pos_list:
        dist = np.hypot(pos[0] - start_pos[0], pos[1] - start_pos[1])
        if dist >= 1:
            valid_exits.append(pos)
    if valid_exits:
        print("有效出口位置：")
        for i, pos in enumerate(valid_exits):
            print(f"  出口{i+1}: x={pos[0]:.3f}, y={pos[1]:.3f}, 距起点{np.hypot(pos[0]-start_pos[0], pos[1]-start_pos[1]):.3f}m")
    else:
        print("未检测到有效出口位置（距离起点均小于1m）")
    
    # 处理历史点云数据：去重并保留一位小数
    history_points_cleaned = []
    seen_points = set()
    for point in history_points:
        # 保留一位小数
        rounded_point = (round(point[0], 1), round(point[1], 1))
        if rounded_point not in seen_points:
            history_points_cleaned.append(rounded_point)
            seen_points.add(rounded_point)
    
    # 新增：合并距离小于1.1m的出口点
    merged_exits = valid_exits.copy()
    merged = True
    while merged:
        merged = False
        i = 0
        while i < len(merged_exits):
            j = i + 1
            while j < len(merged_exits):
                dist = np.hypot(merged_exits[i][0] - merged_exits[j][0], 
                               merged_exits[i][1] - merged_exits[j][1])
                if dist < 1.1:
                    # 计算中点
                    mid_x = (merged_exits[i][0] + merged_exits[j][0]) / 2
                    mid_y = (merged_exits[i][1] + merged_exits[j][1]) / 2
                    mid_point = [mid_x, mid_y]
                    
                    # 记录要合并的点坐标用于打印
                    point1 = merged_exits[i]
                    point2 = merged_exits[j]
                    
                    # 移除原来的两个点，添加中点
                    merged_exits.pop(j)
                    merged_exits.pop(i)
                    merged_exits.append(mid_point)
                    merged = True
                    print(f"合并出口点：({point1[0]:.3f}, {point1[1]:.3f}) 和 ({point2[0]:.3f}, {point2[1]:.3f}) -> ({mid_x:.3f}, {mid_y:.3f})")
                    break
                j += 1
            if merged:
                break
            i += 1
    
    update_plot(ax_explore, ax_full)
    plt.ioff()
    plt.show()
    plt.close()  # 关闭绘图窗口
    
    # 保存点云、起点和出口数据到.pkl文件
    explore_result_filename = map_file_path.split('.')[0] + '_result.pkl'
    print(f"\n保存探索结果到文件: {explore_result_filename}")
    
    try:
        # 创建要保存的数据字典
        explore_data = {
            'point_cloud': history_points_cleaned,  # 点云列表 [(x1, y1), (x2, y2), ...]
            'start': (start_x, start_y),  # 起点坐标 (x, y)
            'exits': merged_exits  # 出口坐标列表 [(x1, y1), (x2, y2), ...]
        }
        
        # 将数据序列化到文件
        with open(explore_result_filename, 'wb') as f:
            pickle.dump(explore_data, f)
            
        print(f"成功保存探索结果到 {explore_result_filename}")
        print(f"数据格式: 包含 'point_cloud'、'start' 和 'exits' 键的字典")
        print(f"- 点云数据: {len(history_points_cleaned)} 个点")
        print(f"- 起始位置: ({start_x}, {start_y})")
        print(f"- 出口位置: {len(merged_exits)} 个")
        
    except Exception as e:
        print(f"保存探索结果时出错: {str(e)}")

    # 保存雷达数据到.txt文件
    output_filename = map_file_path.split('.')[0] + '_radar_data.txt'
    print(f"\n保存雷达数据到文件: {output_filename}")
    
    try:
        with open(output_filename, 'w') as f:
            # 第一行写入元数据：栅格大小和小车半径
            # 格式：METADATA GRID_SIZE 0.05 CAR_RADIUS 0.05
            grid_size = RES  # 从common.utils导入的栅格大小
            car_radius = RES  # 小车半径默认为一个栅格大小
            f.write(f"METADATA GRID_SIZE {grid_size} CAR_RADIUS {car_radius}\n")
            
            # 保存雷达扫描数据，每行一次扫描
            for car_pos, radar_scan in all_radar_data:
                # 将None值替换为-1，50值(无墙交接)替换为0，便于文本格式保存和读取
                radar_line = []
                for val in radar_scan:
                    if val is None:
                        radar_line.append('-1')
                    elif val >= 40:  # 大于等于40的视为无墙交接
                        radar_line.append('0')
                    else:
                        radar_line.append(str(round(val, 3)))
                
                # 每行开头添加小车位置信息: CAR_X CAR_Y radar_data...
                car_x, car_y = car_pos
                f.write(f"{car_x} {car_y} " + ' '.join(radar_line) + '\n')
            
            # 倒数第二行保存入口和出口信息，格式：ENTRANCE x y EXIT1 x1 y1 EXIT2 x2 y2...
            entrance_exit_line = f"ENTRANCE {start_x} {start_y} "
            for i, (ex, ey) in enumerate(merged_exits):
                entrance_exit_line += f"EXIT{i+1} {ex} {ey} "
            f.write(entrance_exit_line.strip() + '\n')
            
        print(f"成功保存 {len(all_radar_data)} 次雷达扫描数据到文本文件")
        print(f"文件格式: 第一行包含元数据(栅格大小和小车半径)")
        print(f"          中间各行格式为 '小车X坐标 小车Y坐标 雷达数据(360个值)'")
        print(f"          最后一行包含入口和出口坐标")
        print(f"特别说明: 雷达值中-1表示无效数据, 0表示无墙交接(射线值>=40m)")
        print(f"元数据信息: 栅格大小={grid_size}米, 小车半径={car_radius}米")
        
    except Exception as e:
        print(f"保存雷达数据时出错: {str(e)}")

    # 不再自动调用map_and_path_planner.py处理生成的雷达数据文件
    print(f"\n已禁用自动调用map_and_path_planner.py")
    print(f"您可以手动运行以下命令处理雷达数据:")
    print(f"python map_and_path_planner.py {output_filename}")

    return merged_exits, history_points_cleaned

if __name__ == "__main__":
    # 添加命令行参数解析
    import argparse
    parser = argparse.ArgumentParser(description='探索迷宫并生成雷达数据')
    parser.add_argument('map_file', nargs='?', default='maze_edges.json', help='迷宫地图文件路径')
    parser.add_argument('--no-planner', action='store_true', default=True, help='禁用自动调用map_and_path_planner.py')
    parser.add_argument('--no-animation', action='store_true', help='禁用路径规划动画')
    parser.add_argument('--speed', type=float, default=0.01, help='动画速度 (暂停时间，越小越快，默认0.01)')
    parser.add_argument('--no-car-animation', action='store_true', help='禁用小车移动动画')
    parser.add_argument('--car-speed', type=float, default=0.05, help='小车动画速度 (暂停时间，越小越快，默认0.05)')
    parser.add_argument('--output', default=None, help='输出文件名前缀，不包含扩展名')
    args = parser.parse_args()
    
    # 使用解析后的参数
    map_file = args.map_file
    print(f"使用指定的迷宫文件: {map_file}")
    
    # 从地图文件中读取起始位置
    try:
        with open(map_file, 'r') as f:
            map_data = json.load(f)
            print(f"地图文件内容: {json.dumps(map_data, indent=2)}")
            
            # 检查地图文件中可能的起始位置字段
            start_fields = ['start_point', 'start_position', 'start', 'initial_position']
            start_found = False
            
            # 从地图文件中读取起始位置
            for field in start_fields:
                if field in map_data:
                    start_x, start_y = map_data[field]
                    print(f"从地图文件字段'{field}'读取起始位置: ({start_x}, {start_y})")
                    start_found = True
                    break
                        
                if not start_found:
                    # 如果地图中没有起始位置信息，使用默认值
                    start_x, start_y = 3, 0
                    print(f"地图文件中未找到起始位置信息，使用默认位置: ({start_x}, {start_y})")
    except Exception as e:
        print(f"读取地图文件出错: {str(e)}")
        # 发生错误时使用默认值
        start_x, start_y = 3, 0
        print(f"使用默认起始位置: ({start_x}, {start_y})")
        
    # 初始化全局控制器
    vehicle_controller = create_vehicle_controller(start_x, start_y, 0, map_file)
    radar_interface = create_radar_interface(vehicle_controller.robot, map_file)
    
    # 更新小车位置
    car_pos[0] = vehicle_controller.robot.x
    car_pos[1] = vehicle_controller.robot.y
    
    exits, point_cloud = explore_maze(start_x, start_y, map_file)
    
    print(f"\n探索完成！")
    print(f"发现 {len(exits)} 个有效出口")
    print(f"收集到 {len(point_cloud)} 个去重后的点云数据")
    


    # 新增：对返回数据进行可视化
    plt.figure(figsize=(12, 9))
    
    # 点云数据可视化
    if point_cloud:
        cloud_x, cloud_y = zip(*point_cloud)
        plt.scatter(cloud_x, cloud_y, c='green', s=1, alpha=0.6, label='点云数据')
    plt.scatter(start_x, start_y, c='blue', s=200, marker='o', label='起始位置')
    if exits:
        exit_x, exit_y = zip(*exits)
        plt.scatter(exit_x, exit_y, c='red', s=100, marker='*', label='出口位置')
        # 标注出口编号
        for i, (x, y) in enumerate(exits):
            plt.annotate(f'出口{i+1}', (x, y), xytext=(5, 5), textcoords='offset points')
    plt.xlabel('X坐标')
    plt.ylabel('Y坐标')
    plt.title('迷宫探索结果可视化', fontsize=16)
    # 将图例移到右侧
    plt.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), frameon=True)
    plt.grid(True, alpha=0.3)
    
    # 保持坐标轴比例一致，使用合适的显示范围
    plt.gca().set_aspect('equal')
    plt.xlim(-5, 25)
    plt.ylim(-5, 25)
    
    # 调整布局，为图例留出空间
    plt.tight_layout()
    plt.subplots_adjust(right=0.85)
    plt.show()
    
    # 打印详细统计信息
    print(f"\n=== 详细统计信息 ===")
    print(f"起始位置: ({start_x}, {start_y})")
    if exits:
        print(f"出口位置:")
        for i, (x, y) in enumerate(exits):
            dist = np.hypot(x - start_x, y - start_y)
            print(f"  出口{i+1}: ({x:.3f}, {y:.3f}), 距起点: {dist:.3f}m")
    else:
        print("未发现有效出口")
    
    if point_cloud:
        # 计算点云覆盖范围
        cloud_x, cloud_y = zip(*point_cloud)
        min_x, max_x = min(cloud_x), max(cloud_x)
        min_y, max_y = min(cloud_y), max(cloud_y)
        print(f"点云覆盖范围: X[{min_x:.1f}, {max_x:.1f}], Y[{min_y:.1f}, {max_y:.1f}]")
        print(f"点云密度: {len(point_cloud)} 个点")
    else:
        print("未收集到点云数据")
        
    # 保存雷达数据到.txt文件
    output_filename = map_file.split('.')[0] + '_radar_data.txt'
    print(f"\n保存雷达数据到文件: {output_filename}")
    
    try:
        with open(output_filename, 'w') as f:
            # 第一行写入元数据：栅格大小和小车半径
            # 格式：METADATA GRID_SIZE 0.05 CAR_RADIUS 0.05
            grid_size = RES  # 从common.utils导入的栅格大小
            car_radius = RES  # 小车半径默认为一个栅格大小
            f.write(f"METADATA GRID_SIZE {grid_size} CAR_RADIUS {car_radius}\n")
            
            # 保存雷达扫描数据，每行一次扫描
            for car_pos, radar_scan in all_radar_data:
                # 将None值替换为-1，50值(无墙交接)替换为0，便于文本格式保存和读取
                radar_line = []
                for val in radar_scan:
                    if val is None:
                        radar_line.append('-1')
                    elif val >= 40:  # 大于等于40的视为无墙交接
                        radar_line.append('0')
                    else:
                        radar_line.append(str(round(val, 3)))
                
                # 每行开头添加小车位置信息: CAR_X CAR_Y radar_data...
                car_x, car_y = car_pos
                f.write(f"{car_x} {car_y} " + ' '.join(radar_line) + '\n')
            
            # 倒数第二行保存入口和出口信息，格式：ENTRANCE x y EXIT1 x1 y1 EXIT2 x2 y2...
            entrance_exit_line = f"ENTRANCE {start_x} {start_y} "
            for i, (ex, ey) in enumerate(exits):
                entrance_exit_line += f"EXIT{i+1} {ex} {ey} "
            f.write(entrance_exit_line.strip() + '\n')
            
        print(f"成功保存 {len(all_radar_data)} 次雷达扫描数据到文本文件")
        print(f"文件格式: 第一行包含元数据(栅格大小和小车半径)")
        print(f"          中间各行格式为 '小车X坐标 小车Y坐标 雷达数据(360个值)'")
        print(f"          最后一行包含入口和出口坐标")
        print(f"特别说明: 雷达值中-1表示无效数据, 0表示无墙交接(射线值>=40m)")
        print(f"元数据信息: 栅格大小={grid_size}米, 小车半径={car_radius}米")
        
    except Exception as e:
        print(f"保存雷达数据时出错: {str(e)}")

    # 不再自动调用map_and_path_planner.py处理生成的雷达数据文件
    print(f"\n已禁用自动调用map_and_path_planner.py")
    print(f"您可以手动运行以下命令处理雷达数据:")
    print(f"python map_and_path_planner.py {output_filename}")
