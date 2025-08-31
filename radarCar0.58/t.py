import numpy as np
import matplotlib.pyplot as plt

# 读取距离数据
filename = 'radar_data_0_27.50_22.50.txt'
with open(filename, 'r', encoding='utf-8') as f:
    distances = [float(line.strip()) for line in f if line.strip()]

distances = np.array(distances)
angles = np.deg2rad(np.arange(len(distances)))

# 极坐标雷达图
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, polar=True)
ax.plot(angles, distances, '.', color='lime', label='雷达距离点')
# 设置0度在右侧，逆时针为正方向
ax.set_theta_zero_location('E')  # 0度在右侧
ax.set_theta_direction(1)       # 逆时针为正方向
ax.set_title('雷达距离极坐标图')
ax.set_ylim(0, np.nanmax(distances) * 1.1)
ax.legend()
plt.show()
