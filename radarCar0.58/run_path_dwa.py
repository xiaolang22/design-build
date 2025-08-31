#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
用于测试path_DWA.py的DWA算法跟踪A*规划的路径
"""

import os
import sys
import argparse

def main():
    """
    主函数：解析命令行参数并启动DWA路径跟踪
    """
    parser = argparse.ArgumentParser(description='使用path_DWA.py的DWA算法跟踪A*规划的路径')
    parser.add_argument('--file', default='1_radar_data.txt', help='雷达数据文件路径 (默认: 1_radar_data.txt)')
    parser.add_argument('--no-animation', action='store_true', help='禁用路径规划动画')
    parser.add_argument('--speed', type=float, default=0.01, help='路径规划动画速度 (暂停时间，越小越快，默认0.01)')
    parser.add_argument('--car-speed', type=float, default=0.02, help='小车动画速度 (暂停时间，越小越快，默认0.02)')
    parser.add_argument('--all-exits', action='store_true', help='为所有出口规划路径并显示最短路径')
    args = parser.parse_args()
    
    # 构建命令行参数
    cmd_args = [
        f'"{sys.executable}"',  # 使用当前Python解释器
        'map_and_path_planner.py',
        f'"{args.file}"',
        '--use-dwa'  # 启用DWA控制器
    ]
    
    # 添加其他参数
    if args.no_animation:
        cmd_args.append('--no-animation')
    
    if args.speed != 0.01:
        cmd_args.append(f'--speed {args.speed}')
    
    if args.car_speed != 0.05:
        cmd_args.append(f'--car-speed {args.car_speed}')
    
    if args.all_exits:
        cmd_args.append('--all-exits')
    
    # 构建并执行命令
    cmd = ' '.join(cmd_args)
    print(f"执行命令: {cmd}")
    os.system(cmd)

if __name__ == "__main__":
    main() 