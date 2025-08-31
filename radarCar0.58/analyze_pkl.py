#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
分析explore_result_1.pkl文件内容
"""

import pickle
import numpy as np
import sys

def analyze_pkl(filename):
    """分析pkl文件内容"""
    try:
        with open(filename, 'rb') as f:
            data = pickle.load(f)
            
        print(f"\n{'='*50}")
        print(f"文件 {filename} 内容分析:")
        print(f"{'='*50}")
        
        # 分析数据结构
        if isinstance(data, dict):
            print(f"数据类型: 字典 (dict)")
            print(f"包含的键: {list(data.keys())}")
            
            # 分析每个键的内容
            for key in data.keys():
                print(f"\n{'-'*40}")
                print(f"键: '{key}'")
                print(f"值类型: {type(data[key])}")
                
                # 根据值的类型进行不同的处理
                if isinstance(data[key], list):
                    print(f"列表长度: {len(data[key])}")
                    if len(data[key]) > 0:
                        print(f"列表元素类型: {type(data[key][0])}")
                        if key == 'point_cloud':
                            print(f"点云样本 (前5个点): {data[key][:5]}")
                elif isinstance(data[key], tuple):
                    print(f"元组内容: {data[key]}")
                elif isinstance(data[key], np.ndarray):
                    print(f"数组形状: {data[key].shape}")
                    print(f"数组类型: {data[key].dtype}")
                    print(f"数组样本 (前5个元素): {data[key][:5]}")
                else:
                    print(f"值内容: {data[key]}")
        else:
            print(f"数据类型: {type(data)}")
            print(f"数据内容: {data}")
            
    except FileNotFoundError:
        print(f"错误: 找不到文件 {filename}")
    except Exception as e:
        print(f"分析文件时出错: {e}")

if __name__ == "__main__":
    # 默认文件名
    filename = 'explore_result_1.pkl'
    
    # 如果提供了命令行参数，使用提供的文件名
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        
    analyze_pkl(filename) 