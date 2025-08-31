from vehicle_control import create_vehicle_controller
import numpy as np

def test_collision_detection():
    """
    专门测试碰撞检测功能
    """
    print("=== 碰撞检测功能测试 ===")
    
    # 1. 创建车辆控制器
    controller = create_vehicle_controller(
        start_x=15.0, 
        start_y=15.0, 
        start_theta=np.deg2rad(45)
    )
    
    print(f"初始位置: {controller.get_vehicle_position()}")
    
    # 2. 测试不同距离的移动
    test_cases = [
        (0.5, 0, "向前移动0.5米"),
        (1.0, 0, "向前移动1米"),
        (2.0, 0, "向前移动2米"),
        (5.0, 0, "向前移动5米"),
        (10.0, 0, "向前移动10米"),  # 这个应该会碰壁
    ]
    
    for distance, direction, description in test_cases:
        print(f"\n--- {description} ---")
        
        # 获取当前雷达数据
        radar_data = controller.get_radar_data()
        front_min = min(radar_data[345:] + radar_data[:16])  # 前方-15到15度
        
        print(f"  前方最小距离: {front_min:.2f} 米")
        print(f"  移动距离: {distance} 米")
        print(f"  车体半径: {controller.vehicle_radius} 米")
        print(f"  安全裕量: {controller.safety_margin} 米")
        print(f"  安全距离: {distance + controller.vehicle_radius + controller.safety_margin:.2f} 米")
        
        # 执行移动
        collision = controller.move_vehicle(distance, direction)
        
        if collision:
            print(f"  ❌ 预测会碰壁！移动失败")
        else:
            print(f"  ✅ 预测安全，移动成功")
            x, y, theta = controller.get_vehicle_position()
            print(f"  新位置: ({x:.2f}, {y:.2f})")
    
    # 3. 测试不同方向的移动
    print(f"\n=== 测试不同方向移动 ===")
    controller = create_vehicle_controller(
        start_x=15.0, 
        start_y=15.0, 
        start_theta=np.deg2rad(45)
    )
    
    directions = [
        (0, "前"),
        (1, "右"), 
        (2, "后"),
        (3, "左")
    ]
    
    for direction, name in directions:
        print(f"\n--- 向{name}移动1米 ---")
        collision = controller.move_vehicle(1.0, direction)
        
        if collision:
            print(f"  ❌ 预测会碰壁！")
        else:
            print(f"  ✅ 预测安全，移动成功")
            x, y, theta = controller.get_vehicle_position()
            print(f"  位置: ({x:.2f}, {y:.2f})")
    
    # 4. 测试参数验证
    print(f"\n=== 测试参数验证 ===")
    controller = create_vehicle_controller(
        start_x=15.0, 
        start_y=15.0, 
        start_theta=np.deg2rad(45)
    )
    
    try:
        controller.move_vehicle(-1.0, 0)  # 负距离
        print("❌ 应该抛出异常但没有")
    except ValueError as e:
        print(f"✅ 正确捕获异常: {e}")
    
    try:
        controller.move_vehicle(1.0, 5)  # 无效方向
        print("❌ 应该抛出异常但没有")
    except ValueError as e:
        print(f"✅ 正确捕获异常: {e}")

if __name__ == "__main__":
    test_collision_detection() 