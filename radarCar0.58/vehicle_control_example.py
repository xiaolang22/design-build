from vehicle_control import create_vehicle_controller
from radar_interface import create_radar_interface, RadarInterface
import numpy as np

def main():
    """
    演示车辆控制接口的使用
    """
    # 1. 创建机器人、雷达接口和车辆控制器
    start_x, start_y, start_theta = 15.0, 15.0, np.deg2rad(45)
    controller = create_vehicle_controller(start_x, start_y, start_theta)
    radar = create_radar_interface(controller.robot)
    
    print("——车辆控制接口演示——")
    print(f"初始位置: ({controller.robot.x:.2f}, {controller.robot.y:.2f}, {np.rad2deg(controller.robot.theta):.1f}°)")
    
    # 2. 获取并显示雷达数据
    radar_data = radar.get_current_radar_data()
    print(f"\n雷达数据长度: {len(radar_data)}")
    print(f"前方距离 (0度): {radar_data[0]:.2f} 米")
    print(f"右方距离 (90度): {radar_data[90]:.2f} 米")
    print(f"后方距离 (180度): {radar_data[180]:.2f} 米")
    print(f"左方距离 (270度): {radar_data[270]:.2f} 米")
    
    # 3. 测试不同的移动操作
    test_moves = [
        (1.0, 0, "向前移动1米"),
        (1.0, 1, "向右移动1米"),
        (1.0, 2, "向后移动1米"),
        (1.0, 3, "向左移动1米"),
        (2.0, 0, "向前移动2米"),
        (0.5, 1, "向右移动0.5米"),
    ]
    
    for distance, direction, description in test_moves:
        print(f"\n尝试: {description}")
        print(f"  距离: {distance}, 方向: {direction}")
        
        # 获取移动前的雷达数据
        radar_data = radar.get_current_radar_data()
        
        # 根据方向显示相关雷达数据
        if direction == 0:  # 前
            relevant_data = radar_data[345:] + radar_data[:16]  # -15到15度
            print(f"  前方雷达数据范围: {min(relevant_data):.2f} - {max(relevant_data):.2f} 米")
        elif direction == 1:  # 右
            relevant_data = radar_data[75:106]  # 75到105度
            print(f"  右方雷达数据范围: {min(relevant_data):.2f} - {max(relevant_data):.2f} 米")
        elif direction == 2:  # 后
            relevant_data = radar_data[165:196]  # 165到195度
            print(f"  后方雷达数据范围: {min(relevant_data):.2f} - {max(relevant_data):.2f} 米")
        elif direction == 3:  # 左
            relevant_data = radar_data[255:286]  # 255到285度
            print(f"  左方雷达数据范围: {min(relevant_data):.2f} - {max(relevant_data):.2f} 米")
        
        # 调用移动接口
        collision = controller.move_vehicle(distance, direction, radar_data)
        
        if collision:
            print(f"  预测会碰壁！移动失败")
        else:
            print(f"  预测安全，移动成功")
            print(f"  当前位置: ({controller.robot.x:.2f}, {controller.robot.y:.2f}), 朝向: {np.rad2deg(controller.robot.theta):.1f}°")
    
    print(f"\n最终位置: ({controller.robot.x:.2f}, {controller.robot.y:.2f}, {np.rad2deg(controller.robot.theta):.1f}°)")
    
    # 4. 保存雷达数据到文件
    filename = RadarInterface.save_radar_data(radar_data)
    print(f"\n雷达数据已保存到: {filename}")

if __name__ == "__main__":
    main() 