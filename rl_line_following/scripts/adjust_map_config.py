#!/usr/bin/env python3
"""
调整地图配置以匹配Gazebo中的地图显示
根据机器人初始位置和地图尺寸来调整origin和resolution
"""
import yaml
import os

def adjust_map_config(yaml_path, robot_x=2.0, robot_y=-0.7, map_center_x=0, map_center_y=0):
    """
    调整地图配置
    
    Args:
        yaml_path: map.yaml文件路径
        robot_x, robot_y: 机器人在Gazebo中的初始位置
        map_center_x, map_center_y: 地图中心在Gazebo中的位置（通常是0,0）
    """
    # 读取当前配置
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    print(f"当前配置:")
    print(f"  resolution: {config.get('resolution', 'N/A')}")
    print(f"  origin: {config.get('origin', 'N/A')}")
    
    # 读取PGM图像获取尺寸
    pgm_path = os.path.join(os.path.dirname(yaml_path), config['image'])
    if os.path.exists(pgm_path):
        from PIL import Image
        img = Image.open(pgm_path)
        width, height = img.size
        print(f"\n地图图像尺寸: {width} x {height} pixels")
        
        resolution = config.get('resolution', 0.05)
        map_width_m = width * resolution
        map_height_m = height * resolution
        print(f"地图实际尺寸: {map_width_m:.2f} x {map_height_m:.2f} meters")
        
        # 调整origin，使地图中心在(0,0)
        # origin是地图左下角在世界坐标系中的位置
        new_origin_x = -map_width_m / 2
        new_origin_y = -map_height_m / 2
        
        print(f"\n建议的新配置:")
        print(f"  resolution: {resolution}")
        print(f"  origin: [{new_origin_x:.2f}, {new_origin_y:.2f}, 0.0]")
        
        # 询问是否应用
        response = input("\n是否应用这些设置？(y/n): ")
        if response.lower() == 'y':
            config['origin'] = [new_origin_x, new_origin_y, 0.0]
            
            # 备份原文件
            backup_path = yaml_path + '.backup'
            with open(backup_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"已备份原配置到: {backup_path}")
            
            # 保存新配置
            with open(yaml_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"已更新配置: {yaml_path}")
            return True
    else:
        print(f"警告: 找不到PGM文件: {pgm_path}")
    
    return False

if __name__ == '__main__':
    import sys
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    yaml_path = os.path.join(package_dir, 'maps', 'my_map.yaml')
    
    if not os.path.exists(yaml_path):
        print(f"错误: 找不到 {yaml_path}")
        sys.exit(1)
    
    print("=" * 60)
    print("地图配置调整工具")
    print("=" * 60)
    print("\n根据Gazebo中的地图显示，调整Rviz中的地图配置")
    print("机器人初始位置: x=2.0, y=-0.7 (从spawn_models.launch)")
    print()
    
    adjust_map_config(yaml_path)

