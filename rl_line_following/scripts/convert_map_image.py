#!/usr/bin/env python3
"""
将map.jpg转换为ROS map_server可以使用的pgm格式
"""
import cv2
import numpy as np
import yaml
import os

def convert_jpg_to_pgm(jpg_path, output_dir):
    """
    将JPG图像转换为PGM格式的地图
    """
    # 读取JPG图像
    img = cv2.imread(jpg_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print(f"错误：无法读取图像 {jpg_path}")
        return False
    
    print(f"原始图像尺寸: {img.shape}")
    
    # 二值化处理（确保黑白图像）
    # 如果图像不是二值化的，进行阈值处理
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    
    # ROS map_server中：
    # - 0 (黑色) = 障碍物 (occupied)
    # - 255 (白色) = 自由空间 (free)
    # - 205 = 未知区域
    
    # 反转：如果原图中白色是路径，黑色是障碍物，需要反转
    # 如果原图中黑色是路径，白色是背景，不需要反转
    # 这里假设白色(255)是路径/自由空间，黑色(0)是障碍物
    # 如果相反，取消下面这行的注释
    # binary = cv2.bitwise_not(binary)
    
    # 保存为PGM
    pgm_path = os.path.join(output_dir, "map.pgm")
    cv2.imwrite(pgm_path, binary)
    print(f"已保存PGM图像: {pgm_path}")
    
    # 创建YAML配置文件
    yaml_path = os.path.join(output_dir, "map.yaml")
    
    # 获取图像尺寸
    height, width = binary.shape
    
    # 估算分辨率（需要根据实际情况调整）
    # 假设地图是10m x 6m（根据my_map.yaml推测）
    resolution = 0.05  # 5cm/pixel，可以根据实际地图大小调整
    
    yaml_content = {
        'image': 'map.pgm',
        'resolution': resolution,
        'origin': [-width * resolution / 2, -height * resolution / 2, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)
    
    print(f"已保存YAML配置: {yaml_path}")
    print(f"\n地图信息:")
    print(f"  尺寸: {width} x {height} pixels")
    print(f"  分辨率: {resolution} m/pixel")
    print(f"  实际尺寸: {width * resolution:.2f} x {height * resolution:.2f} m")
    print(f"  原点: {yaml_content['origin']}")
    
    return True

if __name__ == '__main__':
    import sys
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    maps_dir = os.path.join(package_dir, 'maps')
    
    jpg_path = os.path.join(maps_dir, 'map.jpg')
    
    if not os.path.exists(jpg_path):
        print(f"错误：找不到 {jpg_path}")
        sys.exit(1)
    
    print("开始转换地图图像...")
    if convert_jpg_to_pgm(jpg_path, maps_dir):
        print("\n✓ 转换成功！")
        print(f"\n使用方法:")
        print(f"  在launch文件中使用: {maps_dir}/map.yaml")
    else:
        print("\n✗ 转换失败")
        sys.exit(1)

