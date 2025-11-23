#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import math
import bisect
import numpy as np
import matplotlib.pyplot as plt

"""
修订后的 lidar_grid_mapping.py，针对目前出现的问题：
1. 将激光角度改为 0°~180° 的区间，避免 -90°~+90° 与实际设备不符。
2. 添加激光相对车体安装的偏移量 LASER_OFFSET_RAD，可根据需要调整。
3. 其余部分(时间对齐、读取ld.nav等)与前版本类似。
"""

# 如果雷达的 0° 实际是车头向左，则把 offset 设为 -pi/2，能转到“车头前方=0°”
# 如果还是不对，可换成 0, +math.pi/2, math.pi 等。
LASER_OFFSET_RAD = 0 

def load_lidar_data(lms_file):
    """
    读取 .lms 文件(二进制):
      float: [AngRng, AngRes, Unit]
      后续若干帧: {time_ms(4字节), dist_array(max_len * unsigned short)}
    返回 (AngRng, AngRes, Unit, scans)
    scans为 list of (time_ms, [dist0, dist1, ...])，已转为米
    """
    with open(lms_file, "rb") as f:
        header = f.read(12)
        AngRng, AngRes, Unit = struct.unpack("fff", header)
        max_len = int(AngRng / AngRes + 1)

        scans = []
        idx = 0
        while True:
            buf_time = f.read(4)
            if not buf_time or len(buf_time) < 4:
                break
            time_ms = struct.unpack("<i", buf_time)[0]

            buf_data = f.read(2 * max_len)
            if not buf_data or len(buf_data) < 2 * max_len:
                break
            dist_array = struct.unpack("<" + "H" * max_len, buf_data)

            # 题目: dat / Unit => meter。Unit=100 => 1count=0.01m
            dist_m = [d / Unit for d in dist_array]
            scans.append((time_ms, dist_m))

    return AngRng, AngRes, Unit, scans

def load_nav_data(nav_file):
    """
    读取 ld.nav: time ang.x ang.y ang.z shv.x shv.y shv.z
    time(ms), yaw=ang.z(弧度)，x=shv.x(米)，y=shv.y(米)
    """
    time_list = []
    x_list    = []
    y_list    = []
    yaw_list  = []
    with open(nav_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 7:
                t   = int(parts[0])
                # ang.z => 航向角(弧度)
                yaw = float(parts[3])
                sx  = float(parts[4])
                sy  = float(parts[5])
                time_list.append(t)
                x_list.append(sx)
                y_list.append(sy)
                yaw_list.append(yaw)
    return time_list, x_list, y_list, yaw_list

def find_pose_at(time_list, x_list, y_list, yaw_list, t_query):
    """ 二分查找在 nav 的时间序列里找与 t_query 最近的姿态 (x, y, yaw). """
    idx = bisect.bisect_left(time_list, t_query)
    if idx <= 0:
        return x_list[0], y_list[0], yaw_list[0]
    if idx >= len(time_list):
        return x_list[-1], y_list[-1], yaw_list[-1]
    before_t = time_list[idx-1]
    after_t  = time_list[idx]
    if abs(t_query - before_t) < abs(t_query - after_t):
        return x_list[idx-1], y_list[idx-1], yaw_list[idx-1]
    else:
        return x_list[idx], y_list[idx], yaw_list[idx]

def lidar_mapping(lms_file, nav_file):
    """ 核心：读取激光 + nav 轨迹 -> 构建 hit_map """
    # 1) 读激光
    AngRng, AngRes, Unit, scans = load_lidar_data(lms_file)
    print(f"Load Lidar: AngRng={AngRng}, AngRes={AngRes}, Unit={Unit}, frames={len(scans)}")

    # 2) 读 nav
    time_list, x_list, y_list, yaw_list = load_nav_data(nav_file)
    print(f"Load Nav: total poses={len(time_list)}")

    # 3) 激光束角度：0° ~ 180° (若硬件就是这区间)
    max_len = int(AngRng / AngRes + 1)
    angles_deg = []
    angle_start = 0.0
    for i in range(max_len):
        a_deg = angle_start + i * AngRes  # 0, 0.5, 1.0, ..., 180
        angles_deg.append(a_deg)

    # 4) 收集激光点
    all_points = []
    for (t_ms, dist_arr) in scans:
        # 查轨迹姿态
        rx, ry, rtheta = find_pose_at(time_list, x_list, y_list, yaw_list, t_ms)

        # 遍历束
        for i_beam, r_m in enumerate(dist_arr):
            if r_m < 0.01:
                continue
            alpha_deg = angles_deg[i_beam]
            alpha_rad = math.radians(alpha_deg)

            # 加雷达相对车体安装偏移
            # 例如雷达 0° 指向车左侧 => offset=-π/2，把它转到车头方向
            beam_world_angle = rtheta + alpha_rad + LASER_OFFSET_RAD

            wx = rx + r_m * math.cos(beam_world_angle)
            wy = ry + r_m * math.sin(beam_world_angle)
            all_points.append((wx, wy))

    if not all_points:
        print("[Warn] No valid points!")
        return None, 0, 0, 0

    # 5) 动态地图范围
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    margin = 5.0
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    resolution = 0.1
    width  = int((max_x - min_x)/resolution) + 1
    height = int((max_y - min_y)/resolution) + 1
    print(f"Map bounding box: x=[{min_x:.1f},{max_x:.1f}], y=[{min_y:.1f},{max_y:.1f}]")
    print(f"Map size={width}x{height}, resolution={resolution:.2f}")

    # 6) 投票
    hit_map = np.zeros((height, width), dtype=np.float32)
    def world_to_map(wx, wy):
        ix = int((wx - min_x)/resolution)
        iy = int((wy - min_y)/resolution)
        return ix, iy

    for (wx, wy) in all_points:
        ix, iy = world_to_map(wx, wy)
        if 0 <= ix < width and 0 <= iy < height:
            hit_map[iy, ix] += 1.0

    hit_map[hit_map > 20] = 20  # 防止过亮

    return hit_map, min_x, min_y, resolution

def main():
    lms_file = "URG_X_20130903_195003.lms"
    nav_file = "ld.nav"  # 你的nav文件

    hit_map, origin_x, origin_y, resolution = lidar_mapping(lms_file, nav_file)
    if hit_map is None:
        print("[Err] Building map failed.")
        return


    h, w = hit_map.shape
    extent = [origin_x, origin_x + w*resolution, origin_y, origin_y + h*resolution]

    plt.figure("Occupancy Grid Map", figsize=(6,6))
    im = plt.imshow(hit_map, origin='lower', extent=extent, cmap='viridis')
    plt.colorbar(im, label="Hit count")
    plt.title("Occupancy Grid Map")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    out_png = "lidar_grid_map.png"
    plt.savefig(out_png, dpi=150)
    print(f"[Info] Map saved to {out_png}")
    plt.show()

if __name__ == "__main__":
    main()
