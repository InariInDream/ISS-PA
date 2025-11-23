#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import bisect
import matplotlib.pyplot as plt

DISTANCE_PER_COUNT = 0.003846154  # 1 Count ≈ 0.003846154 m
ENC_MAX_COUNT = 30000             # 编码器在 30000 处溢出
HALF_MAX = ENC_MAX_COUNT // 2     # 15000

def load_encoder_data(file_path):
    """ 读取编码器文件: E Millisec 1 Count """
    time_list = []
    count_list = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 4 and parts[0] == 'E':
                ms = int(parts[1])
                c  = int(parts[3])
                time_list.append(ms)
                count_list.append(c)
    return time_list, count_list

def load_imu_data(file_path):
    """
    读取 IMU 文件: IMU time 0 0 roll pitch yaw
    注意: 航向角在第7个字段(下标6)
    """
    imu_time_list = []
    yaw_list = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 7 and parts[0] == 'IMU':
                ms = int(parts[1])
                # 最后一个数据 -180° ~ 180° 才是航向角
                yaw_deg = float(parts[6])
                imu_time_list.append(ms)
                yaw_list.append(yaw_deg)
    return imu_time_list, yaw_list

def find_yaw_at(imu_time_list, yaw_list, t_enc):
    """ 在 imu_time_list 中找到与 t_enc 最近的 yaw(度) """
    idx = bisect.bisect_left(imu_time_list, t_enc)
    if idx <= 0:
        return yaw_list[0]
    if idx >= len(imu_time_list):
        return yaw_list[-1]
    before_t = imu_time_list[idx-1]
    after_t  = imu_time_list[idx]
    if abs(t_enc - before_t) < abs(t_enc - after_t):
        return yaw_list[idx-1]
    else:
        return yaw_list[idx]

def calc_trajectory(encoder_time, encoder_count, imu_time, imu_yaw):
    """
    根据编码器和IMU数据做2D航位推算, 并处理编码器溢出
    返回: [ (x0, y0, theta0), ... ] 
    """
    trajectory = []
    if not encoder_time:
        return trajectory

    # 初始位置
    x = 0.0
    y = 0.0
    # 初始航向: 根据第一个编码器时刻
    yaw_deg_init = find_yaw_at(imu_time, imu_yaw, encoder_time[0])
    theta = math.radians(yaw_deg_init)
    trajectory.append((x, y, theta))

    prev_count = encoder_count[0]

    for i in range(1, len(encoder_time)):
        t_enc = encoder_time[i]
        c_enc = encoder_count[i]

        # 计算脉冲差值, 并做溢出补偿
        dt_count = c_enc - prev_count
        prev_count = c_enc

        if dt_count > HALF_MAX:
            dt_count -= ENC_MAX_COUNT
        elif dt_count < -HALF_MAX:
            dt_count += ENC_MAX_COUNT

        # 里程增量
        dist = dt_count * DISTANCE_PER_COUNT

        # 查找最近IMU航向角
        yaw_deg = find_yaw_at(imu_time, imu_yaw, t_enc)
        theta = math.radians(yaw_deg)

        # 更新位置
        x += dist * math.cos(theta)
        y += dist * math.sin(theta)

        trajectory.append((x, y, theta))

    return trajectory

def main():
    encoder_file = "COMPort_X_20130903_195003.txt"
    imu_file = "InterSense_X_20130903_195003.txt"

    # 1) 加载数据
    encoder_time, encoder_count = load_encoder_data(encoder_file)
    imu_time, imu_yaw = load_imu_data(imu_file)

    if not encoder_time or not imu_time:
        print("[Warn] data empty, exit.")
        return

    # 2) 航位推算
    trajectory = calc_trajectory(encoder_time, encoder_count, imu_time, imu_yaw)
    print(f"[Info] Trajectory computed: total {len(trajectory)} points.")

    # 3) 绘图
    xs = [pt[0] for pt in trajectory]
    ys = [pt[1] for pt in trajectory]

    plt.figure(figsize=(6,6))
    plt.plot(xs, ys, '-')
    plt.title("Dead Reckoning Trajectory")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.axis('equal')

    out_png = "dead_reckoning_trajectory.png"
    plt.savefig(out_png, dpi=150)
    print(f"[Info] Saved to {out_png}")

    # plt.show()

if __name__ == "__main__":
    main()
