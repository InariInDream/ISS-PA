# Rviz路径跟踪和摄像头查看功能使用说明

本包现在支持以下两个功能：
1. 在Rviz上绘制路径，小车按照路径行驶
2. 查看Gazebo中的摄像头图像信息

## 文件说明

### 新创建的脚本文件
- `scripts/rviz_path_planner.py` - 路径规划器，在Rviz上绘制并发布路径
- `scripts/path_tracker.py` - 路径跟踪控制器，让小车按照路径行驶
- `scripts/camera_viewer.py` - 摄像头图像查看器

### Launch文件
- `launch/rviz_path_following.launch` - 集成所有功能的启动文件

## 使用方法

### 1. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make  # 或者 catkin build
source devel/setup.bash
```

### 2. 启动功能

```bash
roslaunch rl_line_following rviz_path_following.launch
```

这将启动：
- Gazebo仿真环境
- Rviz可视化界面
- 路径规划器
- 路径跟踪器
- 摄像头图像查看器

### 3. 查看摄像头图像

摄像头图像会在新窗口中显示：
- RGB图像窗口显示彩色图像
- 深度图像窗口显示深度信息（带颜色映射）

如果看不到图像，请检查摄像头话题是否发布：
```bash
rostopic list | grep camera
rostopic list | grep rgb
rostopic list | grep depth
```

可能的话题名称：
- `/rgb/image_raw` - RGB图像
- `/depth/image_raw` - 深度图像
- `/rgb/image_raw` - Kinect相机RGB
- `/camera/rgb/image_raw` - 另一种可能的命名

如果话题名称不同，请修改 `scripts/camera_viewer.py` 中的话题名称。

### 4. 查看路径跟踪

在Rviz中：
1. 添加 "Marker" 显示类型，订阅 `/planner/markers` 话题查看路径点
2. 添加 "Path" 显示类型，订阅 `/planner/path` 话题查看路径
3. 路径会自动发布，小车会按照路径行驶

当前路径点是预定义的（见 `rviz_path_planner.py`），包括：
- (0, 0)
- (2, 0)
- (2, 2)
- (0, 2)
- (0, 0)

### 5. 修改路径点

编辑 `scripts/rviz_path_planner.py` 的 `main` 函数部分，修改路径点：

```python
planner.add_point(1.0, 1.0)  # 添加点(1, 1)
planner.add_point(3.0, 1.0)  # 添加点(3, 1)
planner.add_point(3.0, 3.0)  # 添加点(3, 3)
```

### 6. 调整控制参数

可以在launch文件中调整路径跟踪参数：

编辑 `launch/rviz_path_following.launch`，在path_tracker节点处添加参数：

```xml
<node pkg="rl_line_following" type="path_tracker.py" name="path_tracker" output="screen">
    <param name="max_linear_speed" value="0.5" />
    <param name="max_angular_speed" value="1.0" />
    <param name="lookahead_distance" value="0.3" />
    <param name="goal_tolerance" value="0.1" />
    <param name="kp_linear" value="0.5" />
    <param name="kp_angular" value="1.0" />
</node>
```

## 故障排除

### 问题1：看不到摄像头图像

解决方案：
1. 检查摄像头话题是否正确发布：
   ```bash
   rostopic echo /rgb/image_raw
   ```
2. 如果话题名不同，修改 `camera_viewer.py` 中的订阅话题
3. 确保安装了cv_bridge：
   ```bash
   sudo apt-get install ros-noetic-cv-bridge
   ```

### 问题2：小车不移动

解决方案：
1. 检查是否有路径发布：
   ```bash
   rostopic echo /planner/path
   ```
2. 检查控制命令是否发布：
   ```bash
   rostopic echo /cmd_vel
   ```
3. 确保odom话题正常工作：
   ```bash
   rostopic echo /odom
   ```

### 问题3：在Rviz中看不到路径

解决方案：
1. 在Rviz中添加显示类型：
   - Markers (订阅 /planner/markers)
   - Path (订阅 /planner/path)
2. 检查Fixed Frame是否设置为 "map"

## 高级用法

### 创建圆形路径

取消注释 `rviz_path_planner.py` 中的这行：
```python
planner.create_circle_path(0, 0, 2, 20)  # 中心(0,0)，半径2，20个点
```

### 动态添加路径点

可以通过服务调用来动态添加点（需要实现服务，当前为示例代码）

### 在仿真中实时控制

启动键盘控制：
```bash
roslaunch rl_line_following keyboard.launch
```

然后可以用路径跟踪或键盘控制。

## 摄像头话题配置

如果需要修改摄像头订阅话题，编辑 `scripts/camera_viewer.py`：

```python
# 修改这行
self.rgb_sub = rospy.Subscriber('/your_camera_topic', Image, self.rgb_callback)
```

常见的话题名：
- `/racebot/camera/rgb/image_raw` - 带命名空间的相机
- `/camera/rgb/image_raw` - 标准格式
- `/rgb/image_raw` - 简化格式

