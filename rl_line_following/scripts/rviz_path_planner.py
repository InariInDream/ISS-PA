#!/usr/bin/env python3
"""
交互式Rviz路径规划器 - 通过点击Rviz中的2D Nav Goal来添加路径点
"""
import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header, Empty
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
import math

class RvizPathPlanner:
    def __init__(self):
        rospy.init_node('rviz_path_planner')
        
        # 发布器
        self.path_pub = rospy.Publisher('/planner/path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/planner/markers', Marker, queue_size=10)
        
        # 订阅Rviz的2D Nav Goal（用户点击的目标点）
        # 注意：Rviz的2D Nav Goal工具发布的话题是 /move_base_simple/goal
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=10)
        
        rospy.loginfo("已订阅话题: /move_base_simple/goal")
        
        # 提供清空路径的服务
        self.clear_service = rospy.Service('/planner/clear_path', EmptySrv, self.clear_path_service)
        
        # 存储路径点
        self.path_points = []
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("交互式Rviz路径规划器已启动！")
        rospy.loginfo("=" * 60)
        rospy.loginfo("使用方法：")
        rospy.loginfo("1. 在Rviz中点击 '2D Nav Goal' 工具（工具栏上方）")
        rospy.loginfo("2. 在地图上点击任意位置添加路径点")
        rospy.loginfo("3. 可以连续点击多个点来创建路径")
        rospy.loginfo("4. 调用服务 '/planner/clear_path' 来清空路径")
        rospy.loginfo("5. 在Rviz中添加显示类型：")
        rospy.loginfo("   - Marker (订阅 /planner/markers) 查看路径点")
        rospy.loginfo("   - Path (订阅 /planner/path) 查看路径连线")
        rospy.loginfo("=" * 60)
        
        # 启动定时器发布标记
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_markers)
        
    def publish_markers(self, event):
        """发布可视化的标记点"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # 红色
        
        # 添加所有路径点
        for point in self.path_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.1
            marker.points.append(p)
        
        self.marker_pub.publish(marker)
    
    def add_point(self, x, y):
        """添加一个路径点"""
        self.path_points.append((x, y))
        rospy.loginfo(f"添加路径点: ({x}, {y})")
        self.publish_path()
    
    def publish_path(self):
        """发布Path消息"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        from geometry_msgs.msg import PoseStamped
        for i, point in enumerate(self.path_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # 计算朝向（指向下一个点）
            if i < len(self.path_points) - 1:
                next_point = self.path_points[i + 1]
                dx = next_point[0] - point[0]
                dy = next_point[1] - point[1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            
            # 四元数
            import tf.transformations as tft
            quat = tft.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def goal_callback(self, msg):
        """处理用户在Rviz中点击的2D Nav Goal"""
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"收到2D Nav Goal消息！")
            rospy.loginfo(f"  Frame: {msg.header.frame_id}")
            rospy.loginfo(f"  位置: ({x:.3f}, {y:.3f})")
            rospy.loginfo(f"  姿态: ({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})")
            
            # 添加路径点
            self.add_point(x, y)
            rospy.loginfo(f"✓ 成功添加路径点 #{len(self.path_points)}: ({x:.2f}, {y:.2f})")
            rospy.loginfo(f"当前路径共有 {len(self.path_points)} 个点")
            rospy.loginfo("=" * 60)
        except Exception as e:
            rospy.logerr(f"处理目标点时出错: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def clear_path(self):
        """清空路径"""
        self.path_points = []
        self.publish_path()  # 发布空路径
        rospy.loginfo("路径已清空")
    
    def clear_path_service(self, req):
        """服务回调：清空路径"""
        self.clear_path()
        return EmptyResponse()  # Empty服务返回空响应
        
    def create_circle_path(self, center_x, center_y, radius, num_points=20):
        """创建圆形路径"""
        self.path_points = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.path_points.append((x, y))
        self.publish_path()
        rospy.loginfo(f"创建了圆形路径，中心: ({center_x}, {center_y}), 半径: {radius}")

if __name__ == '__main__':
    try:
        planner = RvizPathPlanner()
        
        # 不再使用hardcode的路径点
        # 用户现在可以通过在Rviz中点击2D Nav Goal来添加路径点
        
        rospy.loginfo("等待用户在Rviz中点击路径点...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

