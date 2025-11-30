#!/usr/bin/env python3
"""
可视化增强器 - 在Rviz中显示更多信息（速度、加速度、轨迹等）
"""
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import math

class VisualizationEnhancer:
    def __init__(self):
        rospy.init_node('visualization_enhancer')
        
        # 订阅
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布
        self.marker_pub = rospy.Publisher('/visualization/markers', MarkerArray, queue_size=10)
        
        # 状态
        self.current_odom = None
        self.velocity_history = []
        self.max_history = 50
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_markers)
        
        rospy.loginfo("可视化增强器已启动")
    
    def odom_callback(self, msg):
        """更新odom数据"""
        self.current_odom = msg
        
        # 记录速度历史
        speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.velocity_history.append(speed)
        if len(self.velocity_history) > self.max_history:
            self.velocity_history.pop(0)
    
    def publish_markers(self, event):
        """发布可视化标记"""
        if self.current_odom is None:
            return
        
        marker_array = MarkerArray()
        
        # 1. 速度向量标记
        speed = math.sqrt(self.current_odom.twist.twist.linear.x**2 + 
                         self.current_odom.twist.twist.linear.y**2)
        
        # 速度箭头
        speed_marker = Marker()
        speed_marker.header.frame_id = "map"
        speed_marker.header.stamp = rospy.Time.now()
        speed_marker.ns = "velocity"
        speed_marker.id = 0
        speed_marker.type = Marker.ARROW
        speed_marker.action = Marker.ADD
        
        # 箭头起点（机器人位置）
        start_point = Point()
        start_point.x = self.current_odom.pose.pose.position.x
        start_point.y = self.current_odom.pose.pose.position.y
        start_point.z = 0.2
        
        # 箭头终点（速度方向）
        import tf.transformations as tft
        quat = (
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z,
            self.current_odom.pose.pose.orientation.w
        )
        euler = tft.euler_from_quaternion(quat)
        yaw = euler[2]
        
        end_point = Point()
        end_point.x = start_point.x + speed * 2.0 * math.cos(yaw)  # 放大显示
        end_point.y = start_point.y + speed * 2.0 * math.sin(yaw)
        end_point.z = 0.2
        
        speed_marker.points = [start_point, end_point]
        speed_marker.scale = Vector3(0.1, 0.2, 0.1)  # 箭头粗细
        
        # 根据速度设置颜色（绿色=慢，红色=快）
        speed_normalized = min(speed / 1.2, 1.0)
        speed_marker.color = ColorRGBA(1.0 - speed_normalized, speed_normalized, 0.0, 1.0)
        
        marker_array.markers.append(speed_marker)
        
        # 2. 速度文本标记
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "speed_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = start_point.x
        text_marker.pose.position.y = start_point.y
        text_marker.pose.position.z = 0.5
        text_marker.scale.z = 0.3
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text_marker.text = f"Speed: {speed:.2f} m/s"
        
        marker_array.markers.append(text_marker)
        
        # 发布
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        enhancer = VisualizationEnhancer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

