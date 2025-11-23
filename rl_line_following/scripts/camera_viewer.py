#!/usr/bin/env python3
"""
摄像头图像查看器 - 订阅并显示Gazebo中的摄像头图像
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer')
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 订阅摄像头话题 - 使用实际的话题名称
        # 尝试多个可能的话题名称
        self.rgb_sub = rospy.Subscriber('/depth_camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/depth_camera/depth/image_raw', Image, self.depth_callback)
        
        rospy.loginfo("订阅RGB话题: /depth_camera/rgb/image_raw")
        rospy.loginfo("订阅深度话题: /depth_camera/depth/image_raw")
        
        rospy.loginfo("摄像头查看器已启动，等待图像...")
        
    def rgb_callback(self, msg):
        """处理RGB图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 显示图像
            cv2.imshow("RGB Camera View", cv_image)
            cv2.waitKey(3)  # 刷新窗口
            
        except Exception as e:
            rospy.logerr(f"处理RGB图像时出错: {e}")
    
    def depth_callback(self, msg):
        """处理深度图像"""
        try:
            # 将ROS深度图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            
            # 检查图像是否有效
            if cv_image is None or cv_image.size == 0:
                return
            
            # 归一化显示
            cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image_normalized = cv_image_normalized.astype('uint8')
            
            # 应用颜色映射以便更好观察
            cv_image_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
            
            # 显示图像
            cv2.imshow("Depth Camera View", cv_image_colormap)
            cv2.waitKey(3)
            
        except Exception as e:
            rospy.logerr(f"处理深度图像时出错: {e}")
    
    def run(self):
        """运行主循环"""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass

