# 导入所需的消息类型和导航器类
from geometry_msgs.msg import PoseStamped  # 用于定义位置和姿态的消息类型
from nav2_simple_commander.robot_navigator import BasicNavigator  # 导航器类，用于控制机器人导航
import rclpy  # ROS2的Python客户端库

def main():
    rclpy.init()

    # 创建导航器对象
    navigator = BasicNavigator()
    # 创建初始位姿（位置 + 姿态）
    initial_pose = PoseStamped()
    
    # 设置位姿的参考坐标系为“map”（地图坐标系）
    initial_pose.header.frame_id = 'map'
    # 设置位姿的时间戳为当前时间
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # 设置初始位置的坐标（0.0, 0.0），即机器人的起始位置
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    
    # 设置初始姿态为单位四元数（1.0, 0.0, 0.0, 0.0），表示机器人朝向正北（没有旋转）
    initial_pose.pose.orientation.w = 1.0
    
    # 设置机器人的初始位姿
    navigator.setInitialPose(initial_pose)
    
    # 等待直到导航2（Nav2）启动并准备好
    navigator.waitUntilNav2Active()
    
    # 启动ROS2事件循环（保持节点运行）
    rclpy.spin(navigator)
    
    # 关闭ROS2节点
    rclpy.shutdown()

# 仅在脚本被直接执行时调用main函数
if __name__ == '__main__':
    main()
