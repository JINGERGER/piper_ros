#!/usr/bin/env python3
"""
简单的Piper机器人位姿控制示例
使用基础的ROS2话题和服务接口
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from math import pi, cos, sin, sqrt

class SimplePiperController(Node):
    def __init__(self):
        super().__init__('simple_piper_controller')
        
        # 发布关节状态话题
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 发布目标位姿话题 (如果有相应的控制器监听)
        self.pose_pub = self.create_publisher(PoseStamped, '/piper/target_pose', 10)
        
        # 订阅当前关节状态
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        self.get_logger().info("Simple Piper Controller 已启动!")
    
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        self.current_joint_state = msg
    
    def publish_joint_positions(self, joint_positions, joint_names=None):
        """
        发布关节位置命令
        Args:
            joint_positions: list, 关节角度列表 (弧度)
            joint_names: list, 关节名称列表
        """
        if joint_names is None:
            # 默认Piper关节名称 (根据您的URDF调整)
            joint_names = [
                'joint1', 'joint2', 'joint3', 
                'joint4', 'joint5', 'joint6'
            ]
        
        if len(joint_positions) != len(joint_names):
            self.get_logger().error(f"关节数量不匹配: {len(joint_positions)} vs {len(joint_names)}")
            return
        
        # 创建关节状态消息
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = joint_names
        joint_msg.position = joint_positions
        joint_msg.velocity = [0.0] * len(joint_positions)
        joint_msg.effort = [0.0] * len(joint_positions)
        
        # 发布
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f"发布关节位置: {[f'{pos:.3f}' for pos in joint_positions]}")
    
    def publish_target_pose(self, pose):
        """
        发布目标位姿
        Args:
            pose: geometry_msgs/Pose 目标位姿
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = pose
        
        self.pose_pub.publish(pose_stamped)
        self.get_logger().info(f"发布目标位姿: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
    
    def create_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        创建位姿对象
        Args:
            x, y, z: 位置坐标
            qx, qy, qz, qw: 四元数 (默认无旋转)
        Returns:
            geometry_msgs/Pose
        """
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        欧拉角转四元数
        Args:
            roll, pitch, yaw: 欧拉角 (弧度)
        Returns:
            tuple: (qx, qy, qz, qw)
        """
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return (qx, qy, qz, qw)
    
    def move_to_joint_positions(self, positions, duration=5.0):
        """
        平滑移动到关节位置
        Args:
            positions: list, 目标关节位置
            duration: float, 运动时间 (秒)
        """
        if self.current_joint_state is None:
            self.get_logger().warn("未接收到当前关节状态，使用零位置作为起始点")
            start_positions = [0.0] * len(positions)
        else:
            start_positions = list(self.current_joint_state.position)[:len(positions)]
        
        # 确保起始位置长度匹配
        while len(start_positions) < len(positions):
            start_positions.append(0.0)
        
        steps = int(duration * 50)  # 50Hz控制频率
        
        for i in range(steps + 1):
            # 线性插值
            t = i / steps
            current_positions = []
            
            for start, target in zip(start_positions, positions):
                current_pos = start + (target - start) * t
                current_positions.append(current_pos)
            
            # 发布当前位置
            self.publish_joint_positions(current_positions)
            
            # 等待
            time.sleep(0.02)  # 50Hz
        
        self.get_logger().info("关节运动完成!")

def main():
    rclpy.init()
    
    # 创建控制器节点
    controller = SimplePiperController()
    
    try:
        # 等待一下让系统初始化
        time.sleep(1)
        
        # 示例1: 发布目标位姿
        controller.get_logger().info("=== 示例1: 发布目标位姿 ===")
        
        # 创建目标位姿 (机器人前方30cm，高度40cm)
        qx, qy, qz, qw = controller.euler_to_quaternion(0, pi/2, 0)  # 末端向下
        target_pose = controller.create_pose(
            x=0.3, y=0.0, z=0.4,
            qx=qx, qy=qy, qz=qz, qw=qw
        )
        
        # 发布位姿 (需要有相应的控制器监听这个话题)
        controller.publish_target_pose(target_pose)
        
        time.sleep(2)
        
        # 示例2: 直接控制关节
        controller.get_logger().info("=== 示例2: 关节空间控制 ===")
        
        # 定义一些预设关节位置
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零位置
        position1 = [0.0, -pi/4, pi/2, 0.0, pi/4, 0.0]   # 位置1
        position2 = [pi/6, -pi/6, pi/3, pi/6, pi/6, pi/6] # 位置2
        
        # 移动序列
        positions = [home_position, position1, position2, home_position]
        position_names = ["零位置", "位置1", "位置2", "返回零位置"]
        
        for pos, name in zip(positions, position_names):
            controller.get_logger().info(f"移动到 {name}")
            controller.move_to_joint_positions(pos, duration=3.0)
            time.sleep(1)  # 暂停1秒
        
        # 示例3: 发布更多目标位姿
        controller.get_logger().info("=== 示例3: 多个目标位姿 ===")
        
        poses = [
            (0.25, 0.1, 0.35, "位置A"),
            (0.25, -0.1, 0.35, "位置B"), 
            (0.2, 0.0, 0.3, "位置C")
        ]
        
        for x, y, z, name in poses:
            controller.get_logger().info(f"发布目标位姿: {name}")
            qx, qy, qz, qw = controller.euler_to_quaternion(0, pi/3, 0)
            pose = controller.create_pose(x, y, z, qx, qy, qz, qw)
            controller.publish_target_pose(pose)
            time.sleep(2)
        
        controller.get_logger().info("所有示例完成!")
        
        # 保持节点运行以继续发布
        controller.get_logger().info("节点将保持运行，按Ctrl+C退出...")
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("接收到中断信号，正在退出...")
    except Exception as e:
        controller.get_logger().error(f"发生错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
