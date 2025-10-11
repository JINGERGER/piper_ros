#!/usr/bin/env python3
"""
Piper Direct Trajectory Control - Bypasses MoveIt Planning
This script directly sends joint trajectory commands to the controller,
providing both joint-space and pose-mapped control without MoveIt planning.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from math import pi, sin, cos, atan2, sqrt
import time
import threading

class PiperArmDirect:
    """直接控制版本的PiperArm - 类似样例接口但绕过MoveIt"""
    
    def __init__(self, node):
        self.node = node
        # 直接连接到关节轨迹控制器
        self._trajectory_client = ActionClient(
            node, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 订阅关节状态
        self._joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint_positions = None
        
        # 等待轨迹控制器
        node.get_logger().info('等待关节轨迹控制器...')
        if self._trajectory_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().info('✓ 关节轨迹控制器已连接')
        else:
            node.get_logger().error('✗ 关节轨迹控制器不可用')
        
        # 等待关节状态
        node.get_logger().info('等待关节状态...')
        timeout = 10.0
        start_time = time.time()
        while self.current_joint_positions is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if self.current_joint_positions is not None:
            node.get_logger().info('✓ 已获取关节状态')
        else:
            node.get_logger().error('✗ 未能获取关节状态')
    
    def _joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])
    
    def set_start_state_to_current_state(self):
        """设置起始状态为当前状态 - 类似样例"""
        self.node.get_logger().info("起始状态设为当前状态")
        if self.current_joint_positions:
            self.node.get_logger().info(f"当前关节位置: {[f'{pos:.3f}' for pos in self.current_joint_positions]}")
    
    def set_joint_goal(self, joint_positions):
        """设置关节目标 - 类似样例"""
        self.goal_joints = joint_positions
        self.node.get_logger().info(f"设置关节目标: {[f'{pos:.3f}' for pos in joint_positions]}")
    
    def set_goal_state(self, pose_stamped_msg=None, pose_link="link6", joint_positions=None):
        """设置目标状态 - 兼容位姿和关节目标"""
        if joint_positions is not None:
            self.set_joint_goal(joint_positions)
        elif pose_stamped_msg is not None:
            # 如果提供位姿，我们使用预定义的安全关节配置
            self.node.get_logger().info("收到位姿目标，使用预定义安全关节配置")
            self.node.get_logger().info(f"位置: x={pose_stamped_msg.pose.position.x:.3f}, "
                                       f"y={pose_stamped_msg.pose.position.y:.3f}, "
                                       f"z={pose_stamped_msg.pose.position.z:.3f}")
            
            # 根据位姿选择合适的关节配置
            x = pose_stamped_msg.pose.position.x
            y = pose_stamped_msg.pose.position.y
            z = pose_stamped_msg.pose.position.z
            
            # 简单的映射规则：根据目标位置选择关节配置
            if abs(y) < 0.05:  # 中心位置
                if z < 0.25:
                    joint_config = [0.0, -0.2, 0.4, 0.0, 0.2, 0.0]  # 低位置
                else:
                    joint_config = [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]  # 高位置
            elif y > 0:  # 右侧
                joint_config = [0.3, -0.3, 0.6, 0.0, 0.3, 0.0]
            else:  # 左侧
                joint_config = [-0.3, -0.3, 0.6, 0.0, 0.3, 0.0]
            
            self.set_joint_goal(joint_config)
        else:
            self.node.get_logger().error("未提供有效的目标!")
    
    def plan_and_execute(self):
        """规划并执行运动 - 直接发送轨迹"""
        if not hasattr(self, 'goal_joints'):
            self.node.get_logger().error("未设置关节目标!")
            return False
        
        if self.current_joint_positions is None:
            self.node.get_logger().error("未获取到当前关节状态!")
            return False
        
        try:
            # 创建轨迹消息
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # 创建轨迹点
            point = JointTrajectoryPoint()
            point.positions = self.goal_joints
            point.velocities = [0.0] * len(self.goal_joints)
            point.accelerations = [0.0] * len(self.goal_joints)
            
            # 计算运动时间（基于最大关节变化）
            max_joint_change = max(abs(goal - current) 
                                 for goal, current in zip(self.goal_joints, self.current_joint_positions))
            motion_time = max(2.0, max_joint_change * 3.0)  # 至少2秒，复杂运动更长
            
            point.time_from_start = Duration(sec=int(motion_time), nanosec=int((motion_time % 1) * 1e9))
            trajectory.points = [point]
            
            # 创建action目标
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            self.node.get_logger().info(f"发送轨迹命令，预计用时: {motion_time:.1f}秒")
            
            # 发送目标
            send_goal_future = self._trajectory_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.node.get_logger().error("✗ 发送轨迹超时")
                return False
            
            if not goal_handle.accepted:
                self.node.get_logger().error("✗ 轨迹被拒绝")
                return False
            
            self.node.get_logger().info("✓ 轨迹已接受，执行中...")
            
            # 等待执行完成
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=motion_time + 5.0)
            
            result_response = get_result_future.result()
            if result_response is None:
                self.node.get_logger().error("✗ 执行超时")
                return False
            
            result = result_response.result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.node.get_logger().info("✓ 轨迹执行成功!")
                return True
            else:
                self.node.get_logger().error(f"✗ 轨迹执行失败，错误代码: {result.error_code}")
                return False
        
        except Exception as e:
            self.node.get_logger().error(f"轨迹执行错误: {e}")
            return False

def plan_and_execute(piper, piper_arm, logger):
    """独立的规划执行函数 - 完全类似样例"""
    logger.info("开始直接轨迹规划和执行...")
    success = piper_arm.plan_and_execute()
    
    if success:
        logger.info("✓ 直接轨迹执行成功!")
    else:
        logger.error("✗ 直接轨迹执行失败!")
    
    return success

def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数"""
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return (qx, qy, qz, qw)

def main():
    rclpy.init()
    
    # 创建节点
    node = Node('piper_direct_control')
    logger = node.get_logger()
    
    # 创建机器人接口 (类似样例中的panda)
    piper = node
    
    # 创建臂接口 (直接控制版本，但保持样例接口)
    piper_arm = PiperArmDirect(node)
    
    try:
        time.sleep(2)  # 等待初始化
        
        logger.info("=== 使用直接轨迹控制 (绕过MoveIt) ===")
        
        # === 方法1: 关节控制 (最可靠) ===
        logger.info("\n=== 方法1: 关节空间控制 ===")
        
        # 1. 设置起始状态为当前状态
        piper_arm.set_start_state_to_current_state()
        
        # 2. 设置关节目标
        safe_joint_positions = [0.0, -0.2, 0.4, 0.0, 0.2, 0.0]  # 安全配置
        piper_arm.set_joint_goal(safe_joint_positions)
        
        # 3. 规划并执行
        success = plan_and_execute(piper, piper_arm, logger)
        
        if success:
            time.sleep(2)
            
            # 更多关节配置
            joint_configs = [
                {"name": "伸展位置", "joints": [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]},
                {"name": "左转位置", "joints": [0.5, -0.3, 0.6, 0.0, 0.3, 0.0]},
                {"name": "右转位置", "joints": [-0.5, -0.3, 0.6, 0.0, 0.3, 0.0]},
                {"name": "收起位置", "joints": [0.0, -0.1, 0.2, 0.0, 0.1, 0.0]},
                {"name": "回零位置", "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            ]
            
            for config in joint_configs:
                logger.info(f"\n=== 移动到 {config['name']} ===")
                
                piper_arm.set_start_state_to_current_state()
                piper_arm.set_joint_goal(config['joints'])
                success = plan_and_execute(piper, piper_arm, logger)
                
                if not success:
                    logger.warn("跳过剩余动作...")
                    break
                
                time.sleep(2)
        
        # === 方法2: 位姿控制 (使用预定义映射) ===
        logger.info("\n=== 方法2: 位姿控制 (映射到关节配置) ===")
        
        if success:
            # 创建位姿目标 (样例风格)
            example_poses = [
                {"name": "中心低位", "x": 0.3, "y": 0.0, "z": 0.2},
                {"name": "中心高位", "x": 0.3, "y": 0.0, "z": 0.4},
                {"name": "右侧位置", "x": 0.3, "y": 0.1, "z": 0.3},
                {"name": "左侧位置", "x": 0.3, "y": -0.1, "z": 0.3}
            ]
            
            for pose_info in example_poses:
                logger.info(f"\n=== 移动到 {pose_info['name']} ===")
                
                # 按样例风格创建位姿
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "base_link"
                pose_goal.header.stamp = node.get_clock().now().to_msg()
                
                pose_goal.pose.position.x = pose_info['x']
                pose_goal.pose.position.y = pose_info['y']
                pose_goal.pose.position.z = pose_info['z']
                
                # 水平朝向
                pose_goal.pose.orientation.x = 0.0
                pose_goal.pose.orientation.y = 0.0
                pose_goal.pose.orientation.z = 0.0
                pose_goal.pose.orientation.w = 1.0
                
                # 按样例风格执行
                piper_arm.set_start_state_to_current_state()
                piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
                success = plan_and_execute(piper, piper_arm, logger)
                
                if not success:
                    logger.warn("跳过剩余位姿...")
                    break
                
                time.sleep(2)
        
        logger.info("\n=== 直接控制演示完成! ===")
        logger.info("💡 直接轨迹控制绕过了MoveIt规划，更可靠!")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
