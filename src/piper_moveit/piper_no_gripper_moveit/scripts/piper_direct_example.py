#!/usr/bin/env python3
"""
Piper机器人直接轨迹控制 - 完全绕过MoveIt规划
基于样例风格的API，但使用直接轨迹发送
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import time
from math import pi, cos, sin
import numpy as np

class PiperDirectArm:
    """直接轨迹控制版本的piper_arm接口"""
    
    def __init__(self, node):
        self.node = node
        
        # 创建轨迹跟踪action客户端
        self._trajectory_client = ActionClient(
            node, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 等待轨迹控制器
        node.get_logger().info('等待轨迹控制器...')
        if self._trajectory_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().info('✓ 轨迹控制器已连接')
        else:
            node.get_logger().error('✗ 轨迹控制器不可用')
            
        # 订阅关节状态
        self._joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_joint_positions = None
        self.current_state_set = False
        
        # 等待关节状态
        node.get_logger().info('等待关节状态...')
        start_time = time.time()
        while self.current_joint_positions is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if self.current_joint_positions is not None:
            node.get_logger().info('✓ 已获取关节状态')
        else:
            node.get_logger().warn('⚠️  未获取到关节状态，使用零位置')
            self.current_joint_positions = [0.0] * 6
    
    def _joint_state_callback(self, msg):
        """关节状态回调"""
        try:
            # 提取我们关心的关节位置
            positions = []
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    positions.append(msg.position[idx])
                else:
                    positions.append(0.0)  # 默认值
            
            self.current_joint_positions = positions
        except Exception as e:
            self.node.get_logger().warn(f"关节状态解析错误: {e}")
    
    def set_start_state_to_current_state(self):
        """设置起始状态为当前状态"""
        self.current_state_set = True
        self.node.get_logger().info("起始状态设为当前状态")
        if self.current_joint_positions:
            self.node.get_logger().info(f"当前关节位置: {[f'{pos:.3f}' for pos in self.current_joint_positions]}")
    
    def set_joint_goal(self, joint_positions):
        """设置关节目标 - 类似样例"""
        self.goal_joints = joint_positions
        self.node.get_logger().info(f"设置关节目标: {[f'{pos:.3f}' for pos in joint_positions]}")
    
    def set_goal_state(self, pose_stamped_msg=None, pose_link="link6", joint_positions=None):
        """设置目标状态 - 兼容位姿和关节设置"""
        if joint_positions is not None:
            self.set_joint_goal(joint_positions)
        elif pose_stamped_msg is not None:
            # 对于位姿目标，我们使用简化的IK或预设关节配置
            self.node.get_logger().info(f"设置位姿目标: link={pose_link}")
            self.node.get_logger().info(f"位置: x={pose_stamped_msg.pose.position.x:.3f}, "
                                       f"y={pose_stamped_msg.pose.position.y:.3f}, "
                                       f"z={pose_stamped_msg.pose.position.z:.3f}")
            
            # 简化版：根据位置选择合适的关节配置
            joint_config = self._pose_to_joint_config(pose_stamped_msg.pose)
            self.set_joint_goal(joint_config)
        else:
            self.node.get_logger().error("必须提供位姿或关节目标!")
    
    def _pose_to_joint_config(self, pose):
        """简化的位姿到关节配置映射"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # 根据位置选择合适的关节配置
        if abs(y) < 0.05:  # 中心位置
            if z < 0.3:
                return [0.0, -0.2, 0.5, 0.0, 0.3, 0.0]  # 低位置
            else:
                return [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]  # 高位置
        elif y > 0:  # 左侧
            return [0.3, -0.3, 0.6, 0.0, 0.4, 0.0]
        else:  # 右侧
            return [-0.3, -0.3, 0.6, 0.0, 0.4, 0.0]
    
    def plan_and_execute(self):
        """规划并执行 - 直接发送轨迹"""
        if not hasattr(self, 'goal_joints'):
            self.node.get_logger().error("未设置关节目标!")
            return False
        
        try:
            # 确保关节目标在安全范围内
            safe_joints = self._clamp_joints(self.goal_joints)
            
            # 创建轨迹消息
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # 起始点（当前位置）
            start_point = JointTrajectoryPoint()
            start_point.positions = self.current_joint_positions or [0.0] * 6
            start_point.velocities = [0.0] * 6
            start_point.time_from_start = Duration(sec=0, nanosec=0)
            
            # 目标点
            goal_point = JointTrajectoryPoint()
            goal_point.positions = safe_joints
            goal_point.velocities = [0.0] * 6
            goal_point.time_from_start = Duration(sec=3, nanosec=0)  # 3秒运动时间
            
            trajectory.points = [start_point, goal_point]
            
            # 创建action目标
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            # 发送轨迹
            self.node.get_logger().info("发送轨迹到控制器...")
            send_goal_future = self._trajectory_client.send_goal_async(goal_msg)
            
            # 等待接受
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
            
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.node.get_logger().error("✗ 轨迹发送超时")
                return False
                
            if not goal_handle.accepted:
                self.node.get_logger().error("✗ 轨迹被拒绝")
                return False
            
            self.node.get_logger().info("✓ 轨迹已接受，执行中...")
            
            # 等待完成
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=10.0)
            
            result_response = get_result_future.result()
            if result_response is None:
                self.node.get_logger().error("✗ 轨迹执行超时")
                return False
            
            result = result_response.result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.node.get_logger().info("✓ 轨迹执行成功!")
                # 更新当前位置
                self.current_joint_positions = safe_joints
                return True
            else:
                self.node.get_logger().error(f"✗ 轨迹执行失败，错误代码: {result.error_code}")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"轨迹执行错误: {e}")
            return False
    
    def _clamp_joints(self, joint_positions):
        """限制关节角度在安全范围内"""
        # 定义安全的关节限制（弧度）
        joint_limits = [
            (-pi, pi),      # joint1
            (-pi/2, pi/2),  # joint2  
            (-pi, pi),      # joint3
            (-pi, pi),      # joint4
            (-pi/2, pi/2),  # joint5
            (-pi, pi)       # joint6
        ]
        
        clamped = []
        for i, (pos, (min_limit, max_limit)) in enumerate(zip(joint_positions, joint_limits)):
            if pos < min_limit:
                self.node.get_logger().warn(f"关节{i+1}位置{pos:.3f}低于限制{min_limit:.3f}，已调整")
                clamped.append(min_limit)
            elif pos > max_limit:
                self.node.get_logger().warn(f"关节{i+1}位置{pos:.3f}高于限制{max_limit:.3f}，已调整")
                clamped.append(max_limit)
            else:
                clamped.append(pos)
        
        return clamped

def plan_and_execute(piper, piper_arm, logger):
    """样例风格的规划执行函数"""
    logger.info("开始轨迹规划和执行...")
    success = piper_arm.plan_and_execute()
    
    if success:
        logger.info("✓ 轨迹规划和执行成功!")
    else:
        logger.error("✗ 轨迹规划和执行失败!")
    
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
    node = Node('piper_direct_example')
    logger = node.get_logger()
    
    # 创建机器人接口
    piper = node
    piper_arm = PiperDirectArm(node)
    
    try:
        time.sleep(1)  # 等待初始化
        
        logger.info("=== Piper直接轨迹控制示例 ===")
        logger.info("完全绕过MoveIt规划，直接发送轨迹到控制器")
        
        # === 关节空间控制示例 ===
        
        logger.info("\n=== 关节空间控制 ===")
        
        # 1. 设置起始状态
        piper_arm.set_start_state_to_current_state()
        
        # 2. 设置关节目标
        safe_joint_goal = [0.0, -0.2, 0.5, 0.0, 0.3, 0.0]  # 安全的关节配置
        piper_arm.set_joint_goal(safe_joint_goal)
        
        # 3. 执行
        success = plan_and_execute(piper, piper_arm, logger)
        
        if success:
            time.sleep(2)
            
            # 更多关节配置
            joint_configs = [
                {"name": "左转", "joints": [0.3, -0.2, 0.5, 0.0, 0.3, 0.0]},
                {"name": "右转", "joints": [-0.3, -0.2, 0.5, 0.0, 0.3, 0.0]},
                {"name": "回中心", "joints": [0.0, -0.1, 0.3, 0.0, 0.2, 0.0]},
            ]
            
            for config in joint_configs:
                logger.info(f"\n=== {config['name']} ===")
                piper_arm.set_start_state_to_current_state()
                piper_arm.set_joint_goal(config['joints'])
                
                success = plan_and_execute(piper, piper_arm, logger)
                if not success:
                    logger.warn("跳过剩余动作...")
                    break
                
                time.sleep(2)
        
        # === 位姿控制示例（简化版）===
        
        logger.info("\n=== 位姿控制示例（简化版）===")
        
        # 创建位姿目标
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = node.get_clock().now().to_msg()
        
        pose_examples = [
            {"name": "中心位置", "x": 0.3, "y": 0.0, "z": 0.3},
            {"name": "左侧位置", "x": 0.25, "y": 0.1, "z": 0.3},
            {"name": "右侧位置", "x": 0.25, "y": -0.1, "z": 0.3},
        ]
        
        for pose_info in pose_examples:
            logger.info(f"\n=== {pose_info['name']} ===")
            
            # 设置位姿
            pose_goal.pose.position.x = pose_info['x']
            pose_goal.pose.position.y = pose_info['y']
            pose_goal.pose.position.z = pose_info['z']
            pose_goal.pose.orientation.w = 1.0  # 简单姿态
            
            # 按样例风格执行
            piper_arm.set_start_state_to_current_state()
            piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
            
            success = plan_and_execute(piper, piper_arm, logger)
            if not success:
                logger.warn("跳过剩余位姿...")
                break
            
            time.sleep(2)
        
        logger.info("\n✅ 所有示例完成!")
        logger.info("💡 直接轨迹控制避免了MoveIt规划问题，更加可靠!")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
