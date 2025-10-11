#!/usr/bin/env python3
"""
简单的Piper关节空间控制 - 更可靠的方法
使用关节目标而不是位姿目标，避免IK问题
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    JointConstraint, PlanningOptions, RobotState
)
import time
from math import pi

class PiperJointController:
    """基于关节空间的控制器"""
    
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(node, MoveGroup, '/move_action')
        
        # 等待服务器
        node.get_logger().info('等待move_group action服务器...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().info('✓ move_group已连接')
        else:
            node.get_logger().error('✗ move_group不可用')
            
        # 预定义的安全关节位置
        self.safe_positions = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "ready": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
            "side_right": [0.5, -0.3, 0.8, 0.0, 0.5, 0.0],
            "side_left": [-0.5, -0.3, 0.8, 0.0, 0.5, 0.0],
            "up": [0.0, -0.8, 0.5, 0.0, 1.3, 0.0],
            "forward": [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        }
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    def move_to_joint_positions(self, joint_positions, position_name="custom"):
        """移动到指定关节位置"""
        try:
            # 创建MoveGroup goal
            goal_msg = MoveGroup.Goal()
            
            # 设置规划请求
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = 'arm'
            goal_msg.request.pipeline_id = 'ompl'
            goal_msg.request.planner_id = 'RRTConnect'
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.max_velocity_scaling_factor = 0.2  # 稍微快一些
            goal_msg.request.max_acceleration_scaling_factor = 0.2
            
            # 设置起始状态
            goal_msg.request.start_state = RobotState()
            goal_msg.request.start_state.is_diff = True
            
            # 创建关节约束
            constraints = Constraints()
            constraints.name = f"joint_goal_{position_name}"
            
            for i, (joint_name, position) in enumerate(zip(self.joint_names, joint_positions)):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = position
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            
            goal_msg.request.goal_constraints.append(constraints)
            
            # 规划选项
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False  # 执行运动
            
            # 发送目标
            self.node.get_logger().info(f"发送关节目标: {position_name}")
            self.node.get_logger().info(f"关节位置: {[f'{pos:.3f}' for pos in joint_positions]}")
            
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            # 等待接受
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
            
            if send_goal_future.result() is None:
                self.node.get_logger().error("✗ 发送目标超时")
                return False
                
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error("✗ 关节目标被拒绝")
                return False
            
            self.node.get_logger().info("✓ 关节目标已接受，执行中...")
            
            # 等待结果
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=30.0)
            
            if get_result_future.result() is None:
                self.node.get_logger().error("✗ 执行超时")
                return False
                
            result = get_result_future.result().result
            if result.error_code.val == 1:  # SUCCESS
                self.node.get_logger().info(f"✓ 成功到达 {position_name}!")
                return True
            else:
                self.node.get_logger().error(f"✗ 运动失败，错误代码: {result.error_code.val}")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"关节控制错误: {e}")
            return False
    
    def move_to_named_position(self, position_name):
        """移动到预定义位置"""
        if position_name not in self.safe_positions:
            self.node.get_logger().error(f"未知位置: {position_name}")
            self.node.get_logger().info(f"可用位置: {list(self.safe_positions.keys())}")
            return False
        
        positions = self.safe_positions[position_name]
        return self.move_to_joint_positions(positions, position_name)
    
    def demo_sequence(self):
        """演示序列"""
        sequence = ["home", "ready", "side_right", "ready", "side_left", "ready", "up", "ready", "forward", "home"]
        
        for position_name in sequence:
            self.node.get_logger().info(f"\n=== 移动到 {position_name} ===")
            success = self.move_to_named_position(position_name)
            
            if not success:
                self.node.get_logger().warn(f"跳过 {position_name}，继续下一个...")
            
            time.sleep(2)  # 暂停2秒
        
        self.node.get_logger().info("演示序列完成!")

def main():
    rclpy.init()
    
    # 创建节点
    node = Node('piper_joint_controller')
    logger = node.get_logger()
    
    # 创建控制器
    controller = PiperJointController(node)
    
    try:
        time.sleep(2)  # 等待初始化
        
        logger.info("=== Piper关节空间控制演示 ===")
        
        # 方式1: 使用预定义位置
        logger.info("\n1. 移动到home位置")
        controller.move_to_named_position("home")
        
        time.sleep(2)
        
        logger.info("\n2. 移动到ready位置")
        controller.move_to_named_position("ready")
        
        time.sleep(2)
        
        # 方式2: 自定义关节位置
        logger.info("\n3. 移动到自定义位置")
        custom_joints = [0.3, -0.4, 0.6, 0.0, 0.8, 0.0]
        controller.move_to_joint_positions(custom_joints, "custom_demo")
        
        time.sleep(2)
        
        # 方式3: 运行演示序列
        logger.info("\n4. 运行完整演示序列")
        response = input("是否运行完整演示序列? (y/N): ")
        if response.lower() == 'y':
            controller.demo_sequence()
        
        # 交互式控制
        logger.info("\n5. 交互式关节控制")
        while True:
            print("\n" + "="*40)
            print("可用的预定义位置:")
            for name in controller.safe_positions.keys():
                positions = controller.safe_positions[name]
                print(f"  {name}: {[f'{p:.2f}' for p in positions]}")
            print("="*40)
            
            choice = input("输入位置名称 (或 'q' 退出): ").strip()
            
            if choice.lower() == 'q':
                break
            elif choice in controller.safe_positions:
                controller.move_to_named_position(choice)
            else:
                print(f"未知位置: {choice}")
        
        logger.info("程序结束")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
