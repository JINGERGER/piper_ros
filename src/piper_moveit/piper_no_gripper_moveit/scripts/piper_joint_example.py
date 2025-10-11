#!/usr/bin/env python3
"""
基于样例风格的关节控制 - 更可靠的方法
类似 panda_arm.set_joint_goal() 的用法
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

class PiperArmJoint:
    """类似样例中的panda_arm接口，但使用关节控制"""
    
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(node, MoveGroup, '/move_action')
        
        # 等待服务器
        node.get_logger().info('等待move_group action服务器...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().info('✓ move_group已连接')
        else:
            node.get_logger().error('✗ move_group不可用')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    def set_start_state_to_current_state(self):
        """设置起始状态为当前状态 - 类似样例"""
        self.node.get_logger().info("起始状态设为当前状态")
    
    def set_joint_goal(self, joint_positions):
        """设置关节目标 - 类似样例中的set_joint_goal"""
        self.goal_joints = joint_positions
        self.node.get_logger().info(f"设置关节目标: {[f'{pos:.3f}' for pos in joint_positions]}")
    
    def plan_and_execute(self):
        """规划并执行关节运动"""
        if not hasattr(self, 'goal_joints'):
            self.node.get_logger().error("未设置关节目标!")
            return False
        
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
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            
            # 设置起始状态
            goal_msg.request.start_state = RobotState()
            goal_msg.request.start_state.is_diff = True
            
            # 创建关节约束
            constraints = Constraints()
            constraints.name = "joint_goal"
            
            for joint_name, position in zip(self.joint_names, self.goal_joints):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = position
                joint_constraint.tolerance_above = 0.05
                joint_constraint.tolerance_below = 0.05
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            
            goal_msg.request.goal_constraints.append(constraints)
            
            # 规划选项
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False  # 执行运动
            
            # 发送目标
            self.node.get_logger().info("发送关节运动规划请求...")
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            # 等待接受
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.node.get_logger().error("✗ 发送关节目标超时")
                return False
                
            if not goal_handle.accepted:
                self.node.get_logger().error("✗ 关节目标被拒绝")
                return False
            
            self.node.get_logger().info("✓ 关节目标已接受，执行中...")
            
            # 等待结果
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=20.0)
            
            result_response = get_result_future.result()
            if result_response is None:
                self.node.get_logger().error("✗ 关节运动超时")
                return False
                
            result = result_response.result
            if result.error_code.val == 1:  # SUCCESS
                self.node.get_logger().info("✓ 关节运动成功!")
                return True
            else:
                self.node.get_logger().error(f"✗ 关节运动失败，错误代码: {result.error_code.val}")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"关节控制错误: {e}")
            return False

def plan_and_execute(piper, piper_arm, logger):
    """独立的规划执行函数 - 完全类似样例"""
    logger.info("开始关节规划和执行...")
    success = piper_arm.plan_and_execute()
    
    if success:
        logger.info("✓ 关节规划和执行成功!")
    else:
        logger.error("✗ 关节规划和执行失败!")
    
    return success

def main():
    rclpy.init()
    
    # 创建节点
    node = Node('piper_joint_example')
    logger = node.get_logger()
    
    # 创建机器人接口 (类似样例中的panda)
    piper = node  # 简化版本
    
    # 创建臂接口 (类似样例中的panda_arm，但用关节控制)
    piper_arm = PiperArmJoint(node)
    
    try:
        time.sleep(2)  # 等待初始化
        
        logger.info("=== 按样例风格使用关节控制 ===")
        
        # === 完全按照样例的风格使用，但设置关节目标 ===
        
        # 1. 设置起始状态为当前状态
        piper_arm.set_start_state_to_current_state()
        
        # 2. 设置关节目标 (类似样例中的set_joint_goal)
        # 这相当于样例中的 panda_arm.set_joint_goal([0.0, -pi/4, 0.0, -3*pi/4, 0.0, pi/2, pi/4])
        joint_goal = [0.0, -pi/4, pi/2, 0.0, pi/4, 0.0]  # 一个安全的关节配置
        piper_arm.set_joint_goal(joint_goal)
        
        # 3. 规划并执行 (完全类似样例)
        success = plan_and_execute(piper, piper_arm, logger)
        
        if not success:
            logger.warn("第一个关节目标失败，尝试零位置...")
            
            # 尝试回到零位置
            piper_arm.set_start_state_to_current_state()
            piper_arm.set_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            plan_and_execute(piper, piper_arm, logger)
        
        # 更多关节配置示例
        joint_configs = [
            {"name": "收起位置", "joints": [0.0, -pi/4, pi/2, 0.0, pi/4, 0.0]},
            {"name": "伸展位置", "joints": [0.0, -pi/6, pi/3, 0.0, pi/6, 0.0]},
            {"name": "左转位置", "joints": [pi/4, -pi/6, pi/4, 0.0, pi/3, 0.0]},
            {"name": "右转位置", "joints": [-pi/4, -pi/6, pi/4, 0.0, pi/3, 0.0]},
            {"name": "回零位置", "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        ]
        
        for config in joint_configs:
            logger.info(f"\n=== 移动到 {config['name']} ===")
            
            # 按样例风格执行
            piper_arm.set_start_state_to_current_state()
            piper_arm.set_joint_goal(config['joints'])
            success = plan_and_execute(piper, piper_arm, logger)
            
            if not success:
                logger.warn(f"跳过剩余动作...")
                break
            
            time.sleep(2)
        
        logger.info("所有关节控制示例完成!")
        logger.info("💡 关节控制不需要IK求解，通常更可靠!")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
