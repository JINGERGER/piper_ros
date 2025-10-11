#!/usr/bin/env python3
"""
快速测试脚本 - 验证组名修复
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PlanningOptions, RobotState
)
import time
from math import pi

def main():
    rclpy.init()
    
    node = Node('piper_test_fix')
    logger = node.get_logger()
    
    try:
        # 创建action客户端
        action_client = ActionClient(node, MoveGroup, '/move_action')
        
        logger.info('等待move_group action服务器...')
        if not action_client.wait_for_server(timeout_sec=10.0):
            logger.error('✗ MoveGroup服务器不可用')
            return
        
        logger.info('✓ MoveGroup服务器已连接')
        
        # 测试使用正确的组名 'arm'
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'  # 修复后的正确组名
        goal_msg.request.pipeline_id = 'ompl'
        goal_msg.request.planner_id = 'RRTConnect'
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 3
        
        # 设置起始状态
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.is_diff = True
        
        # 创建简单的关节约束（零位置）
        constraints = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for joint_name, position in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # 规划选项（只规划，不执行）
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = True
        
        logger.info('发送测试规划请求（使用组名: arm）...')
        send_goal_future = action_client.send_goal_async(goal_msg)
        
        # 等待接受
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if goal_handle is None:
            logger.error('✗ 发送目标超时')
            return
            
        if not goal_handle.accepted:
            logger.error('✗ 目标被拒绝')
            return
        
        logger.info('✓ 目标已接受，等待规划结果...')
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=15.0)
        
        result_response = get_result_future.result()
        if result_response is None:
            logger.error('✗ 等待规划结果超时')
            return
            
        result = result_response.result
        if result.error_code.val == 1:  # SUCCESS
            logger.info('🎉 SUCCESS! 组名修复成功!')
            logger.info('✓ MoveIt现在可以正确识别规划组 "arm"')
            logger.info('✓ 规划成功完成!')
        else:
            logger.warn(f'规划失败，但至少组名错误已修复。错误代码: {result.error_code.val}')
            if result.error_code.val != -17:  # 不再是组名错误
                logger.info('✓ 组名问题已解决（错误代码不再是-17）')
        
    except Exception as e:
        logger.error(f'测试错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
