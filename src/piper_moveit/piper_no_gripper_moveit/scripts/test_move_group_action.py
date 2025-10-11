#!/usr/bin/env python3
"""
测试MoveGroup Action的简单脚本
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    JointConstraint,
    PlanningOptions,
    RobotState
)
import time

class TestMoveGroupClient(Node):
    def __init__(self):
        super().__init__('test_move_group_client')
        
        # 创建action客户端
        self._action_client = ActionClient(self, MoveGroup, '/move_group')
        
        self.get_logger().info('等待MoveGroup action服务器...')
        
        # 等待服务器启动
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('✓ MoveGroup action服务器已连接!')
        else:
            self.get_logger().error('✗ MoveGroup action服务器不可用!')
            return

    def send_joint_goal(self):
        """发送关节目标位置"""
        
        # 创建目标消息
        goal_msg = MoveGroup.Goal()
        
        # 设置规划请求
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'arm'  # 从SRDF确认的组名
        goal_msg.request.pipeline_id = 'ompl'
        goal_msg.request.planner_id = 'RRTConnect'
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # 设置起始状态（使用当前状态）
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.is_diff = True
        
        # 创建关节约束 - 移动到零位置
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零位置
        
        constraints = Constraints()
        constraints.name = "joint_goal"
        
        for joint_name, position in zip(joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.05
            joint_constraint.tolerance_below = 0.05
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # 设置规划选项
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = True  # 先只规划，不执行
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.look_around_attempts = 0
        goal_msg.planning_options.max_safe_execution_cost = 0.0
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.replan_attempts = 0
        goal_msg.planning_options.replan_delay = 0.0
        
        # 发送目标
        self.get_logger().info('发送运动规划请求...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 等待goal被接受
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('✗ 运动规划请求被拒绝!')
            return False
        
        self.get_logger().info('✓ 运动规划请求已接受，等待结果...')
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
        
        result = get_result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ 运动规划成功!')
            self.get_logger().info(f'规划时间: {result.planning_time:.3f}秒')
            trajectory = result.trajectory
            if trajectory.joint_trajectory.points:
                self.get_logger().info(f'轨迹包含 {len(trajectory.joint_trajectory.points)} 个点')
                self.get_logger().info('轨迹规划成功，可以执行!')
                return True
        else:
            self.get_logger().error(f'✗ 运动规划失败，错误代码: {result.error_code.val}')
            return False

def main():
    rclpy.init()
    
    client = TestMoveGroupClient()
    
    try:
        # 等待一下让系统初始化
        time.sleep(1)
        
        # 测试关节运动规划
        success = client.send_joint_goal()
        
        if success:
            print("\n=== 测试成功! ===")
            print("MoveGroup action服务器工作正常")
            print("可以使用以下命令执行实际运动:")
            print("将 plan_only: true 改为 plan_only: false")
        else:
            print("\n=== 测试失败! ===")
            print("请检查MoveIt配置")
            
    except KeyboardInterrupt:
        client.get_logger().info('接收到中断信号')
    except Exception as e:
        client.get_logger().error(f'错误: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
