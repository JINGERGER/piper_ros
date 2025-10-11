#!/usr/bin/env python3
"""
Piper机器人微调控制 - 只做很小的关节移动
基于当前关节状态进行微小调整
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PlanningOptions, RobotState
)
import time
from math import pi

class PiperMicroController(Node):
    def __init__(self):
        super().__init__('piper_micro_controller')
        
        # Action客户端
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        
        # 等待服务器
        self.get_logger().info('等待move_group action服务器...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('✓ move_group已连接')
        else:
            self.get_logger().error('✗ move_group不可用')
            return
            
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        timeout = 10.0
        start_time = time.time()
        
        while self.current_joint_state is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            self.get_logger().error('无法获取关节状态')
            return
        
        self.get_logger().info('✓ 已获取关节状态')
    
    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.current_joint_state = msg
    
    def get_current_joint_positions(self):
        """获取当前关节位置"""
        if self.current_joint_state is None:
            return None
        
        # 按标准顺序排列关节位置
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        positions = []
        
        for joint_name in joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f"找不到关节 {joint_name}")
                return None
        
        return positions
    
    def move_to_joint_positions(self, target_positions, joint_names=None):
        """移动到关节位置"""
        if joint_names is None:
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
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
            goal_msg.request.max_velocity_scaling_factor = 0.05  # 非常慢的速度
            goal_msg.request.max_acceleration_scaling_factor = 0.05
            
            # 设置起始状态
            goal_msg.request.start_state = RobotState()
            goal_msg.request.start_state.is_diff = True
            
            # 创建关节约束
            constraints = Constraints()
            constraints.name = "joint_goal"
            
            for joint_name, position in zip(joint_names, target_positions):
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
            goal_msg.planning_options.plan_only = False
            
            # 显示目标位置
            self.get_logger().info("目标关节位置:")
            for name, pos in zip(joint_names, target_positions):
                self.get_logger().info(f"  {name}: {pos:.4f}")
            
            # 发送目标
            self.get_logger().info("发送微调运动请求...")
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            # 等待接受
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.get_logger().error("✗ 发送目标超时")
                return False
                
            if not goal_handle.accepted:
                self.get_logger().error("✗ 目标被拒绝")
                return False
            
            self.get_logger().info("✓ 目标已接受，执行中...")
            
            # 等待结果
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
            
            result_response = get_result_future.result()
            if result_response is None:
                self.get_logger().error("✗ 执行超时")
                return False
                
            result = result_response.result
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info("✓ 微调成功!")
                return True
            else:
                self.get_logger().error(f"✗ 微调失败，错误代码: {result.error_code.val}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"微调错误: {e}")
            return False
    
    def run_micro_movements(self):
        """运行微小移动测试"""
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            self.get_logger().error("无法获取当前关节位置")
            return
        
        self.get_logger().info("当前关节位置:")
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for name, pos in zip(joint_names, current_positions):
            self.get_logger().info(f"  {name}: {pos:.4f}")
        
        # 微小移动测试序列
        micro_movements = [
            {
                'name': '第一关节微调 +0.1弧度',
                'deltas': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            {
                'name': '第一关节回零',
                'deltas': [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            {
                'name': '第二关节微调 +0.1弧度',
                'deltas': [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]
            },
            {
                'name': '第二关节回零',
                'deltas': [0.0, -0.1, 0.0, 0.0, 0.0, 0.0]
            },
            {
                'name': '多关节小幅调整',
                'deltas': [0.05, 0.05, 0.05, 0.0, 0.0, 0.0]
            },
            {
                'name': '回到初始位置',
                'deltas': [-0.05, -0.05, -0.05, 0.0, 0.0, 0.0]
            }
        ]
        
        for movement in micro_movements:
            self.get_logger().info(f"\n=== {movement['name']} ===")
            
            # 计算目标位置
            target_positions = []
            for i, delta in enumerate(movement['deltas']):
                target_positions.append(current_positions[i] + delta)
            
            # 执行移动
            success = self.move_to_joint_positions(target_positions)
            
            if success:
                self.get_logger().info("✓ 微调成功")
                # 更新当前位置为目标位置
                current_positions = target_positions[:]
            else:
                self.get_logger().error("✗ 微调失败，停止后续测试")
                break
            
            time.sleep(2)  # 等待稳定
        
        self.get_logger().info("\n=== 微调测试完成 ===")

def main():
    rclpy.init()
    
    controller = PiperMicroController()
    
    try:
        controller.run_micro_movements()
        
    except KeyboardInterrupt:
        controller.get_logger().info("收到中断信号")
    except Exception as e:
        controller.get_logger().error(f"程序错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
