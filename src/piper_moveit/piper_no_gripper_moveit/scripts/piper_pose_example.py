#!/usr/bin/env python3
"""
简化的Piper位姿控制脚本 - 基于样例风格
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint,
    PlanningOptions, RobotState
)
from shape_msgs.msg import SolidPrimitive
import time
from math import pi, cos, sin

class PiperArm:
    """类似样例中的panda_arm接口"""
    
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(node, MoveGroup, '/move_action')
        
        # 等待服务器
        node.get_logger().info('等待move_group action服务器...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            node.get_logger().info('✓ move_group已连接')
        else:
            node.get_logger().error('✗ move_group不可用')
            
        self.current_state_set = False
    
    def set_start_state_to_current_state(self):
        """设置起始状态为当前状态 - 类似样例"""
        self.current_state_set = True
        self.node.get_logger().info("起始状态设为当前状态")
    
    def check_ik_solution(self, pose_stamped):
        """检查位姿是否有IK解"""
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import PositionIKRequest, RobotState
        
        # 创建IK客户端 (如果还没有)
        if not hasattr(self, '_ik_client'):
            self._ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
            if not self._ik_client.wait_for_service(timeout_sec=5.0):
                self.node.get_logger().warn("IK服务不可用，跳过检查")
                return True  # 如果服务不可用，假设可达
        
        try:
            # 创建IK请求
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'arm'
            request.ik_request.pose_stamped = pose_stamped
            request.ik_request.ik_link_name = self.goal_link
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout = rclpy.duration.Duration(seconds=3.0).to_msg()
            
            # 设置起始状态
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.is_diff = True
            
            # 调用IK服务
            future = self._ik_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    self.node.get_logger().info("✓ IK解存在")
                    return True
                else:
                    self.node.get_logger().warn(f"✗ IK无解，错误代码: {response.error_code.val}")
                    return False
            else:
                self.node.get_logger().warn("IK服务调用超时")
                return True  # 超时时假设可达
                
        except Exception as e:
            self.node.get_logger().warn(f"IK检查失败: {e}")
            return True  # 出错时假设可达
    
    def set_goal_state(self, pose_stamped_msg, pose_link="link6"):
        """设置目标状态 - 类似样例中的接口，增加IK检查"""
        self.goal_pose = pose_stamped_msg
        self.goal_link = pose_link
        
        self.node.get_logger().info(f"设置目标位姿: link={pose_link}")
        self.node.get_logger().info(f"位置: x={pose_stamped_msg.pose.position.x:.3f}, "
                                   f"y={pose_stamped_msg.pose.position.y:.3f}, "
                                   f"z={pose_stamped_msg.pose.position.z:.3f}")
        
        # 检查IK解
        if not self.check_ik_solution(pose_stamped_msg):
            self.node.get_logger().warn("⚠️  目标位姿可能无IK解，但仍会尝试规划")

    def plan_and_execute(self):
        """规划并执行运动 - 整合版本"""
        if not hasattr(self, 'goal_pose'):
            self.node.get_logger().error("未设置目标位姿!")
            return False
        
        try:
            # 创建MoveGroup goal
            goal_msg = MoveGroup.Goal()
            
            # 设置规划请求
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = 'arm'
            goal_msg.request.pipeline_id = 'ompl'
            goal_msg.request.planner_id = 'RRTConnect'
            goal_msg.request.allowed_planning_time = 10.0  # 增加规划时间
            goal_msg.request.num_planning_attempts = 10    # 增加尝试次数
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            
            # 设置起始状态
            goal_msg.request.start_state = RobotState()
            goal_msg.request.start_state.is_diff = True
            
            # 创建约束
            constraints = Constraints()
            constraints.name = "pose_goal"
            
            # 位置约束
            pos_constraint = PositionConstraint()
            pos_constraint.header = self.goal_pose.header
            pos_constraint.link_name = self.goal_link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # 创建约束区域 (更大的容差球体)
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.01]  # 1cm容差，更宽松
            
            pos_constraint.constraint_region.primitives = [sphere]
            pos_constraint.constraint_region.primitive_poses = [self.goal_pose.pose]
            pos_constraint.weight = 1.0
            
            # 方向约束 (更宽松的方向容差)
            orient_constraint = OrientationConstraint()
            orient_constraint.header = self.goal_pose.header
            orient_constraint.link_name = self.goal_link
            orient_constraint.orientation = self.goal_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.2  # 增加到0.2弧度
            orient_constraint.absolute_y_axis_tolerance = 0.2
            orient_constraint.absolute_z_axis_tolerance = 0.2
            orient_constraint.weight = 1.0
            
            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(orient_constraint)
            goal_msg.request.goal_constraints.append(constraints)
            
            # 规划选项
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False  # 执行运动
            
            # 发送目标
            self.node.get_logger().info("发送运动规划请求...")
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            # 等待接受
            rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.node.get_logger().error("✗ 发送目标超时，请检查MoveGroup服务器状态")
                return False
                
            if not goal_handle.accepted:
                self.node.get_logger().error("✗ 目标被拒绝")
                return False
            
            self.node.get_logger().info("✓ 目标已接受，执行中...")
            
            # 等待结果
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=30.0)
            
            result_response = get_result_future.result()
            if result_response is None:
                self.node.get_logger().error("✗ 等待执行结果超时")
                return False
                
            result = result_response.result
            if result.error_code.val == 1:  # SUCCESS
                self.node.get_logger().info("✓ 运动执行成功!")
                return True
            else:
                self.node.get_logger().error(f"✗ 运动失败，错误代码: {result.error_code.val}")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"执行错误: {e}")
            return False

def plan_and_execute(piper, piper_arm, logger):
    """独立的规划执行函数 - 完全类似样例"""
    logger.info("开始规划和执行...")
    success = piper_arm.plan_and_execute()
    
    if success:
        logger.info("✓ 规划和执行成功!")
    else:
        logger.error("✗ 规划和执行失败!")
    
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
    node = Node('piper_pose_example')
    logger = node.get_logger()
    
    # 创建机器人接口 (类似样例中的panda)
    piper = node  # 简化版本
    
    # 创建臂接口 (类似样例中的panda_arm)
    piper_arm = PiperArm(node)
    
    try:
        time.sleep(2)  # 等待初始化
        
        # === 完全按照样例的风格使用 ===
        
        # 1. 设置起始状态为当前状态
        piper_arm.set_start_state_to_current_state()
        
        # 2. 设置位姿目标 (PoseStamped消息)
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"  # 类似样例中的"panda_link0"
        pose_goal.header.stamp = node.get_clock().now().to_msg()
        
        # 设置位置 (非常保守的位置，接近机器人基座)
        pose_goal.pose.position.x = 0.2   # 前方20cm (非常保守)
        pose_goal.pose.position.y = 0.0   # 中心位置
        pose_goal.pose.position.z = 0.2   # 高度20cm (很低，很安全)
        
        # 设置方向 - 水平向前 (更简单的姿态)
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 1.0
        
        # 设置目标状态 (类似样例，指定末端执行器链接)
        piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        # 3. 规划并执行 (完全类似样例)
        success = plan_and_execute(piper, piper_arm, logger)
        
        if not success:
            logger.warn("第一个位姿失败，尝试另一个位姿...")
            
            # 尝试另一个更容易到达的位姿
            pose_goal.pose.position.x = 0.2
            pose_goal.pose.position.y = 0.0
            pose_goal.pose.position.z = 0.25
            
            # 水平向前的姿态
            pose_goal.pose.orientation.x = 0.0
            pose_goal.pose.orientation.y = 0.0
            pose_goal.pose.orientation.z = 0.0
            pose_goal.pose.orientation.w = 1.0
            
            piper_arm.set_start_state_to_current_state()
            piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
            plan_and_execute(piper, piper_arm, logger)
        
        # 极其保守的示例位姿 (确保在工作空间内)
        example_poses = [
            {"name": "右侧位置", "x": 0.2, "y": 0.05, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0},  # 很小的偏移
            {"name": "左侧位置", "x": 0.2, "y": -0.05, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}, # 很小的偏移
            {"name": "高一点位置", "x": 0.2, "y": 0.0, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0}   # 稍高一点
        ]
        
        for pose_info in example_poses:
            logger.info(f"\n=== 移动到 {pose_info['name']} ===")
            
            # 设置新位姿
            pose_goal.pose.position.x = pose_info['x']
            pose_goal.pose.position.y = pose_info['y'] 
            pose_goal.pose.position.z = pose_info['z']
            
            qx, qy, qz, qw = euler_to_quaternion(pose_info['roll'], pose_info['pitch'], pose_info['yaw'])
            pose_goal.pose.orientation.x = qx
            pose_goal.pose.orientation.y = qy
            pose_goal.pose.orientation.z = qz
            pose_goal.pose.orientation.w = qw
            
            # 按样例风格执行
            piper_arm.set_start_state_to_current_state()
            piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
            plan_and_execute(piper, piper_arm, logger)
            
            time.sleep(2)
        
        logger.info("所有示例完成!")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"程序错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
