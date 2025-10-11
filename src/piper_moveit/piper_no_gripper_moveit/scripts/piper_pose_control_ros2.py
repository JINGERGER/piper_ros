#!/usr/bin/env python3
"""
Piper机器人位姿控制脚本 - ROS2版本
使用MoveIt2服务接口控制机器人末端执行器移动到指定位姿
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints,
    PositionConstraint, OrientationConstraint,
    PositionIKRequest, RobotState, MoveItErrorCodes
)
from moveit_msgs.srv import GetPositionIK
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from math import pi, cos, sin, sqrt

class PiperPoseController(Node):
    def __init__(self):
        super().__init__('piper_pose_controller')
        
        # 创建MoveGroup动作客户端
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # 创建IK服务客户端
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 发布关节状态
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 等待服务可用
        self.get_logger().info("等待MoveIt服务可用...")
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup动作服务不可用!")
            return
            
        if not self._ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK服务不可用!")
            return
            
        self.get_logger().info("MoveIt服务已连接!")
        
        # 机器人配置
        self.group_name = "arm"  # 正确的SRDF组名
        self.end_effector_link = "link6"  # 根据您的URDF调整
        self.planning_frame = "base_link"
        
    def move_to_pose(self, target_pose, planning_time=5.0):
        """
        移动到目标位姿
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
            planning_time: float 规划时间限制
        Returns:
            bool: 是否成功
        """
        # 创建运动规划请求
        goal_msg = MoveGroup.Goal()
        
        # 设置规划请求
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = planning_time
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # 设置目标位姿约束
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        # 创建位置约束
        position_constraint = moveit_msgs.msg.PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # 设置约束容差
        position_constraint.constraint_region.sphere_radius = 0.001
        
        # 创建方向约束
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        # 添加约束到请求
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        # 设置规划选项
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        self.get_logger().info(f"发送运动规划请求到位姿: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # 发送请求
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        
        # 等待结果
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("运动规划请求被拒绝!")
            return False
            
        self.get_logger().info("运动规划请求已接受，等待执行...")
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("成功到达目标位姿!")
            return True
        else:
            self.get_logger().error(f"运动执行失败，错误代码: {result.result.error_code.val}")
            return False
    
    def get_ik_solution(self, target_pose):
        """
        获取逆运动学解
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
        Returns:
            list: 关节角度列表，如果无解返回None
        """
        request = GetPositionIK.Request()
        
        # 设置IK请求
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state.joint_state.name = []
        request.ik_request.robot_state.joint_state.position = []
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.ik_link_name = self.end_effector_link
        
        # 调用服务
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            return response.solution.joint_state.position
        else:
            self.get_logger().warn(f"IK求解失败，错误代码: {response.error_code.val}")
            return None
    
    def create_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        创建位姿对象
        Args:
            x, y, z: 位置坐标
            roll, pitch, yaw: 欧拉角 (弧度)
        Returns:
            geometry_msgs/Pose
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # 欧拉角转四元数
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        return pose

def main():
    rclpy.init()
    
    # 创建控制器节点
    controller = PiperPoseController()
    
    try:
        # 示例1: 移动到特定位姿
        controller.get_logger().info("=== 示例1: 移动到目标位姿 ===")
        
        # 创建目标位姿 (位置: x=0.3m, y=0.1m, z=0.4m, 方向: 垂直向下)
        target_pose = controller.create_pose(
            x=0.3, y=0.1, z=0.4,  # 位置
            roll=0.0, pitch=pi/2, yaw=0.0  # 末端执行器垂直向下
        )
        
        # 首先检查IK解是否存在
        ik_solution = controller.get_ik_solution(target_pose)
        if ik_solution:
            controller.get_logger().info(f"找到IK解: {[f'{angle:.3f}' for angle in ik_solution]}")
            
            # 执行运动
            success = controller.move_to_pose(target_pose)
            if success:
                controller.get_logger().info("✓ 成功到达目标位姿!")
            else:
                controller.get_logger().error("✗ 移动失败!")
        else:
            controller.get_logger().error("✗ 目标位姿无IK解，请调整位姿参数!")
        
        # 等待一段时间
        time.sleep(2)
        
        # 示例2: 移动到另一个位姿
        controller.get_logger().info("=== 示例2: 移动到第二个位姿 ===")
        
        target_pose2 = controller.create_pose(
            x=0.25, y=-0.1, z=0.35,
            roll=0.0, pitch=pi/4, yaw=pi/6  # 稍微倾斜
        )
        
        ik_solution2 = controller.get_ik_solution(target_pose2)
        if ik_solution2:
            controller.get_logger().info(f"找到IK解: {[f'{angle:.3f}' for angle in ik_solution2]}")
            success = controller.move_to_pose(target_pose2)
            if success:
                controller.get_logger().info("✓ 成功到达第二个位姿!")
        else:
            controller.get_logger().error("✗ 第二个位姿无IK解!")
            
    except KeyboardInterrupt:
        controller.get_logger().info("接收到中断信号，正在退出...")
    except Exception as e:
        controller.get_logger().error(f"发生错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
