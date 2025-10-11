#!/usr/bin/env python3
"""
Piper机器人位姿控制脚本
使用MoveIt2 Python API控制机器人末端执行器移动到指定位姿
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
import sys
import copy
from math import pi, tau, dist, fabs, cos, sqrt
import numpy as np

# 尝试导入MoveIt Python接口
try:
    from pymoveit2 import MoveIt2
    from pymoveit2.robots import piper
    PYMOVEIT2_AVAILABLE = True
except ImportError:
    PYMOVEIT2_AVAILABLE = False

class PiperPoseController(Node):
    def __init__(self):
        super().__init__('piper_pose_controller')
        
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建机器人接口
        self.robot = moveit_commander.RobotCommander()
        
        # 创建场景接口
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建move group接口（根据您的SRDF配置调整group名称）
        self.move_group = moveit_commander.MoveGroupCommander("arm")  # 正确的组名
        
        # 创建显示轨迹发布器
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, "/move_group/display_planned_path", 20
        )
        
        # 获取规划框架
        self.planning_frame = self.move_group.get_planning_frame()
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        
        # 获取末端执行器链接
        self.eef_link = self.move_group.get_end_effector_link()
        self.get_logger().info(f"End effector link: {self.eef_link}")
        
        # 获取机器人的组名
        group_names = self.robot.get_group_names()
        self.get_logger().info(f"Available Planning Groups: {group_names}")
        
        # 设置规划参数
        self.move_group.set_planning_time(10.0)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
    def go_to_pose_goal(self, target_pose):
        """
        移动到目标位姿
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
        """
        # 设置位姿目标
        self.move_group.set_pose_target(target_pose)
        
        # 规划运动
        self.get_logger().info("Planning motion to target pose...")
        success = self.move_group.go(wait=True)
        
        # 停止运动，确保没有残留运动
        self.move_group.stop()
        # 清除位姿目标
        self.move_group.clear_pose_targets()
        
        # 检查是否到达目标
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(target_pose, current_pose, 0.01)
    
    def go_to_joint_state(self, joint_goal):
        """
        移动到关节目标
        Args:
            joint_goal: list 关节角度列表
        """
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
    def plan_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        """
        规划笛卡尔路径
        Args:
            waypoints: list of geometry_msgs/Pose 路径点列表
            eef_step: float 末端执行器步长
            jump_threshold: float 跳跃阈值
        """
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, eef_step, jump_threshold
        )
        
        return plan, fraction
    
    def execute_plan(self, plan):
        """执行规划的轨迹"""
        return self.move_group.execute(plan, wait=True)
    
    def display_trajectory(self, plan):
        """显示轨迹"""
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
    
    def get_current_pose(self):
        """获取当前位姿"""
        return self.move_group.get_current_pose().pose
    
    def get_current_joint_values(self):
        """获取当前关节角度"""
        return self.move_group.get_current_joint_values()
    
    def all_close(self, goal, actual, tolerance):
        """检查是否接近目标位姿"""
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = self.pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = self.pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
        return True
    
    def pose_to_list(self, pose):
        """将Pose转换为列表"""
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

def main():
    rclpy.init()
    
    # 创建控制器节点
    controller = PiperPoseController()
    
    try:
        # 示例1: 移动到特定位姿
        controller.get_logger().info("Moving to target pose...")
        
        # 创建目标位姿
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0  # 保持垂直朝下
        pose_goal.position.x = 0.4     # 前方40cm
        pose_goal.position.y = 0.1     # 右侧10cm
        pose_goal.position.z = 0.4     # 高度40cm
        
        # 执行运动
        success = controller.go_to_pose_goal(pose_goal)
        
        if success:
            controller.get_logger().info("Successfully reached target pose!")
        else:
            controller.get_logger().error("Failed to reach target pose!")
        
        # 示例2: 获取当前位姿
        current_pose = controller.get_current_pose()
        controller.get_logger().info(f"Current pose: {current_pose}")
        
        # 示例3: 笛卡尔路径规划
        controller.get_logger().info("Planning Cartesian path...")
        
        waypoints = []
        wpose = controller.get_current_pose()
        
        # 创建路径点
        wpose.position.z -= 0.1  # 下降10cm
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.y += 0.1  # 右移10cm
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.z += 0.1  # 上升10cm
        waypoints.append(copy.deepcopy(wpose))
        
        # 规划路径
        (plan, fraction) = controller.plan_cartesian_path(waypoints)
        
        controller.get_logger().info(f"Cartesian path planned with {fraction*100}% success")
        
        if fraction > 0.9:  # 如果路径规划成功率超过90%
            controller.get_logger().info("Executing Cartesian path...")
            controller.execute_plan(plan)
        
    except Exception as e:
        controller.get_logger().error(f"Error occurred: {e}")
    
    finally:
        # 清理
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
