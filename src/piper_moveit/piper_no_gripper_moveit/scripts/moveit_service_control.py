#!/usr/bin/env python3
"""
使用MoveIt2服务接口控制Piper机器人
通过GetMotionPlan服务获取轨迹，然后执行
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters, Constraints,
    PositionConstraint, OrientationConstraint, JointConstraint,
    PositionIKRequest, RobotState, MoveItErrorCodes
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time
from math import pi, cos, sin

class MoveItPiperController(Node):
    def __init__(self):
        super().__init__('moveit_piper_controller')
        
        # 创建服务客户端
        self.motion_plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 创建发布器
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/piper_controller/joint_trajectory', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 订阅当前状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # 等待服务可用
        self.get_logger().info("等待MoveIt服务...")
        if not self.motion_plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("运动规划服务不可用!")
            return
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK服务不可用!")
            return
            
        self.get_logger().info("MoveIt服务已连接!")
        
        # 机器人配置
        self.group_name = "arm"
        self.end_effector_link = "link6"
        self.base_frame = "base_link"
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        self.current_joint_state = None
    
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        self.current_joint_state = msg
    
    def get_motion_plan(self, target_pose, start_state=None):
        """
        获取到目标位姿的运动规划
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
            start_state: RobotState 起始状态 (可选)
        Returns:
            MotionPlanResponse 或 None
        """
        request = GetMotionPlan.Request()
        
        # 设置运动规划请求
        request.motion_plan_request.group_name = self.group_name
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.1
        request.motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        # 设置工作空间
        workspace = WorkspaceParameters()
        workspace.header.frame_id = self.base_frame
        workspace.min_corner.x = -1.0
        workspace.min_corner.y = -1.0
        workspace.min_corner.z = -0.5
        workspace.max_corner.x = 1.0
        workspace.max_corner.y = 1.0
        workspace.max_corner.z = 1.0
        request.motion_plan_request.workspace_parameters = workspace
        
        # 设置起始状态
        if start_state is None and self.current_joint_state is not None:
            start_state = RobotState()
            start_state.joint_state = self.current_joint_state
        
        if start_state is not None:
            request.motion_plan_request.start_state = start_state
        
        # 创建目标约束
        constraints = Constraints()
        constraints.name = "target_pose"
        
        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0  
        pos_constraint.target_point_offset.z = 0.0
        
        # 创建约束区域 (使用SolidPrimitive)
        from shape_msgs.msg import SolidPrimitive
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm容差
        
        pos_constraint.constraint_region.primitives = [sphere]
        pos_constraint.constraint_region.primitive_poses = [target_pose]
        pos_constraint.weight = 1.0
        
        # 方向约束
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.01
        orient_constraint.absolute_y_axis_tolerance = 0.01
        orient_constraint.absolute_z_axis_tolerance = 0.01
        orient_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        request.motion_plan_request.goal_constraints.append(constraints)
        
        self.get_logger().info(f"请求运动规划到位姿: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # 调用服务
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("运动规划成功!")
            return response.motion_plan_response
        else:
            self.get_logger().error(f"运动规划失败，错误代码: {response.motion_plan_response.error_code.val}")
            return None
    
    def execute_trajectory(self, trajectory):
        """
        执行轨迹
        Args:
            trajectory: JointTrajectory 关节轨迹
        """
        if not trajectory.points:
            self.get_logger().error("轨迹为空!")
            return False
        
        self.get_logger().info(f"执行轨迹，包含 {len(trajectory.points)} 个点")
        
        # 设置轨迹头部信息
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # 发布轨迹
        self.trajectory_pub.publish(trajectory)
        
        # 等待执行完成 (简单的时间等待)
        total_time = 0.0
        if trajectory.points:
            last_point = trajectory.points[-1]
            total_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        
        self.get_logger().info(f"等待轨迹执行完成 ({total_time:.2f} 秒)...")
        time.sleep(total_time + 1.0)  # 额外等待1秒
        
        return True
    
    def move_to_pose(self, target_pose):
        """
        移动到目标位姿
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
        Returns:
            bool: 是否成功
        """
        # 获取运动规划
        plan_response = self.get_motion_plan(target_pose)
        
        if plan_response is None:
            return False
        
        # 执行轨迹
        return self.execute_trajectory(plan_response.trajectory.joint_trajectory)
    
    def compute_ik(self, target_pose):
        """
        计算逆运动学
        Args:
            target_pose: geometry_msgs/Pose 目标位姿
        Returns:
            list: 关节角度列表 或 None
        """
        request = GetPositionIK.Request()
        
        # 设置IK请求
        request.ik_request.group_name = self.group_name
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.ik_link_name = self.end_effector_link
        
        # 设置起始状态
        if self.current_joint_state is not None:
            request.ik_request.robot_state.joint_state = self.current_joint_state
        
        # 调用服务
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            return list(response.solution.joint_state.position)
        else:
            self.get_logger().warn(f"IK计算失败，错误代码: {response.error_code.val}")
            return None
    
    def create_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """创建位姿对象"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return (qx, qy, qz, qw)

def main():
    rclpy.init()
    
    controller = MoveItPiperController()
    
    try:
        # 等待系统初始化
        time.sleep(2)
        
        # 示例位姿序列
        poses = [
            {"x": 0.3, "y": 0.0, "z": 0.4, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "正前方"},
            {"x": 0.25, "y": 0.15, "z": 0.35, "roll": 0, "pitch": pi/3, "yaw": pi/6, "name": "右前方"},
            {"x": 0.25, "y": -0.15, "z": 0.35, "roll": 0, "pitch": pi/3, "yaw": -pi/6, "name": "左前方"},
            {"x": 0.2, "y": 0.0, "z": 0.5, "roll": 0, "pitch": 0, "yaw": 0, "name": "正上方"},
        ]
        
        for pose_info in poses:
            controller.get_logger().info(f"=== 移动到 {pose_info['name']} ===")
            
            # 创建目标位姿
            qx, qy, qz, qw = controller.euler_to_quaternion(
                pose_info['roll'], pose_info['pitch'], pose_info['yaw']
            )
            target_pose = controller.create_pose(
                pose_info['x'], pose_info['y'], pose_info['z'],
                qx, qy, qz, qw
            )
            
            # 首先检查IK解
            ik_solution = controller.compute_ik(target_pose)
            if ik_solution:
                controller.get_logger().info(f"IK解: {[f'{angle:.3f}' for angle in ik_solution]}")
                
                # 执行运动
                success = controller.move_to_pose(target_pose)
                if success:
                    controller.get_logger().info(f"✓ 成功到达 {pose_info['name']}")
                else:
                    controller.get_logger().error(f"✗ 移动到 {pose_info['name']} 失败")
            else:
                controller.get_logger().error(f"✗ {pose_info['name']} 无IK解")
            
            # 等待一段时间
            time.sleep(3)
        
        controller.get_logger().info("所有动作完成!")
        
    except KeyboardInterrupt:
        controller.get_logger().info("接收到中断信号")
    except Exception as e:
        controller.get_logger().error(f"发生错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
