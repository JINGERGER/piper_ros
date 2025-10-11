#!/usr/bin/env python3
"""
基于样例的Piper机器人位姿控制脚本
使用MoveIt Python API控制机器人末端执行器移动到指定位姿
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import sys
import time
from math import pi, cos, sin

# 尝试导入MoveIt Python接口
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_PY_AVAILABLE = True
except ImportError:
    MOVEIT_PY_AVAILABLE = False
    print("警告: MoveItPy不可用，将使用替代方案")

class PiperPoseController(Node):
    def __init__(self):
        super().__init__('piper_pose_controller')
        
        if MOVEIT_PY_AVAILABLE:
            # 使用MoveItPy (推荐方式)
            self.setup_moveit_py()
        else:
            # 使用Action客户端作为替代方案
            self.setup_action_client()
    
    def setup_moveit_py(self):
        """使用MoveItPy设置"""
        try:
            # 初始化MoveItPy
            self.moveit = MoveItPy(node_name="piper_moveit_py")
            
            # 获取规划组
            self.piper_arm = self.moveit.get_planning_component("arm")
            
            # 获取机器人模型
            self.robot_model = self.moveit.get_robot_model()
            
            self.get_logger().info("✓ MoveItPy初始化成功")
            
        except Exception as e:
            self.get_logger().error(f"MoveItPy初始化失败: {e}")
            self.setup_action_client()
    
    def setup_action_client(self):
        """设置Action客户端作为替代方案"""
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            MotionPlanRequest, Constraints, 
            PositionConstraint, OrientationConstraint,
            PlanningOptions, RobotState
        )
        
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info("等待MoveGroup action服务器...")
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("✓ MoveGroup action服务器已连接")
        else:
            self.get_logger().error("✗ MoveGroup action服务器不可用")
    
    def set_pose_goal_moveit_py(self, pose_goal):
        """使用MoveItPy设置位姿目标"""
        try:
            # 设置起始状态为当前状态
            self.piper_arm.set_start_state_to_current_state()
            
            # 设置位姿目标
            self.piper_arm.set_goal_state(
                pose_stamped_msg=pose_goal, 
                pose_link="link6"  # Piper的末端执行器链接
            )
            
            # 规划运动
            plan_result = self.piper_arm.plan()
            
            if plan_result:
                self.get_logger().info("✓ 运动规划成功")
                
                # 执行运动
                robot_trajectory = plan_result.trajectory
                success = self.moveit.execute(
                    robot_trajectory, 
                    controllers=["arm_controller"]  # 根据您的控制器名称调整
                )
                
                if success:
                    self.get_logger().info("✓ 运动执行成功!")
                    return True
                else:
                    self.get_logger().error("✗ 运动执行失败")
                    return False
            else:
                self.get_logger().error("✗ 运动规划失败")
                return False
                
        except Exception as e:
            self.get_logger().error(f"位姿控制错误: {e}")
            return False
    
    def set_pose_goal_action(self, pose_goal):
        """使用Action客户端设置位姿目标"""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            MotionPlanRequest, Constraints,
            PositionConstraint, OrientationConstraint,
            PlanningOptions, RobotState
        )
        
        try:
            # 创建目标消息
            goal_msg = MoveGroup.Goal()
            
            # 设置规划请求
            goal_msg.request = MotionPlanRequest()
            goal_msg.request.group_name = 'arm'
            goal_msg.request.pipeline_id = 'ompl'
            goal_msg.request.planner_id = 'RRTConnect'
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 3
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            
            # 设置起始状态
            goal_msg.request.start_state = RobotState()
            goal_msg.request.start_state.is_diff = True
            
            # 创建位置约束
            constraints = Constraints()
            constraints.name = "pose_goal"
            
            # 位置约束
            pos_constraint = PositionConstraint()
            pos_constraint.header = pose_goal.header
            pos_constraint.link_name = "link6"
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # 设置位置目标
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Point
            
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.001]  # 1mm容差
            
            pos_constraint.constraint_region.primitives = [sphere]
            pos_constraint.constraint_region.primitive_poses = [pose_goal.pose]
            pos_constraint.weight = 1.0
            
            # 方向约束
            orient_constraint = OrientationConstraint()
            orient_constraint.header = pose_goal.header
            orient_constraint.link_name = "link6"
            orient_constraint.orientation = pose_goal.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.01
            orient_constraint.absolute_y_axis_tolerance = 0.01
            orient_constraint.absolute_z_axis_tolerance = 0.01
            orient_constraint.weight = 1.0
            
            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(orient_constraint)
            goal_msg.request.goal_constraints.append(constraints)
            
            # 设置规划选项
            goal_msg.planning_options = PlanningOptions()
            goal_msg.planning_options.plan_only = False  # 执行运动
            
            # 发送目标
            self.get_logger().info("发送位姿目标...")
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            # 等待结果
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("✗ 位姿目标被拒绝")
                return False
            
            self.get_logger().info("✓ 位姿目标已接受，等待执行...")
            
            # 等待执行结果
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=30.0)
            
            result = get_result_future.result().result
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info("✓ 位姿目标执行成功!")
                return True
            else:
                self.get_logger().error(f"✗ 位姿目标执行失败，错误代码: {result.error_code.val}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Action控制错误: {e}")
            return False
    
    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        移动到指定位姿
        Args:
            x, y, z: 位置坐标 (米)
            qx, qy, qz, qw: 四元数方向
        """
        # 创建位姿目标
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"  # Piper的基坐标系
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw
        
        self.get_logger().info(f"目标位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # 选择控制方法
        if MOVEIT_PY_AVAILABLE and hasattr(self, 'piper_arm'):
            return self.set_pose_goal_moveit_py(pose_goal)
        else:
            return self.set_pose_goal_action(pose_goal)
    
    def move_to_pose_euler(self, x, y, z, roll, pitch, yaw):
        """
        使用欧拉角移动到指定位姿
        Args:
            x, y, z: 位置坐标 (米)
            roll, pitch, yaw: 欧拉角 (弧度)
        """
        # 欧拉角转四元数
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        
        return self.move_to_pose(x, y, z, qx, qy, qz, qw)

def plan_and_execute(controller, pose_info):
    """规划和执行运动 - 类似样例中的plan_and_execute函数"""
    success = controller.move_to_pose_euler(
        pose_info['x'], pose_info['y'], pose_info['z'],
        pose_info['roll'], pose_info['pitch'], pose_info['yaw']
    )
    
    if success:
        controller.get_logger().info(f"✓ 成功到达位姿: {pose_info['name']}")
    else:
        controller.get_logger().error(f"✗ 移动到位姿失败: {pose_info['name']}")
    
    return success

def main():
    rclpy.init()
    
    # 创建控制器
    controller = PiperPoseController()
    
    try:
        # 等待系统初始化
        time.sleep(2)
        
        # 示例位姿序列 (类似样例中的使用方式)
        poses = [
            {
                'name': '正前方抓取位置',
                'x': 0.28, 'y': -0.2, 'z': 0.5,
                'roll': 0.0, 'pitch': pi/2, 'yaw': 0.0
            },
            {
                'name': '右侧观察位置', 
                'x': 0.25, 'y': 0.15, 'z': 0.4,
                'roll': 0.0, 'pitch': pi/3, 'yaw': pi/4
            },
            {
                'name': '左侧放置位置',
                'x': 0.25, 'y': -0.15, 'z': 0.35,
                'roll': 0.0, 'pitch': pi/2, 'yaw': -pi/6
            },
            {
                'name': '高位检查位置',
                'x': 0.2, 'y': 0.0, 'z': 0.6,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            }
        ]
        
        # 执行位姿序列
        for pose_info in poses:
            controller.get_logger().info(f"\n=== 移动到 {pose_info['name']} ===")
            
            # 规划和执行 (类似样例)
            success = plan_and_execute(controller, pose_info)
            
            if not success:
                controller.get_logger().warn("跳过后续位姿...")
                break
            
            # 等待一段时间
            time.sleep(3)
        
        controller.get_logger().info("\n=== 所有位姿执行完成! ===")
        
        # 交互式控制
        while True:
            print("\n" + "="*50)
            print("交互式位姿控制")
            print("="*50)
            print("输入目标位姿 (输入 'q' 退出):")
            
            try:
                x = input("X坐标 (米, 默认0.3): ")
                if x.lower() == 'q':
                    break
                x = float(x) if x else 0.3
                
                y = input("Y坐标 (米, 默认0.0): ")
                y = float(y) if y else 0.0
                
                z = input("Z坐标 (米, 默认0.4): ")
                z = float(z) if z else 0.4
                
                print("选择姿态: 1)垂直向下 2)水平向前 3)自定义")
                attitude = input("选择 (默认1): ")
                
                if attitude == "2":
                    roll, pitch, yaw = 0.0, 0.0, 0.0
                elif attitude == "3":
                    roll = float(input("Roll (弧度, 默认0): ") or 0)
                    pitch = float(input("Pitch (弧度, 默认π/2): ") or pi/2)
                    yaw = float(input("Yaw (弧度, 默认0): ") or 0)
                else:
                    roll, pitch, yaw = 0.0, pi/2, 0.0
                
                # 执行运动
                pose_info = {
                    'name': '自定义位姿',
                    'x': x, 'y': y, 'z': z,
                    'roll': roll, 'pitch': pitch, 'yaw': yaw
                }
                
                plan_and_execute(controller, pose_info)
                
            except (ValueError, KeyboardInterrupt):
                break
        
    except KeyboardInterrupt:
        controller.get_logger().info("接收到中断信号")
    except Exception as e:
        controller.get_logger().error(f"程序错误: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
