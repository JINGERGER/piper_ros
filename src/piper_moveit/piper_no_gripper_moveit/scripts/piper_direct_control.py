#!/usr/bin/env python3
"""
Piper Direct Trajectory Control - Bypasses MoveIt Planning
This script directly sends joint trajectory commands to the controller,
providing both joint-space and pose-mapped control without MoveIt planning.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from math import pi, sin, cos, atan2, sqrt
import time
import threading

class PiperDirectController(Node):
    def __init__(self):
        super().__init__('piper_direct_controller')
        
        # Joint names (in order)
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        # Action client for direct trajectory control
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current joint positions
        self.current_joints = [0.0] * 6
        self.joints_received = False
        
        # Joint limits (radians) - conservative limits
        self.joint_limits = {
            'joint1': (-2.96, 2.96),  # ±170°
            'joint2': (-2.18, 2.18),  # ±125°
            'joint3': (-2.96, 2.96),  # ±170°
            'joint4': (-2.79, 2.79),  # ±160°
            'joint5': (-2.96, 2.96),  # ±170°
            'joint6': (-3.14, 3.14)   # ±180°
        }
        
        # Predefined safe poses (as joint configurations)
        self.safe_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
            'up': [0.0, -1.0, 1.5, 0.0, 0.5, 0.0],
            'forward': [0.0, 0.0, -0.5, 0.0, 0.5, 0.0],
            'left': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
            'right': [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        
        self.get_logger().info("Piper Direct Controller initialized")
        self.get_logger().info("Waiting for joint states...")
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available!")
        else:
            self.get_logger().info("Connected to trajectory action server")
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        try:
            # Map joint states to our joint order
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_joints[i] = msg.position[idx]
            
            if not self.joints_received:
                self.joints_received = True
                self.get_logger().info(f"Current joints: {[f'{j:.3f}' for j in self.current_joints]}")
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
    
    def validate_joint_limits(self, joint_positions):
        """Check if joint positions are within limits"""
        for i, (joint_name, position) in enumerate(zip(self.joint_names, joint_positions)):
            min_limit, max_limit = self.joint_limits[joint_name]
            if position < min_limit or position > max_limit:
                self.get_logger().warn(
                    f"Joint {joint_name} position {position:.3f} outside limits "
                    f"[{min_limit:.3f}, {max_limit:.3f}]"
                )
                # Clamp to limits
                joint_positions[i] = max(min_limit, min(max_limit, position))
        return joint_positions
    
    def move_to_joints(self, target_joints, duration=3.0):
        """Move to target joint positions directly"""
        if not self.joints_received:
            self.get_logger().error("No joint states received yet!")
            return False
        
        # Validate and clamp joint positions
        target_joints = self.validate_joint_limits(list(target_joints))
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        
        # Create and send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Sending joint trajectory: {[f'{j:.3f}' for j in target_joints]}")
        
        # Send goal and wait for result
        future = self.trajectory_client.send_goal_async(goal)
        
        def done_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected!")
                return
            
            self.get_logger().info("Trajectory goal accepted, executing...")
            
            # Get result
            result_future = goal_handle.get_result_async()
            
            def result_callback(future):
                result = future.result().result
                if result.error_code == 0:
                    self.get_logger().info("Trajectory executed successfully!")
                else:
                    self.get_logger().error(f"Trajectory execution failed: {result.error_code}")
            
            result_future.add_done_callback(result_callback)
        
        future.add_done_callback(done_callback)
        return True
    
    def move_to_pose_name(self, pose_name, duration=3.0):
        """Move to a predefined pose"""
        if pose_name not in self.safe_poses:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            self.get_logger().info(f"Available poses: {list(self.safe_poses.keys())}")
            return False
        
        target_joints = self.safe_poses[pose_name]
        self.get_logger().info(f"Moving to pose '{pose_name}'")
        return self.move_to_joints(target_joints, duration)
    
    def move_relative_joints(self, joint_deltas, duration=3.0):
        """Move joints by relative amounts"""
        if not self.joints_received:
            self.get_logger().error("No joint states received yet!")
            return False
        
        if len(joint_deltas) != 6:
            self.get_logger().error("joint_deltas must have 6 values")
            return False
        
        target_joints = [current + delta for current, delta in zip(self.current_joints, joint_deltas)]
        self.get_logger().info(f"Moving relative: {[f'{d:+.3f}' for d in joint_deltas]}")
        return self.move_to_joints(target_joints, duration)
    
    def simple_ik_mapping(self, pose):
        """
        Simple pose-to-joint mapping for basic positions
        This is a simplified IK solution for demonstration
        """
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        # Basic joint configuration based on position
        joints = [0.0] * 6
        
        # Joint 1: Base rotation based on Y position
        joints[0] = atan2(y, x) if x != 0 else 0.0
        
        # Joint 2 & 3: Arm positioning based on height and reach
        reach = sqrt(x*x + y*y)
        if reach > 0.5:  # Too far
            reach = 0.5
        
        # Simple 2-DOF arm positioning
        joints[1] = -reach * 0.5 + z * 0.3
        joints[2] = reach * 1.0 - z * 0.5
        
        # Joints 4-6: Simple wrist orientation
        joints[3] = 0.0  # Keep wrist straight
        joints[4] = -joints[1] - joints[2]  # Level end effector
        joints[5] = 0.0  # No rotation
        
        return joints
    
    def move_to_cartesian_pose(self, pose, duration=3.0):
        """Move to Cartesian pose using simple IK mapping"""
        self.get_logger().info(f"Mapping pose to joints: "
                              f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        target_joints = self.simple_ik_mapping(pose)
        return self.move_to_joints(target_joints, duration)
    
    def get_current_joints(self):
        """Return current joint positions"""
        return self.current_joints.copy() if self.joints_received else None

def main():
    rclpy.init()
    controller = PiperDirectController()
    
    print("\n=== Piper Direct Controller ===")
    print("This controller bypasses MoveIt and sends trajectories directly to the robot.")
    print("\nAvailable commands:")
    print("1. pose <name>     - Move to predefined pose")
    print("2. joints j1 j2 j3 j4 j5 j6 - Move to joint positions (radians)")
    print("3. relative d1 d2 d3 d4 d5 d6 - Move joints by deltas (radians)")
    print("4. cartesian x y z - Move to Cartesian position (simple IK)")
    print("5. current         - Show current joint positions")
    print("6. quit           - Exit")
    print(f"\nAvailable poses: {list(controller.safe_poses.keys())}")
    print("\nWaiting for joint states...")
    
    # Wait for joint states
    while not controller.joints_received and rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    print("Ready for commands!")
    
    try:
        while rclpy.ok():
            try:
                cmd = input("\nEnter command: ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'current':
                    joints = controller.get_current_joints()
                    if joints:
                        print(f"Current joints: {[f'{j:.3f}' for j in joints]}")
                    else:
                        print("No joint data available")
                
                elif cmd[0] == 'pose' and len(cmd) == 2:
                    controller.move_to_pose_name(cmd[1])
                
                elif cmd[0] == 'joints' and len(cmd) == 7:
                    joints = [float(x) for x in cmd[1:]]
                    controller.move_to_joints(joints)
                
                elif cmd[0] == 'relative' and len(cmd) == 7:
                    deltas = [float(x) for x in cmd[1:]]
                    controller.move_relative_joints(deltas)
                
                elif cmd[0] == 'cartesian' and len(cmd) == 4:
                    pose = Pose()
                    pose.position = Point(x=float(cmd[1]), y=float(cmd[2]), z=float(cmd[3]))
                    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    controller.move_to_cartesian_pose(pose)
                
                else:
                    print("Invalid command format")
                
                # Spin briefly to process callbacks
                for _ in range(10):
                    rclpy.spin_once(controller, timeout_sec=0.1)
                    
            except ValueError:
                print("Invalid number format")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
