#!/usr/bin/env python3
"""
Simple Piper Direct Control Test
Quick test for direct trajectory control functionality
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time

class QuickPiperTest(Node):
    def __init__(self):
        super().__init__('quick_piper_test')
        
        # Joint names
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        # Action client
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
        
        self.current_joints = [0.0] * 6
        self.joints_received = False
        
        self.get_logger().info("Quick Piper Test initialized")
        
        # Wait for services
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available!")
        else:
            self.get_logger().info("✓ Trajectory action server connected")
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        try:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_joints[i] = msg.position[idx]
            
            if not self.joints_received:
                self.joints_received = True
                self.get_logger().info(f"✓ Joint states received: {[f'{j:.3f}' for j in self.current_joints]}")
        except Exception as e:
            self.get_logger().error(f"Error in joint state callback: {e}")
    
    def send_small_movement(self):
        """Send a very small joint movement to test"""
        if not self.joints_received:
            self.get_logger().error("No joint states received yet!")
            return False
        
        # Very small movement - just move joint1 by 0.1 radians
        target_joints = self.current_joints.copy()
        target_joints[0] += 0.1  # Small rotation of base joint
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create point
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        
        trajectory.points = [point]
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Sending small movement: joint1 {self.current_joints[0]:.3f} → {target_joints[0]:.3f}")
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        return future
    
    def send_home_position(self):
        """Send robot to home position"""
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create point
        point = JointTrajectoryPoint()
        point.positions = home_joints
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        
        trajectory.points = [point]
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info("Sending home position command")
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        return future

def main():
    rclpy.init()
    node = QuickPiperTest()
    
    print("\n=== Quick Piper Direct Control Test ===")
    print("Testing direct trajectory control (bypassing MoveIt)")
    
    # Wait for joint states
    print("Waiting for joint states...")
    timeout = 10.0
    start_time = time.time()
    while not node.joints_received and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.joints_received:
        print("❌ Failed to receive joint states!")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    print("✅ Ready for testing!")
    
    try:
        while rclpy.ok():
            print("\nCommands:")
            print("1 - Send small movement (joint1 +0.1 rad)")
            print("2 - Send home position")
            print("q - Quit")
            
            choice = input("Enter choice: ").strip()
            
            if choice == 'q':
                break
            elif choice == '1':
                future = node.send_small_movement()
                if future:
                    print("Command sent, waiting for result...")
                    # Wait a bit for execution
                    for _ in range(50):  # 5 seconds
                        rclpy.spin_once(node, timeout_sec=0.1)
            elif choice == '2':
                future = node.send_home_position()
                if future:
                    print("Home command sent, waiting for result...")
                    for _ in range(50):  # 5 seconds
                        rclpy.spin_once(node, timeout_sec=0.1)
            else:
                print("Invalid choice")
            
            # Always spin to update joint states
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
