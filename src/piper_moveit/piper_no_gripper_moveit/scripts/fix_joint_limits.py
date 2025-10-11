#!/usr/bin/env python3
"""
修复Piper机器人关节限制违规问题
当机器人处于违反关节限制的状态时，MoveIt无法规划运动
这个脚本将机器人移动到安全状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
from math import pi

class PiperJointLimitFixer(Node):
    def __init__(self):
        super().__init__('piper_joint_limit_fixer')
        
        # 发布轨迹命令
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # 订阅关节状态
        self.current_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 安全的关节限制 (根据实际硬件调整)
        self.joint_limits = {
            'joint1': (-pi, pi),
            'joint2': (-pi/2, pi/2), 
            'joint3': (-pi, pi),
            'joint4': (-pi, pi),
            'joint5': (-pi/2, pi/2),
            'joint6': (-pi, pi)
        }
        
        self.get_logger().info("关节限制修复器已启动")
    
    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.current_joint_state = msg
    
    def check_joint_limits(self):
        """检查当前关节是否违反限制"""
        if not self.current_joint_state:
            return False, "未收到关节状态"
        
        violations = []
        
        for i, joint_name in enumerate(self.joint_names):
            if i < len(self.current_joint_state.position):
                current_pos = self.current_joint_state.position[i]
                min_limit, max_limit = self.joint_limits[joint_name]
                
                if current_pos < min_limit:
                    violations.append(f"{joint_name}: {current_pos:.4f} < {min_limit:.4f}")
                elif current_pos > max_limit:
                    violations.append(f"{joint_name}: {current_pos:.4f} > {max_limit:.4f}")
        
        if violations:
            return True, violations
        else:
            return False, "所有关节在安全范围内"
    
    def fix_joint_positions(self):
        """修复违反限制的关节位置"""
        if not self.current_joint_state:
            self.get_logger().error("无关节状态数据")
            return False
        
        # 计算安全的关节位置
        safe_positions = []
        
        for i, joint_name in enumerate(self.joint_names):
            if i < len(self.current_joint_state.position):
                current_pos = self.current_joint_state.position[i]
                min_limit, max_limit = self.joint_limits[joint_name]
                
                # 将违规位置夹紧到安全范围
                if current_pos < min_limit:
                    safe_pos = min_limit + 0.01  # 稍微远离限制
                    self.get_logger().warn(f"修复 {joint_name}: {current_pos:.4f} -> {safe_pos:.4f}")
                elif current_pos > max_limit:
                    safe_pos = max_limit - 0.01  # 稍微远离限制
                    self.get_logger().warn(f"修复 {joint_name}: {current_pos:.4f} -> {safe_pos:.4f}")
                else:
                    safe_pos = current_pos  # 保持当前位置
                
                safe_positions.append(safe_pos)
            else:
                safe_positions.append(0.0)  # 默认零位置
        
        # 发送安全轨迹
        return self.send_trajectory(safe_positions, duration=3.0)
    
    def move_to_home_position(self):
        """移动到安全的零位置"""
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("移动到安全的零位置...")
        return self.send_trajectory(home_positions, duration=5.0)
    
    def send_trajectory(self, joint_positions, duration=3.0):
        """发送关节轨迹"""
        try:
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names
            
            # 创建轨迹点
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0] * len(joint_positions)
            point.accelerations = [0.0] * len(joint_positions)
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            
            trajectory.points = [point]
            
            # 发布轨迹
            self.trajectory_pub.publish(trajectory)
            
            self.get_logger().info(f"发送轨迹: {[f'{pos:.3f}' for pos in joint_positions]}")
            
            # 等待执行完成
            time.sleep(duration + 1.0)
            return True
            
        except Exception as e:
            self.get_logger().error(f"发送轨迹失败: {e}")
            return False

def main():
    rclpy.init()
    
    fixer = PiperJointLimitFixer()
    
    try:
        # 等待获取关节状态
        fixer.get_logger().info("等待关节状态...")
        for i in range(50):  # 等待5秒
            rclpy.spin_once(fixer, timeout_sec=0.1)
            if fixer.current_joint_state:
                break
        
        if not fixer.current_joint_state:
            fixer.get_logger().error("无法获取关节状态")
            return
        
        fixer.get_logger().info("✓ 已获取关节状态")
        
        # 显示当前关节位置
        fixer.get_logger().info("当前关节位置:")
        for i, joint_name in enumerate(fixer.joint_names):
            if i < len(fixer.current_joint_state.position):
                pos = fixer.current_joint_state.position[i]
                fixer.get_logger().info(f"  {joint_name}: {pos:.4f}")
        
        # 检查关节限制
        has_violations, result = fixer.check_joint_limits()
        
        if has_violations:
            fixer.get_logger().error("⚠️  发现关节限制违规:")
            for violation in result:
                fixer.get_logger().error(f"  {violation}")
            
            fixer.get_logger().info("🔧 开始修复关节位置...")
            
            if fixer.fix_joint_positions():
                fixer.get_logger().info("✅ 关节位置已修复!")
                
                # 再次检查
                rclpy.spin_once(fixer, timeout_sec=0.1)
                has_violations_after, result_after = fixer.check_joint_limits()
                
                if not has_violations_after:
                    fixer.get_logger().info("✅ 所有关节现在都在安全范围内")
                    fixer.get_logger().info("🚀 现在可以使用MoveIt规划了!")
                else:
                    fixer.get_logger().warn("⚠️  仍有关节限制问题，尝试移动到零位置...")
                    fixer.move_to_home_position()
            else:
                fixer.get_logger().error("❌ 修复失败")
        else:
            fixer.get_logger().info("✅ 所有关节都在安全范围内")
            fixer.get_logger().info(f"状态: {result}")
            
            # 可选：移动到标准零位置
            user_input = input("\n是否移动到零位置? (y/N): ")
            if user_input.lower() == 'y':
                fixer.move_to_home_position()
                fixer.get_logger().info("✅ 已移动到零位置")
        
        fixer.get_logger().info("\n" + "="*50)
        fixer.get_logger().info("修复完成! 现在可以尝试MoveIt控制:")
        fixer.get_logger().info("  python3 piper_pose_example.py")
        fixer.get_logger().info("  python3 piper_joint_example.py")
        fixer.get_logger().info("="*50)
        
    except KeyboardInterrupt:
        fixer.get_logger().info("修复被中断")
    except Exception as e:
        fixer.get_logger().error(f"修复错误: {e}")
    finally:
        fixer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
