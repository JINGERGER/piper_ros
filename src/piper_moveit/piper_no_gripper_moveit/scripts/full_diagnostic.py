#!/usr/bin/env python3
"""
全面诊断Piper机器人和MoveIt状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time

class PiperDiagnostic(Node):
    def __init__(self):
        super().__init__('piper_diagnostic')
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        self.get_logger().info("=== Piper机器人诊断工具 ===")
    
    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.current_joint_state = msg
    
    def run_diagnostic(self):
        """运行完整诊断"""
        print("\n" + "="*60)
        print("  Piper机器人系统诊断")
        print("="*60)
        
        # 等待关节状态
        self.get_logger().info("等待关节状态数据...")
        timeout = 10.0
        start_time = time.time()
        
        while self.current_joint_state is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            print("❌ 无法获取关节状态数据")
            print("💡 检查:")
            print("   - ros2 topic echo /joint_states")
            print("   - 确保robot_state_publisher在运行")
            return False
        
        # 分析关节状态
        self.analyze_joint_state()
        
        # 检查控制器状态
        self.check_controller_status()
        
        # 检查MoveIt配置
        self.check_moveit_config()
        
        return True
    
    def analyze_joint_state(self):
        """分析关节状态"""
        print("\n📊 关节状态分析:")
        print("-" * 40)
        
        js = self.current_joint_state
        print(f"时间戳: {js.header.stamp.sec}.{js.header.stamp.nanosec:09d}")
        print(f"关节数量: {len(js.name)}")
        
        if len(js.name) != len(js.position):
            print("❌ 关节名称和位置数量不匹配!")
            return
        
        print("\n关节状态详情:")
        for i, (name, pos) in enumerate(zip(js.name, js.position)):
            vel = js.velocity[i] if i < len(js.velocity) else 0.0
            effort = js.effort[i] if i < len(js.effort) else 0.0
            
            print(f"  {name:12}: pos={pos:8.4f}  vel={vel:8.4f}  effort={effort:8.4f}")
        
        # 检查是否有异常值
        print("\n🔍 异常检查:")
        issues = []
        
        for i, (name, pos) in enumerate(zip(js.name, js.position)):
            if abs(pos) > 6.28:  # 超过2π
                issues.append(f"关节 {name} 位置异常: {pos:.4f}")
            
            if i < len(js.velocity):
                vel = js.velocity[i]
                if abs(vel) > 10.0:  # 速度过高
                    issues.append(f"关节 {name} 速度异常: {vel:.4f}")
        
        if issues:
            print("❌ 发现异常:")
            for issue in issues:
                print(f"   - {issue}")
        else:
            print("✅ 关节状态正常")
    
    def check_controller_status(self):
        """检查控制器状态"""
        print("\n🎮 控制器状态:")
        print("-" * 40)
        
        # 这里可以添加更多控制器检查
        # 目前只是提示用户手动检查
        print("💡 手动检查控制器:")
        print("   ros2 control list_controllers")
        print("   ros2 control list_hardware_interfaces")
    
    def check_moveit_config(self):
        """检查MoveIt配置"""
        print("\n⚙️  MoveIt配置检查:")
        print("-" * 40)
        
        print("💡 手动检查MoveIt:")
        print("   ros2 param list /move_group | head -20")
        print("   ros2 service list | grep move_group")
        print("   ros2 action list | grep move")
    
    def suggest_solutions(self):
        """建议解决方案"""
        print("\n💡 故障排除建议:")
        print("="*60)
        print("1. 重启系统:")
        print("   - 停止当前launch: Ctrl+C")
        print("   - 重新启动: ros2 launch piper_no_gripper_moveit demo.launch.py")
        print("")
        print("2. 检查关节限制:")
        print("   - 查看 config/joint_limits.yaml")
        print("   - 确保当前关节位置在限制范围内")
        print("")
        print("3. 使用更保守的目标:")
        print("   - 减小关节变化量")
        print("   - 使用当前位置附近的小增量")
        print("")
        print("4. 检查碰撞检测:")
        print("   - 机器人可能检测到自碰撞")
        print("   - 尝试调整起始姿态")

def main():
    rclpy.init()
    
    diagnostic = PiperDiagnostic()
    
    try:
        success = diagnostic.run_diagnostic()
        
        if success:
            print("\n✅ 诊断完成")
        else:
            print("\n❌ 诊断失败")
        
        diagnostic.suggest_solutions()
        
    except KeyboardInterrupt:
        diagnostic.get_logger().info("诊断被中断")
    except Exception as e:
        diagnostic.get_logger().error(f"诊断错误: {e}")
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
