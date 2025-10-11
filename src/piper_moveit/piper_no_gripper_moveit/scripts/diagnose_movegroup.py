#!/usr/bin/env python3
"""
诊断MoveGroup Action服务器状态
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
import time

class MoveGroupDiagnostic(Node):
    def __init__(self):
        super().__init__('move_group_diagnostic')
        
        # 创建action客户端
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info("=== MoveGroup诊断工具 ===")
    
    def check_action_server(self):
        """检查action服务器状态"""
        self.get_logger().info("检查MoveGroup action服务器...")
        
        # 等待服务器
        if self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("✓ MoveGroup action服务器可用")
            return True
        else:
            self.get_logger().error("✗ MoveGroup action服务器不可用")
            return False
    
    def run_diagnostic(self):
        """运行完整诊断"""
        print("\n" + "="*50)
        print("  MoveGroup Action 服务器诊断")
        print("="*50)
        
        # 检查action服务器
        server_available = self.check_action_server()
        
        if not server_available:
            print("\n❌ 诊断结果: MoveGroup action服务器不可用")
            print("\n💡 解决建议:")
            print("1. 确保MoveIt已正确启动:")
            print("   ros2 launch piper_no_gripper_moveit demo.launch.py")
            print("2. 检查move_group节点是否运行:")
            print("   ros2 node list | grep move_group")
            print("3. 检查action服务列表:")
            print("   ros2 action list | grep move_group")
            print("4. 查看move_group日志:")
            print("   ros2 node info /move_group")
            return False
        
        print("\n✅ 诊断结果: MoveGroup action服务器正常")
        print("\n🎯 可以尝试运行位姿控制脚本")
        return True

def main():
    rclpy.init()
    
    diagnostic = MoveGroupDiagnostic()
    
    try:
        success = diagnostic.run_diagnostic()
        
        if success:
            print("\n🚀 系统就绪，可以运行位姿控制脚本")
        else:
            print("\n⚠️  请按照建议修复问题后重试")
            
    except KeyboardInterrupt:
        diagnostic.get_logger().info("诊断被中断")
    except Exception as e:
        diagnostic.get_logger().error(f"诊断错误: {e}")
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
