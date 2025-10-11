#!/usr/bin/env python3
"""
快速启动和测试脚本
选择不同的控制方式来控制Piper机器人
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import time
from math import pi

def print_menu():
    """打印菜单"""
    print("\n" + "="*50)
    print("    Piper机器人位姿控制 - 选择控制方式")
    print("="*50)
    print("1. 简单关节控制 (发布关节角度)")
    print("2. 位姿目标发布 (发布目标位姿话题)")
    print("3. MoveIt服务控制 (使用MoveIt规划服务)")
    print("4. 交互式位姿输入")
    print("5. 预设动作序列")
    print("0. 退出")
    print("="*50)

def get_user_pose():
    """获取用户输入的位姿"""
    print("\n请输入目标位姿:")
    try:
        x = float(input("X坐标 (米): "))
        y = float(input("Y坐标 (米): "))
        z = float(input("Z坐标 (米): "))
        
        print("\n选择姿态:")
        print("1. 垂直向下 (常用于抓取)")
        print("2. 水平向前")
        print("3. 自定义欧拉角")
        
        attitude_choice = input("选择姿态 (1-3): ")
        
        if attitude_choice == "1":
            # 垂直向下
            roll, pitch, yaw = 0.0, pi/2, 0.0
        elif attitude_choice == "2":
            # 水平向前
            roll, pitch, yaw = 0.0, 0.0, 0.0
        elif attitude_choice == "3":
            # 自定义
            roll = float(input("Roll角 (弧度): "))
            pitch = float(input("Pitch角 (弧度): "))
            yaw = float(input("Yaw角 (弧度): "))
        else:
            roll, pitch, yaw = 0.0, pi/2, 0.0
        
        return x, y, z, roll, pitch, yaw
    except ValueError:
        print("输入错误，使用默认值")
        return 0.3, 0.0, 0.4, 0.0, pi/2, 0.0

def run_simple_control():
    """运行简单控制"""
    print("\n启动简单关节控制...")
    try:
        from simple_piper_control import SimplePiperController
        
        rclpy.init()
        controller = SimplePiperController()
        
        # 执行一些基本动作
        positions = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "零位置"),
            ([0.0, -pi/4, pi/2, 0.0, pi/4, 0.0], "位置1"),
            ([pi/6, -pi/6, pi/3, pi/6, pi/6, pi/6], "位置2")
        ]
        
        for pos, name in positions:
            print(f"移动到 {name}")
            controller.move_to_joint_positions(pos, duration=3.0)
            time.sleep(1)
        
        print("简单控制演示完成!")
        
    except ImportError:
        print("错误: 找不到 simple_piper_control 模块")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

def run_pose_publisher():
    """运行位姿发布器"""
    print("\n启动位姿发布器...")
    try:
        from simple_piper_control import SimplePiperController
        
        rclpy.init()
        controller = SimplePiperController()
        
        # 获取用户位姿
        x, y, z, roll, pitch, yaw = get_user_pose()
        
        # 创建并发布位姿
        qx, qy, qz, qw = controller.euler_to_quaternion(roll, pitch, yaw)
        pose = controller.create_pose(x, y, z, qx, qy, qz, qw)
        
        print(f"发布位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        print(f"姿态: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
        
        # 持续发布位姿
        for i in range(10):
            controller.publish_target_pose(pose)
            time.sleep(0.5)
        
        print("位姿发布完成!")
        
    except ImportError:
        print("错误: 找不到 simple_piper_control 模块")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

def run_moveit_control():
    """运行MoveIt控制"""
    print("\n启动MoveIt服务控制...")
    try:
        from moveit_service_control import MoveItPiperController
        
        rclpy.init()
        controller = MoveItPiperController()
        
        # 等待初始化
        time.sleep(2)
        
        # 获取用户位姿
        x, y, z, roll, pitch, yaw = get_user_pose()
        
        # 创建目标位姿
        qx, qy, qz, qw = controller.euler_to_quaternion(roll, pitch, yaw)
        target_pose = controller.create_pose(x, y, z, qx, qy, qz, qw)
        
        print(f"计算到位姿的运动规划: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # 首先检查IK
        ik_solution = controller.compute_ik(target_pose)
        if ik_solution:
            print(f"IK解: {[f'{angle:.3f}' for angle in ik_solution]}")
            
            # 执行运动
            success = controller.move_to_pose(target_pose)
            if success:
                print("✓ 成功到达目标位姿!")
            else:
                print("✗ 运动执行失败!")
        else:
            print("✗ 目标位姿无IK解，请调整参数!")
        
    except ImportError:
        print("错误: 找不到 moveit_service_control 模块")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

def run_preset_sequence():
    """运行预设动作序列"""
    print("\n启动预设动作序列...")
    print("选择预设序列:")
    print("1. 基本抓取序列")
    print("2. 巡检序列")
    print("3. 展示序列")
    
    choice = input("选择序列 (1-3): ")
    
    sequences = {
        "1": [  # 基本抓取序列
            {"x": 0.3, "y": 0.0, "z": 0.5, "roll": 0, "pitch": 0, "yaw": 0, "name": "准备位置"},
            {"x": 0.3, "y": 0.0, "z": 0.3, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "接近物体"},
            {"x": 0.3, "y": 0.0, "z": 0.25, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "抓取位置"},
            {"x": 0.3, "y": 0.0, "z": 0.4, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "提升物体"},
            {"x": 0.2, "y": 0.2, "z": 0.4, "roll": 0, "pitch": pi/2, "yaw": pi/4, "name": "移动到目标"},
        ],
        "2": [  # 巡检序列
            {"x": 0.4, "y": 0.0, "z": 0.3, "roll": 0, "pitch": pi/4, "yaw": 0, "name": "检查点1"},
            {"x": 0.3, "y": 0.2, "z": 0.3, "roll": 0, "pitch": pi/4, "yaw": pi/3, "name": "检查点2"},
            {"x": 0.3, "y": -0.2, "z": 0.3, "roll": 0, "pitch": pi/4, "yaw": -pi/3, "name": "检查点3"},
            {"x": 0.2, "y": 0.0, "z": 0.5, "roll": 0, "pitch": 0, "yaw": 0, "name": "观察点"},
        ],
        "3": [  # 展示序列
            {"x": 0.3, "y": 0.0, "z": 0.4, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "中心位置"},
            {"x": 0.25, "y": 0.15, "z": 0.35, "roll": pi/6, "pitch": pi/3, "yaw": pi/4, "name": "右上"},
            {"x": 0.25, "y": -0.15, "z": 0.35, "roll": -pi/6, "pitch": pi/3, "yaw": -pi/4, "name": "左上"},
            {"x": 0.35, "y": 0.0, "z": 0.25, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "前下"},
            {"x": 0.2, "y": 0.0, "z": 0.6, "roll": 0, "pitch": 0, "yaw": 0, "name": "高位"},
        ]
    }
    
    sequence = sequences.get(choice, sequences["1"])
    
    try:
        from moveit_service_control import MoveItPiperController
        
        rclpy.init()
        controller = MoveItPiperController()
        time.sleep(2)
        
        for pose_info in sequence:
            print(f"\n=== 移动到 {pose_info['name']} ===")
            
            qx, qy, qz, qw = controller.euler_to_quaternion(
                pose_info['roll'], pose_info['pitch'], pose_info['yaw']
            )
            target_pose = controller.create_pose(
                pose_info['x'], pose_info['y'], pose_info['z'],
                qx, qy, qz, qw
            )
            
            success = controller.move_to_pose(target_pose)
            if success:
                print(f"✓ 成功到达 {pose_info['name']}")
            else:
                print(f"✗ 到达 {pose_info['name']} 失败")
            
            time.sleep(2)
        
        print("\n预设序列执行完成!")
        
    except ImportError:
        print("错误: 找不到 moveit_service_control 模块")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        try:
            controller.destroy_node()
            rclpy.shutdown()
        except:
            pass

def main():
    """主函数"""
    while True:
        print_menu()
        choice = input("请选择 (0-5): ")
        
        if choice == "0":
            print("退出程序")
            break
        elif choice == "1":
            run_simple_control()
        elif choice == "2":
            run_pose_publisher()
        elif choice == "3":
            run_moveit_control()
        elif choice == "4":
            run_moveit_control()  # 交互式也使用MoveIt控制
        elif choice == "5":
            run_preset_sequence()
        else:
            print("无效选择，请重新输入")
        
        input("\n按Enter继续...")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"程序错误: {e}")
