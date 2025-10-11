#!/usr/bin/env python3
"""
Piper Robot Control Methods Summary
Lists all available control scripts and their usage
"""

import os

def print_header(title):
    print(f"\n{'='*len(title)}")
    print(title)
    print('='*len(title))

def main():
    scripts_dir = "/home/nezha/piper_ros/src/piper_moveit/piper_no_gripper_moveit/scripts"
    
    print_header("🤖 PIPER ROBOT CONTROL METHODS")
    
    print("\n📋 AVAILABLE CONTROL SCRIPTS:")
    
    print("\n1️⃣  DIRECT TRAJECTORY CONTROL (RECOMMENDED)")
    print("   Script: piper_direct_control.py")
    print("   Description: Bypasses MoveIt planning, sends trajectories directly to controller")
    print("   Usage: python3 piper_direct_control.py")
    print("   Features:")
    print("   - Interactive command interface")
    print("   - Predefined safe poses")
    print("   - Joint space control")
    print("   - Simple Cartesian mapping")
    print("   - Relative joint movements")
    
    print("\n2️⃣  QUICK TEST (FOR DEBUGGING)")
    print("   Script: test_direct_control.py")
    print("   Description: Simple test for direct trajectory control")
    print("   Usage: python3 test_direct_control.py")
    print("   Features:")
    print("   - Small movement test")
    print("   - Home position command")
    print("   - Joint state monitoring")
    
    print("\n3️⃣  MOVEIT2 CONTROL SCRIPTS (MAY HAVE PLANNING ISSUES)")
    
    scripts = [
        ("piper_pose_example.py", "MoveIt2 pose control with IK checking"),
        ("piper_pose_controller.py", "Full-featured MoveIt2 pose controller"),
        ("piper_joint_example.py", "MoveIt2 joint space control"),
        ("piper_micro_control.py", "Micro joint adjustments via MoveIt2"),
        ("piper_joint_control.py", "Alternative MoveIt2 joint controller")
    ]
    
    for script, desc in scripts:
        print(f"   - {script}: {desc}")
    
    print("\n4️⃣  DIAGNOSTIC SCRIPTS")
    print("   - diagnose_movegroup.py: Check MoveIt2 action server status")
    print("   - send_move_goal.sh: Shell script for MoveIt2 action testing")
    
    print_header("🚀 QUICK START GUIDE")
    
    print("\n1. Make sure your Piper robot system is running:")
    print("   ros2 launch piper_no_gripper_moveit piper_no_gripper_moveit_planning_execution.launch.py")
    
    print("\n2. Check controller status:")
    print("   ros2 control list_controllers")
    
    print("\n3. Start with the direct control method (most reliable):")
    print("   cd /home/nezha/piper_ros/src/piper_moveit/piper_no_gripper_moveit/scripts")
    print("   python3 piper_direct_control.py")
    
    print("\n4. Available commands in direct control:")
    print("   - pose home        : Go to home position")
    print("   - pose ready       : Go to ready position") 
    print("   - joints 0 0 0 0 0 0 : Move to specific joint positions")
    print("   - relative 0.1 0 0 0 0 0 : Move joint1 by 0.1 radians")
    print("   - cartesian 0.3 0.1 0.2 : Move to x,y,z position")
    print("   - current          : Show current joint positions")
    
    print_header("⚠️  TROUBLESHOOTING")
    
    print("\n🔧 If MoveIt2 planning fails (error -17):")
    print("   - Use direct control instead: piper_direct_control.py")
    print("   - This bypasses MoveIt2 planning and uses direct trajectories")
    
    print("\n🔧 If joint trajectory controller fails:")
    print("   - Check: ros2 control list_controllers")
    print("   - Ensure 'arm_controller' is active")
    print("   - Restart the robot system if needed")
    
    print("\n🔧 If no joint states received:")
    print("   - Check: ros2 topic echo /joint_states")
    print("   - Ensure joint_state_broadcaster is active")
    
    print_header("📁 FILE LOCATIONS")
    
    print(f"\n📂 Scripts directory: {scripts_dir}")
    print("📂 MoveIt config: /home/nezha/piper_ros/src/piper_moveit/piper_no_gripper_moveit/config/")
    print("📂 Launch files: /home/nezha/piper_ros/src/piper_moveit/piper_no_gripper_moveit/launch/")
    
    print_header("🎯 RECOMMENDED WORKFLOW")
    
    print("\n1. Start robot system (in terminal 1):")
    print("   ros2 launch piper_no_gripper_moveit piper_no_gripper_moveit_planning_execution.launch.py")
    
    print("\n2. Test direct control (in terminal 2):")
    print("   python3 test_direct_control.py")
    
    print("\n3. Use full direct control interface:")
    print("   python3 piper_direct_control.py")
    
    print("\n4. If you need MoveIt2 features, try:")
    print("   python3 piper_pose_example.py")
    
    print("\n💡 The direct control method is currently the most reliable for Piper robot control!")
    print("   It provides joint and pose control without relying on MoveIt2 planning.")

if __name__ == '__main__':
    main()
