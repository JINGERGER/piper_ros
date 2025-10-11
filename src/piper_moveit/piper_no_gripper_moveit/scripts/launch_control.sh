#!/bin/bash
# Quick launcher for Piper robot control

echo "🤖 Piper Robot Control Launcher"
echo "==============================="

# Check if we're in the right directory
SCRIPTS_DIR="/home/nezha/piper_ros/src/piper_moveit/piper_no_gripper_moveit/scripts"
if [ ! -d "$SCRIPTS_DIR" ]; then
    echo "❌ Error: Scripts directory not found!"
    echo "   Expected: $SCRIPTS_DIR"
    exit 1
fi

cd "$SCRIPTS_DIR"

echo "📂 Current directory: $(pwd)"
echo ""

# Check if ROS2 environment is sourced
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found! Please source your ROS2 environment:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/piper_ros/install/setup.bash"
    exit 1
fi

# Show menu
echo "Available control methods:"
echo "1) Direct Control (Recommended) - piper_direct_control.py"
echo "2) Quick Test - test_direct_control.py"
echo "3) Control Summary - control_summary.py"
echo "4) MoveIt2 Pose Control - piper_pose_example.py"
echo "5) MoveIt2 Joint Control - piper_joint_example.py"
echo "6) Diagnostics - diagnose_movegroup.py"
echo "q) Quit"
echo ""

read -p "Enter your choice (1-6, q): " choice

case $choice in
    1)
        echo "🚀 Starting Direct Control..."
        python3 piper_direct_control.py
        ;;
    2)
        echo "🧪 Starting Quick Test..."
        python3 test_direct_control.py
        ;;
    3)
        echo "📋 Showing Control Summary..."
        python3 control_summary.py
        ;;
    4)
        echo "🎯 Starting MoveIt2 Pose Control..."
        python3 piper_pose_example.py
        ;;
    5)
        echo "🔧 Starting MoveIt2 Joint Control..."
        python3 piper_joint_example.py
        ;;
    6)
        echo "🔍 Running Diagnostics..."
        python3 diagnose_movegroup.py
        ;;
    q|Q)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice!"
        exit 1
        ;;
esac
