# Piper Robot Motion Control Solutions

## Overview
This directory contains multiple control approaches for the Piper robot, addressing MoveIt2 planning failures and providing reliable motion control alternatives.

## Problem Solved
The original MoveIt2 planning was failing with error code -17 (PLANNING_FAILED). We developed a comprehensive set of solutions that includes:
1. **Direct trajectory control** (bypasses MoveIt2 planning)
2. **Improved MoveIt2 scripts** with better error handling
3. **Diagnostic tools** for troubleshooting

## 🎯 Recommended Solution: Direct Trajectory Control

### Main Script: `piper_direct_control.py`
- **Purpose**: Bypasses MoveIt2 planning completely
- **Method**: Sends joint trajectories directly to the controller
- **Reliability**: Most reliable method for Piper robot control
  - 不依赖MoveIt服务
  - 适合简单的位置控制

### 2. `moveit_service_control.py`
- **功能**: 使用MoveIt2服务接口进行运动规划
- **特点**:
  - 调用MoveIt的运动规划服务
  - 支持碰撞检测和路径优化
  - 更安全和智能的运动规划

### 3. `piper_control_launcher.py`
- **功能**: 交互式启动器，提供菜单选择不同控制方式
- **特点**:
  - 用户友好的界面
  - 集成多种控制方式
  - 预设动作序列

### 4. `piper_pose_control_ros2.py`
- **功能**: 完整的MoveIt2 Action客户端实现
- **特点**:
  - 使用MoveGroup Action接口
  - 完整的约束和规划选项

## 使用方法

### 准备工作

1. **启动MoveIt2**:
```bash
# 首先source环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动MoveIt2
ros2 launch piper_no_gripper_moveit demo.launch.py
```

2. **在新终端中运行控制脚本**:
```bash
# source环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 进入脚本目录
cd src/piper_moveit/piper_no_gripper_moveit/scripts/
```

### 方法1: 使用启动器 (推荐)
```bash
python3 piper_control_launcher.py
```
然后按照菜单提示选择控制方式。

### 方法2: 直接运行特定脚本

**简单控制**:
```bash
python3 simple_piper_control.py
```

**MoveIt服务控制**:
```bash
python3 moveit_service_control.py
```

**完整MoveIt Action控制**:
```bash
python3 piper_pose_control_ros2.py
```

## 位姿参数说明

### 位置坐标 (米)
- `x`: 前后方向，正值为前方
- `y`: 左右方向，正值为左侧
- `z`: 上下方向，正值为上方

### 姿态角度 (弧度)
- `roll`: 绕x轴旋转
- `pitch`: 绕y轴旋转  
- `yaw`: 绕z轴旋转

### 常用姿态
- **垂直向下**: `roll=0, pitch=π/2, yaw=0`
- **水平向前**: `roll=0, pitch=0, yaw=0`
- **向右倾斜**: `roll=0, pitch=π/4, yaw=π/6`

## 安全范围建议

### 工作空间限制
- **X轴**: 0.2m 到 0.5m
- **Y轴**: -0.3m 到 0.3m  
- **Z轴**: 0.1m 到 0.6m

### 速度限制
- **最大速度缩放**: 0.1 (10%最大速度)
- **最大加速度缩放**: 0.1 (10%最大加速度)

## 故障排除

### 常见问题

1. **"MoveIt服务不可用"**
   - 确保已启动MoveIt2: `ros2 launch piper_no_gripper_moveit demo.launch.py`
   - 检查话题: `ros2 topic list | grep move_group`

2. **"IK无解"**
   - 检查目标位姿是否在工作空间内
   - 调整位姿参数，特别是Z轴高度
   - 尝试不同的末端执行器姿态

3. **"运动规划失败"**
   - 增加规划时间: `planning_time=10.0`
   - 检查是否有碰撞
   - 调整起始位置

4. **"模块导入错误"**
   - 确保已source ROS2环境
   - 安装缺失的Python包: `pip3 install numpy`

### 调试命令

```bash
# 检查MoveIt服务
ros2 service list | grep moveit

# 检查话题
ros2 topic list | grep joint

# 查看当前关节状态
ros2 topic echo /joint_states

# 查看TF变换
ros2 run tf2_tools view_frames
```

## 扩展开发

### 添加新的控制功能

1. **修改脚本**:
   - 在相应的控制器类中添加新方法
   - 实现具体的控制逻辑

2. **测试新功能**:
   - 先在仿真环境中测试
   - 确保安全后再在实物上测试

3. **集成到启动器**:
   - 在`piper_control_launcher.py`中添加新选项
   - 更新菜单和相应的处理函数

### 自定义预设动作

编辑`piper_control_launcher.py`中的`sequences`字典，添加新的动作序列:

```python
"4": [  # 新的自定义序列
    {"x": 0.3, "y": 0.0, "z": 0.4, "roll": 0, "pitch": pi/2, "yaw": 0, "name": "起始位置"},
    # 添加更多位姿点...
]
```

## 注意事项

⚠️ **安全提醒**:
- 首次使用时建议降低速度 (velocity_scaling_factor < 0.1)
- 确保机器人周围无障碍物
- 准备好急停按钮
- 在仿真环境中充分测试后再在实物上运行

📝 **开发建议**:
- 根据实际的URDF和SRDF文件调整关节名称和链接名称
- 根据实际机器人工作空间调整位置限制
- 添加更多的错误处理和状态检查
