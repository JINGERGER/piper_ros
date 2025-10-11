# Piper机器人位姿控制 - 基于样例风格

基于您提供的样例风格，我创建了三种控制Piper机器人的方法：

## 🎯 控制方式对比

### 1. **命令行方式** (`send_move_goal.sh`)
```bash
# 关节控制 (默认)
./send_move_goal.sh

# 位姿控制
./send_move_goal.sh pose

# 执行运动 (而不只是规划)
./send_move_goal.sh pose false
```

### 2. **样例风格Python脚本** (`piper_pose_example.py`) ⭐ 推荐
```bash
python3 piper_pose_example.py
```

这个脚本完全按照您提供的样例风格编写：

```python
# 1. 设置起始状态为当前状态
piper_arm.set_start_state_to_current_state()

# 2. 设置位姿目标 (PoseStamped消息)
pose_goal = PoseStamped()
pose_goal.header.frame_id = "base_link"  # 类似"panda_link0"
pose_goal.pose.orientation.w = 1.0       # 或自定义四元数
pose_goal.pose.position.x = 0.28
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.5

# 3. 设置目标状态
piper_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")

# 4. 规划并执行
plan_and_execute(piper, piper_arm, logger)
```

### 3. **完整控制器** (`piper_pose_controller.py`)
```bash
python3 piper_pose_controller.py
```

支持交互式控制和多种控制模式。

## 🚀 快速开始

### 第一步：启动MoveIt
```bash
# 终端1：启动MoveIt系统
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

### 第二步：运行位姿控制
```bash
# 终端2：运行样例风格控制 (推荐)
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/piper_moveit/piper_no_gripper_moveit/scripts/
python3 piper_pose_example.py
```

## 📋 样例中的关键映射

| 样例 (Panda) | Piper等效 | 说明 |
|-------------|-----------|------|
| `panda_link0` | `base_link` | 基坐标系 |
| `panda_link8` | `link6` | 末端执行器 |
| `panda_arm` | `arm` | 规划组名 |
| `panda` | `piper` | 机器人名称 |

## 🎮 位姿设置示例

### 基本位姿
```python
# 正前方抓取位置 - 类似样例
pose_goal.pose.position.x = 0.28
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.5
pose_goal.pose.orientation.w = 1.0  # 或自定义
```

### 使用欧拉角设置方向
```python
from math import pi, sin, cos

def euler_to_quaternion(roll, pitch, yaw):
    # 转换函数已在脚本中提供
    pass

# 垂直向下 (常用于抓取)
qx, qy, qz, qw = euler_to_quaternion(0, pi/2, 0)

# 水平向前
qx, qy, qz, qw = euler_to_quaternion(0, 0, 0)

# 设置到位姿
pose_goal.pose.orientation.x = qx
pose_goal.pose.orientation.y = qy
pose_goal.pose.orientation.z = qz
pose_goal.pose.orientation.w = qw
```

## 🔧 脚本功能详解

### `piper_pose_example.py` - 样例风格实现
- ✅ 完全按照您的样例API设计
- ✅ `set_start_state_to_current_state()`
- ✅ `set_goal_state(pose_stamped_msg, pose_link)`
- ✅ `plan_and_execute(piper, piper_arm, logger)`
- ✅ 多个预设位姿示例
- ✅ 错误处理和日志

### `send_move_goal.sh` - 命令行版本
- ✅ 支持关节和位姿控制
- ✅ 可配置执行模式
- ✅ 样例中的位姿参数

### `piper_pose_controller.py` - 高级控制器
- ✅ MoveItPy接口 (如果可用)
- ✅ Action客户端备选方案
- ✅ 交互式控制
- ✅ 预设动作序列

## ⚠️ 重要提醒

### 安全设置
```python
# 在所有脚本中都设置了安全的速度限制
max_velocity_scaling_factor: 0.1    # 10%最大速度
max_acceleration_scaling_factor: 0.1 # 10%最大加速度
```

### 工作空间建议
```python
# 推荐的安全工作范围
x: 0.2 ~ 0.5m   # 前后方向
y: -0.3 ~ 0.3m  # 左右方向  
z: 0.1 ~ 0.6m   # 上下方向
```

### 常用姿态
```python
# 垂直向下 (抓取)
roll=0, pitch=π/2, yaw=0

# 水平向前 (观察)  
roll=0, pitch=0, yaw=0

# 倾斜角度 (检查)
roll=0, pitch=π/4, yaw=π/6
```

## 🐛 故障排除

### "Action服务器不可用"
```bash
# 检查MoveIt是否启动
ros2 node list | grep move_group

# 检查action服务
ros2 action list | grep move_group
```

### "规划失败"
- 检查目标位姿是否在工作空间内
- 增加规划时间: `allowed_planning_time: 10.0`
- 调整容差: `tolerance_above/below: 0.1`

### "IK无解"
- 调整末端姿态
- 修改Z轴高度
- 检查关节限制

现在您可以完全按照样例的风格控制Piper机器人了！🚀
