#!/bin/bash
# Piper机器人MoveGroup Action命令 - 支持关节和位姿目标

source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查参数
MODE=${1:-"joint"}  # 默认关节模式
EXECUTE=${2:-"false"}  # 默认只规划不执行

echo "=================================================="
echo "  Piper机器人运动控制脚本"
echo "=================================================="
echo "模式: $MODE (joint/pose)"
echo "执行: $EXECUTE (true/false)"
echo "=================================================="

if [ "$MODE" = "pose" ]; then
    echo "发送位姿目标..."
    
    # 位姿目标：类似样例中的设置
    # pose_goal.pose.position.x = 0.28
    # pose_goal.pose.position.y = -0.2
    # pose_goal.pose.position.z = 0.5
    # pose_goal.pose.orientation.w = 1.0 (垂直向下)
    
    ros2 action send_goal /move_group/move moveit_msgs/action/MoveGroup "
    {
      request: {
        group_name: 'arm',
        pipeline_id: 'ompl',
        planner_id: 'RRTConnect',
        allowed_planning_time: 5.0,
        num_planning_attempts: 3,
        max_velocity_scaling_factor: 0.1,
        max_acceleration_scaling_factor: 0.1,
        start_state: { is_diff: true },
        goal_constraints: [
          {
            position_constraints: [
              {
                header: { frame_id: 'base_link' },
                link_name: 'link6',
                target_point_offset: { x: 0.0, y: 0.0, z: 0.0 },
                constraint_region: {
                  primitives: [
                    { type: 2, dimensions: [0.001] }
                  ],
                  primitive_poses: [
                    {
                      position: { x: 0.28, y: -0.2, z: 0.5 },
                      orientation: { x: 0.0, y: 0.707, z: 0.0, w: 0.707 }
                    }
                  ]
                },
                weight: 1.0
              }
            ],
            orientation_constraints: [
              {
                header: { frame_id: 'base_link' },
                link_name: 'link6',
                orientation: { x: 0.0, y: 0.707, z: 0.0, w: 0.707 },
                absolute_x_axis_tolerance: 0.1,
                absolute_y_axis_tolerance: 0.1,
                absolute_z_axis_tolerance: 0.1,
                weight: 1.0
              }
            ]
          }
        ]
      },
      planning_options: {
        plan_only: $EXECUTE,
        look_around: false,
        look_around_attempts: 0,
        max_safe_execution_cost: 0.0,
        replan: false,
        replan_attempts: 0,
        replan_delay: 0.0
      }
    }
    "
    
else
    echo "发送关节目标..."
    
    # 关节目标：移动到零位置
    ros2 action send_goal /move_group/move moveit_msgs/action/MoveGroup "
    {
      request: {
        group_name: 'arm',
        pipeline_id: 'ompl',
        planner_id: 'RRTConnect',
        allowed_planning_time: 5.0,
        num_planning_attempts: 3,
        max_velocity_scaling_factor: 0.1,
        max_acceleration_scaling_factor: 0.1,
        start_state: { is_diff: true },
        goal_constraints: [
          {
            joint_constraints: [
              { joint_name: 'joint1', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 },
              { joint_name: 'joint2', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 },
              { joint_name: 'joint3', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 },
              { joint_name: 'joint4', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 },
              { joint_name: 'joint5', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 },
              { joint_name: 'joint6', position: 0.0, tolerance_above: 0.05, tolerance_below: 0.05, weight: 1.0 }
            ]
          }
        ]
      },
      planning_options: {
        plan_only: $EXECUTE,
        look_around: false,
        look_around_attempts: 0,
        max_safe_execution_cost: 0.0,
        replan: false,
        replan_attempts: 0,
        replan_delay: 0.0
      }
    }
    "
fi

echo "=================================================="
echo "命令发送完成!"
echo "=================================================="
