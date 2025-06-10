import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class JointStatesToTrajectory(Node):
    def __init__(self):
        super().__init__('joint_states_to_trajectory')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/custom_joint_states', self.joint_states_callback, 10)
        self.last_log_time = 0  # 初始化上次打印时间戳


    def joint_states_callback(self, msg):
        self.get_logger().info(f"Received JointState message: names={msg.name}, positions={msg.position}, velocities={msg.velocity}")

        # 检查关节名称是否匹配
        if not all(joint in msg.name for joint in self.joint_names):
            self.get_logger().warn('Received joint states do not match expected joint names.')
            return

        # 创建 JointTrajectory 消息
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # 创建 JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = [msg.position[msg.name.index(joint)] for joint in self.joint_names]
        point.time_from_start.sec = 1  # 设置时间为 1 秒
        trajectory_msg.points.append(point)

        # 发布消息
        self.publisher.publish(trajectory_msg)
        
        # 限制日志打印频率（每隔 2 秒打印一次）
        current_time = time.time()
        if current_time - self.last_log_time > 5:  # 2 秒间隔
            self.get_logger().info(f'Published trajectory: {trajectory_msg}')
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()