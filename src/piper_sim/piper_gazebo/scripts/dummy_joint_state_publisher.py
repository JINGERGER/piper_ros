# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import time

# class JointAlternatePublisher(Node):
#     def __init__(self):
#         super().__init__('joint_alternate_publisher')
#         self.publisher = self.create_publisher(
#             JointState, 
#             '/joint_states', 
#             10
#         )
#         self.timer = self.create_timer(2.0, self.publish_joints)  # 2秒间隔
#         self.alternate = False
#         self.get_logger().info("关节交替发布器已启动...")

#     def publish_joints(self):
#         msg = JointState()
        
#         # 设置时间戳
#         now = self.get_clock().now().to_msg()
#         msg.header.stamp = now
#         msg.header.frame_id = ''
        
#         # 设置关节名称
#         msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
#         # 交替设置joint1位置
#         joint1_pos = 0.5 if self.alternate else 0.1
#         self.alternate = not self.alternate  # 切换状态
        
#         # 设置关节位置（其他关节固定）
#         msg.position = [
#             joint1_pos,  # joint1
#             0.2, 0.5, 0.5, 0.5, 0.5  # 其他关节
#         ]
        
#         # 发布消息
#         self.publisher.publish(msg)
#         self.get_logger().info(f"已发布: joint1={joint1_pos:.1f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = JointAlternatePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointAlternatePublisher(Node):
    def __init__(self, topic_name='joint_states'):
        super().__init__('joint_alternate_publisher')
        # 使用传入的 topic_name 参数设置发布的话题名
        self.publisher = self.create_publisher(
            JointState, 
            topic_name,  # 动态设置话题名
            10
        )
        self.timer = self.create_timer(2.0, self.publish_joints)  # 2秒间隔
        self.alternate = False
        self.get_logger().info(f"关节交替发布器已启动，发布到话题: {topic_name}")

    def publish_joints(self):
        msg = JointState()
        
        # 设置时间戳
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = ''
        
        # 设置关节名称
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 交替设置joint1位置
        joint1_pos = 0.5 if self.alternate else 0.1
        self.alternate = not self.alternate  # 切换状态
        
        # 设置关节位置（其他关节固定）
        msg.position = [
            joint1_pos,  # joint1
            0.2, 0.5, 0.5, 0.5, 0.5  # 其他关节
        ]
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f"已发布: joint1={joint1_pos:.1f}")

def main(args=None):
    rclpy.init(args=args)
    
    # 动态设置话题名
    topic_name = 'custom_joint_states'  # 修改为你需要的自定义话题名
    node = JointAlternatePublisher(topic_name=topic_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()