import rclpy
from rclpy.node import Node
# 导入 JointState 消息类型
from sensor_msgs.msg import JointState

class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__('robot_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'robot_states',
            self.listener_callback,
            10)
        self.get_logger().info('订阅器已启动，正在等待/robot_states主题的数据...')
        self.counter = 0  # 添加计数器

    def listener_callback(self, msg):
        self.counter += 1
        if self.counter % 5 == 0:
            print("接收到消息：")
            max_name_length = max(len(name) for name in msg.name) if msg.name else 20
            for i in range(len(msg.name)):
                position = msg.position[i] if i < len(msg.position) else "N/A"
                velocity = msg.velocity[i] if i < len(msg.velocity) else "N/A"
                effort = msg.effort[i] if i < len(msg.effort) else "N/A"
                print(f"关节名称: {msg.name[i]:<{max_name_length}} 位置: {str(position):<8} 速度: {str(velocity):<8} 加速度: {str(effort):<8}")
            print("------------------------------")


def main(args=None):
    rclpy.init(args=args)
    subscriber = RobotStateSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()