"""
键盘输入, 向 keyboard_topic 发布
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_topic', 10)
        self.get_logger().info("Keyboard publisher started. Type something and press Enter:")

    def run(self):
        while rclpy.ok():
            try:
                key_input = input()  # 阻塞等待终端输入
            except EOFError:
                break  # Ctrl+D 退出
            msg = String()
            msg.data = key_input
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
