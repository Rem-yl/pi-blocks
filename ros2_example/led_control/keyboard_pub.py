import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardPub(Node):
    def __init__(self):
        super().__init__('keyboard_pub')
        self.pub = self.create_publisher(String, 'led_cmd', 10)

    def run(self):
        print("Press r/g/b/w/o to control LED, q to quit")
        while True:
            c = input(">")
            if c.lower() == 'q': break
            msg = String()
            msg.data = c
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPub()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
