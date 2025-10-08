import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiod.line import Direction, Value
import gpiod

class RGB_LED:
    def __init__(self, r_gpio=17, g_gpio=22, b_gpio=27, chip="/dev/gpiochip4"):
        self.lines = [r_gpio, g_gpio, b_gpio]
        self.req = gpiod.request_lines(chip, consumer="rgb-led",
            config={line: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE) for line in self.lines})

    def red_on(self): self.set_color([1,0,0])
    def green_on(self): self.set_color([0,1,0])
    def blue_on(self): self.set_color([0,0,1])
    def white_on(self): self.set_color([1,1,1])
    def all_off(self): self.set_color([0,0,0])
    def set_color(self, vals):
        for i,v in enumerate(vals):
            self.req.set_value(self.lines[i], Value.ACTIVE if v else Value.INACTIVE)

class LEDNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.rgb = RGB_LED()
        self.subscription = self.create_subscription(String, 'led_cmd', self.led_callback, 10)

    def led_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'r': self.rgb.red_on()
        elif cmd == 'g': self.rgb.green_on()
        elif cmd == 'b': self.rgb.blue_on()
        elif cmd == 'w': self.rgb.white_on()
        elif cmd == 'o': self.rgb.all_off()

def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
