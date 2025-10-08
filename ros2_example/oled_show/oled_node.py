import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import textwrap

class OledKeyboardCenter(Node):
    def __init__(self):
        super().__init__('oled_keyboard_center')

        # 初始化 OLED (128x64)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

        # 字体
        self.font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14
        )
        # Pillow 10+ 获取字体高度
        self.line_height = self.font.getbbox("A")[3]  # bbox = (x0, y0, x1, y1)，高度 = y1 - y0

        # 创建画布
        self.image = Image.new("1", (self.oled.width, self.oled.height))
        self.draw = ImageDraw.Draw(self.image)

        # OLED 清屏
        self.oled.fill(0)
        self.oled.show()

        # 订阅键盘话题
        self.subscription = self.create_subscription(
            String,
            'keyboard_topic',
            self.keyboard_callback,
            10
        )

    def keyboard_callback(self, msg: String):
        text = msg.data

        # 清屏
        self.draw.rectangle((0, 0, self.oled.width, self.oled.height), outline=0, fill=0)

        # 自动换行，宽度根据屏幕和字体大小估算
        max_chars_per_line = int(self.oled.width / (self.font.getbbox("A")[2] - self.font.getbbox("A")[0]))
        lines = textwrap.wrap(text, width=max_chars_per_line)

        # 如果总行数超过 OLED 高度，可显示的最大行数
        max_lines = self.oled.height // self.line_height
        if len(lines) > max_lines:
            lines = lines[:max_lines]

        # 计算垂直居中起点
        total_text_height = len(lines) * self.line_height
        y_start = (self.oled.height - total_text_height) // 2

        # 绘制每一行，水平居中
        for i, line in enumerate(lines):
            # 获取文本宽度
            bbox = self.draw.textbbox((0, 0), line, font=self.font)
            text_width = bbox[2] - bbox[0]
            x = (self.oled.width - text_width) // 2
            y = y_start + i * self.line_height
            self.draw.text((x, y), line, font=self.font, fill=255)

        # 显示画布
        self.oled.image(self.image)
        self.oled.show()

def main(args=None):
    rclpy.init(args=args)
    node = OledKeyboardCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
