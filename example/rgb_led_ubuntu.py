"""
在ubuntu22.04 LTS中运行的led控制代码

python3 example/rgb_led_ubuntu.py
"""
import time
from gpiod.line import Direction, Value
import gpiod

class RGB_LED:
    def __init__(self, r_gpio=17, g_gpio=22, b_gpio=27, chip="/dev/gpiochip4"):
        self.lines = [r_gpio, g_gpio, b_gpio]
        self.chip = chip

        config = {
            line: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            for line in self.lines
        }
        self.req = gpiod.request_lines(chip, consumer="rgb-led", config=config)

    def red_on(self, delay=1):
        self.req.set_value(self.lines[0], Value.ACTIVE)
        self.req.set_value(self.lines[1], Value.INACTIVE)
        self.req.set_value(self.lines[2], Value.INACTIVE)
        if delay > 0:
            time.sleep(delay)

    def green_on(self, delay=1):
        self.req.set_value(self.lines[0], Value.INACTIVE)
        self.req.set_value(self.lines[1], Value.ACTIVE)
        self.req.set_value(self.lines[2], Value.INACTIVE)
        if delay > 0:
            time.sleep(delay)

    def blue_on(self, delay=1):
        self.req.set_value(self.lines[0], Value.INACTIVE)
        self.req.set_value(self.lines[1], Value.INACTIVE)
        self.req.set_value(self.lines[2], Value.ACTIVE)
        if delay > 0:
            time.sleep(delay)

    def white_on(self, delay=1):
        for line in self.lines:
            self.req.set_value(line, Value.ACTIVE)
        if delay > 0:
            time.sleep(delay)

    def all_off(self):
        for line in self.lines:
            self.req.set_value(line, Value.INACTIVE)

if __name__ == "__main__":
    rgb = RGB_LED()
    try:
        while True:
            rgb.red_on(1)
            rgb.green_on(1)
            rgb.blue_on(1)
            rgb.white_on(1)
            rgb.all_off()
            time.sleep(1)
    except KeyboardInterrupt:
        rgb.all_off()
