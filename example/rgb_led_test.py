"""
在Raspberry Pi OS中使用gpiozero库控制的代码
具体环境搭建参看树莓派5学习手册文档
"""
from block import RGB_LED

rgb_led = RGB_LED()
try:
    while True:
        rgb_led.red_on(0.5)
        rgb_led.white_on(0.5)
except KeyboardInterrupt:
    rgb_led.all_off()
except Exception as _:
    rgb_led.all_off()
