"""
在ubuntu22.04 LTS中的oled_clock代码
"""
import time
from datetime import datetime

import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# -----------------------------
# 初始化 OLED (128x64)
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# -----------------------------
# 字体
# -----------------------------
font_small = ImageFont.load_default()
font_large = ImageFont.truetype(
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 24
)

# -----------------------------
# 创建画布
# -----------------------------
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)

# 清屏
oled.fill(0)
oled.show()

# -----------------------------
# 缓存变量
# -----------------------------
prev_time_str = ""
cpu_temp = None
last_temp_update = 0

# -----------------------------
# 主循环
# -----------------------------
while True:
    now = datetime.now()
    date_str = now.strftime("%Y / %m / %d")
    time_str = now.strftime("%H:%M:%S")

    # 每5秒更新一次 CPU 温度
    if time.time() - last_temp_update >= 5 or cpu_temp is None:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_raw = f.readline().strip()
            cpu_temp = float(temp_raw) / 1000.0
        except Exception:
            cpu_temp = None
        last_temp_update = time.time()

    temp_str = f"CPU:{cpu_temp:.1f}°C" if cpu_temp is not None else "CPU:N/A"

    # 仅当时间变化时才刷新画面
    if time_str != prev_time_str:
        # 清空画布
        draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

        # 顶部：日期（左）+ CPU温度（右）
        draw.text((0, 0), date_str, font=font_small, fill=255)
        right_x = oled.width - draw.textlength(temp_str, font=font_small)
        draw.text((right_x, 0), temp_str, font=font_small, fill=255)

        # 中间：大字体时间居中
        text_width = draw.textlength(time_str, font=font_large)
        x_pos = (oled.width - text_width) // 2
        y_pos = (oled.height - 24) // 2 + 5
        draw.text((x_pos, y_pos), time_str, font=font_large, fill=255)

        # 显示画布
        oled.image(image)
        oled.show()

        prev_time_str = time_str
