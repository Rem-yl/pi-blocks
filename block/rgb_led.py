from time import sleep

from gpiozero import LED


class RGB_LED:
    def __init__(self, r_gpio: int = 7, g_gpio: int = 27, b_gpio: int = 22):
        self.r_gpio = r_gpio
        self.g_gpio = g_gpio
        self.b_gpio = b_gpio

        self.red_led = LED(self.r_gpio)
        self.green_led = LED(self.g_gpio)
        self.blue_led = LED(self.b_gpio)

    def red_on(self, delay: float = 1):
        self.red_led.on()
        self.green_led.off()
        self.blue_led.off()
        if delay > 0:
            sleep(delay)

    def green_on(self, delay: float = 1):
        self.red_led.off()
        self.green_led.on()
        self.blue_led.off()
        if delay > 0:
            sleep(delay)

    def blue_on(self, delay: float = 1):
        self.red_led.off()
        self.green_led.off()
        self.blue_led.on()
        if delay > 0:
            sleep(delay)

    def white_on(self, delay: int = 1):
        self.red_led.on()
        self.green_led.on()
        self.blue_led.on()
        if delay > 0:
            sleep(delay)

    def all_off(self):
        self.red_led.off()
        self.green_led.off()
        self.blue_led.off()


if __name__ == "__main__":
    rgb_led = RGB_LED()
    try:
        while True:
            rgb_led.red_on(1)
            rgb_led.white_on(1)
    except KeyboardInterrupt:
        rgb_led.all_off()
    except Exception as _:
        rgb_led.all_off()
