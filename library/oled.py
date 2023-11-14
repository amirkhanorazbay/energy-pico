from library.ssd1306 import SSD1306_I2C

class Oled:
    
    def __init__(self, WIDTH, HEIGHT, i2c1):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        try:
            self.oled = SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c1)
        except Exception as e:
            raise e
        
    def displayText(self, text, position=(0, 0), clear_oled=True, show_text=True):
        if clear_oled:
            self.oled.fill(0)
        self.oled.text(text, position[0], position[1])
        if show_text:
            self.oled.show()

    # Вывод 3-х строк (все) на дисплей
    def Oled(self, arS):
        self.oled.fill(0)
        y = [0,11,22]
        for i in range(3):
            self.oled.text(arS[i], 0, y[i])
        self.oled.show()


