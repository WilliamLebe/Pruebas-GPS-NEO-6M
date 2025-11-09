# WILLIAM ANDRES LEON BETANCOURT
# COD:1401459
# COMUNICACION DIGITAL
# Protocolo digital de comunicación I2C
# UMNG
# ssd1306.py — MicroPython (I2C)
# Uso:
#   from machine import Pin, I2C
#   from ssd1306 import SSD1306_I2C
#   i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
#   oled = SSD1306_I2C(128, 64, i2c1, addr=0x3C)

from micropython import const
import framebuf
import time

# Comandos SSD1306
_SET_CONTRAST        = const(0x81)
_ENTIRE_ON           = const(0xA4)
_NORM_DISPLAY        = const(0xA6)
_INV_DISPLAY         = const(0xA7)
_DISPLAY_OFF         = const(0xAE)
_DISPLAY_ON          = const(0xAF)
_SET_DISP_OFFSET     = const(0xD3)
_SET_COMPINS         = const(0xDA)
_SET_VCOM_DETECT     = const(0xDB)
_SET_DISPLAY_CLOCK   = const(0xD5)
_SET_PRECHARGE       = const(0xD9)
_SET_MULTIPLEX       = const(0xA8)
_SET_START_LINE      = const(0x40)
_SET_MEM_ADDR        = const(0x20)
_SET_COLUMN_ADDR     = const(0x21)
_SET_PAGE_ADDR       = const(0x22)
_SET_SEG_REMAP       = const(0xA0)
_SET_COM_OUT_DIR     = const(0xC0)
_SET_COM_OUT_DIR_REM = const(0xC8)
_CHARGE_PUMP         = const(0x8D)

class SSD1306:
    def __init__(self, width, height, external_vcc=False):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        self.framebuf = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self._init_display()

    # API de framebuf más común
    def fill(self, c):                self.framebuf.fill(c)
    def pixel(self, x, y, c):         self.framebuf.pixel(x, y, c)
    def hline(self, x, y, w, c):      self.framebuf.hline(x, y, w, c)
    def vline(self, x, y, h, c):      self.framebuf.vline(x, y, h, c)
    def line(self, x1, y1, x2, y2, c):self.framebuf.line(x1, y1, x2, y2, c)
    def rect(self, x, y, w, h, c):    self.framebuf.rect(x, y, w, h, c)
    def fill_rect(self, x, y, w, h, c): self.framebuf.fill_rect(x, y, w, h, c)
    def text(self, s, x, y, c=1):     self.framebuf.text(s, x, y, c)
    def scroll(self, dx, dy):         self.framebuf.scroll(dx, dy)

    # Estas las define la subclase I2C/SPI
    def write_cmd(self, cmd):         raise NotImplementedError
    def write_data(self, buf):        raise NotImplementedError

    def poweroff(self):               self.write_cmd(_DISPLAY_OFF)
    def poweron(self):                self.write_cmd(_DISPLAY_ON)

    def contrast(self, contrast):
        self.write_cmd(_SET_CONTRAST)
        self.write_cmd(contrast & 0xFF)

    def invert(self, invert):
        self.write_cmd(_INV_DISPLAY if invert else _NORM_DISPLAY)

    def show(self):
        # Enviar todo el buffer a GDDRAM (modo horizontal)
        self.write_cmd(_SET_MEM_ADDR);   self.write_cmd(0x00)  # Horizontal
        self.write_cmd(_SET_COLUMN_ADDR); self.write_cmd(0); self.write_cmd(self.width - 1)
        self.write_cmd(_SET_PAGE_ADDR);   self.write_cmd(0); self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)

    def _init_display(self):
        self.write_cmd(_DISPLAY_OFF)
        self.write_cmd(_SET_DISPLAY_CLOCK); self.write_cmd(0x80)
        self.write_cmd(_SET_MULTIPLEX);     self.write_cmd(self.height - 1)
        self.write_cmd(_SET_DISP_OFFSET);   self.write_cmd(0x00)
        self.write_cmd(_SET_START_LINE | 0x00)
        self.write_cmd(_CHARGE_PUMP);       self.write_cmd(0x10 if self.external_vcc else 0x14)
        self.write_cmd(_SET_MEM_ADDR);      self.write_cmd(0x00)  # Horizontal
        self.write_cmd(_SET_SEG_REMAP | 0x01)
        self.write_cmd(_SET_COM_OUT_DIR_REM)
        if self.height == 64:
            self.write_cmd(_SET_COMPINS);   self.write_cmd(0x12)
            self.write_cmd(_SET_CONTRAST);  self.write_cmd(0x9F if self.external_vcc else 0xCF)
        else:
            self.write_cmd(_SET_COMPINS);   self.write_cmd(0x02)
            self.write_cmd(_SET_CONTRAST);  self.write_cmd(0x8F if self.external_vcc else 0xCF)
        self.write_cmd(_SET_PRECHARGE);     self.write_cmd(0x22 if self.external_vcc else 0xF1)
        self.write_cmd(_SET_VCOM_DETECT);   self.write_cmd(0x40)
        self.write_cmd(_ENTIRE_ON)
        self.write_cmd(_NORM_DISPLAY)
        self.write_cmd(_DISPLAY_ON)
        time.sleep_ms(100)
        self.fill(0); self.show()

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        # 0x80 = control byte "command"
        self.i2c.writeto(self.addr, b'\x80' + bytes((cmd & 0xFF,)))

    def write_data(self, buf):
        # 0x40 = control byte "data"
        # Enviar en bloques moderados para evitar límites de buffer
        for i in range(0, len(buf), 64):
            self.i2c.writeto(self.addr, b'\x40' + memoryview(buf)[i:i+64])
