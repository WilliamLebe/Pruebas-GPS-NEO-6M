# RECEPTOR
# RX - NRF24L01 + OLED (128x32) + SERVO (GP0) + ESC (GP1)
from machine import Pin, SPI, I2C, PWM
import utime
from ssd1306 import SSD1306_I2C

# ================= OLED =================
i2c  = I2C(1, scl=Pin(15), sda=Pin(14), freq=400_000)
oled = SSD1306_I2C(128, 32, i2c)
SCREEN_W, SCREEN_H = 128, 32

def _circle_points(x0, y0, x, y, put):
    put(x0 + x, y0 + y); put(x0 - x, y0 + y)
    put(x0 + x, y0 - y); put(x0 - x, y0 - y)
    put(x0 + y, y0 + x); put(x0 - y, y0 + x)
    put(x0 + y, y0 - x); put(x0 - y, y0 - x)

def draw_circle(x0, y0, r, filled=False):
    def put(x, y):
        if 0 <= x < SCREEN_W and 0 <= y < SCREEN_H:
            oled.pixel(x, y, 1)
    x, y, err = r, 0, 0
    if filled:
        while x >= y:
            for xi in range(x0 - x, x0 + x + 1):
                put(xi, y0 + y); put(xi, y0 - y)
                put(xi, y0 + x); put(xi, y0 - x)
                put(xi, y0 - y); put(xi, y0 - x)
            y += 1
            if err <= 0: err += 2*y + 1
            if err > 0:  x -= 1; err -= 2*x + 1
    else:
        while x >= y:
            _circle_points(x0, y0, x, y, put)
            y += 1
            if err <= 0: err += 2*y + 1
            if err > 0:  x -= 1; err -= 2*x + 1

def draw_indicator(link_ok):
    cx, cy, r = 5, SCREEN_H - 5, 3
    draw_circle(cx, cy, r, filled=link_ok)

def show(l1="", l2="", link_ok=False):
    oled.fill(0)
    oled.text(l1[:16], 0, 0)
    oled.text(l2[:16], 0, 16)
    draw_indicator(link_ok)
    oled.show()

# ================= NRF24L01 =================
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=1_000_000)
csn = Pin(5, Pin.OUT, value=1)
ce  = Pin(6, Pin.OUT, value=0)
led = Pin(25, Pin.OUT)

CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR = 0x00,0x01,0x02,0x03,0x04
RF_CH, RF_SETUP, STATUS                        = 0x05,0x06,0x07
RX_ADDR_P0, TX_ADDR                            = 0x0A,0x10
RX_PW_P0, FIFO_STATUS                          = 0x11,0x17
RX_DR, TX_DS, MAX_RT                           = 0x40,0x20,0x10
W_REGISTER, R_REGISTER, REGISTER_MASK          = 0x20,0x00,0x1F
FLUSH_TX, FLUSH_RX, R_RX_PAYLOAD               = 0xE1,0xE2,0x61

ADDR    = b'\xe7\xe7\xe7\xe7\xe7'
CHANNEL = 40
PAYLOAD = 32

def reg_write(reg, val):
    csn(0); spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK), val])); csn(1)
def reg_read(reg):
    csn(0); spi.write(bytearray([R_REGISTER | (reg & REGISTER_MASK)])); b = spi.read(1); csn(1); return b[0]
def reg_write_bytes(reg, data):
    csn(0); spi.write(bytearray([W_REGISTER | (reg & REGISTER_MASK)]) + data); csn(1)
def flush():
    csn(0); spi.write(bytearray([FLUSH_TX])); csn(1)
    csn(0); spi.write(bytearray([FLUSH_RX])); csn(1)
    reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

# ================= SERVO (dirección) =================
SERVO_PIN = 0
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)

SERVO_MIN_US = 500
SERVO_MAX_US = 2400
PERIOD_US    = 20000

def servo_write_deg(angle):
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    us = int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0))
    duty = int(us * 65535 // PERIOD_US)
    servo.duty_u16(duty)

def dir_to_angle_jst1(txt):
    t = (txt or "").strip().lower()
    if "izquierda" in t: return 30
    if "derecha"   in t: return 150
    return 90

# ================= ESC (motor trasero) =================
ESC_PIN = 1
esc = PWM(Pin(ESC_PIN))
esc.freq(50)

ESC_MIN_US = 1000
ESC_MID_US = 1500
ESC_MAX_US = 2000

def esc_write_us(us):
    if us < ESC_MIN_US: us = ESC_MIN_US
    if us > ESC_MAX_US: us = ESC_MAX_US
    duty = int(us * 65535 // PERIOD_US)
    esc.duty_u16(duty)

def speed_to_us(percent):
    if percent > 100: percent = 100
    if percent < -100: percent = -100
    return int(ESC_MID_US + (ESC_MAX_US - ESC_MID_US) * (percent / 100.0))

def dir_to_speed_jst2(txt):
    t = (txt or "").strip().lower()
    if "adelante" in t: return 100
    if "reversa"  in t: return -100
    return 0

# ================= Init RF =================
utime.sleep_ms(300)
reg_write(CONFIG, 0x0C)
reg_write(SETUP_AW, 0x03)
reg_write(EN_AA, 0x01)
reg_write(EN_RXADDR, 0x01)
reg_write(SETUP_RETR, 0x3F)
reg_write(RF_CH, CHANNEL)
reg_write(RX_PW_P0, PAYLOAD)
reg_write(RF_SETUP, 0x20 | (3<<1))
reg_write_bytes(RX_ADDR_P0, ADDR)
reg_write_bytes(TX_ADDR,   ADDR)
reg_write(CONFIG, 0x0B)
utime.sleep_ms(2)
flush()
ce(1)

servo_write_deg(90)
esc_write_us(ESC_MID_US)
show("RX listo", f"ch {CHANNEL}", link_ok=False)
utime.sleep_ms(1500)  # calibrar ESC neutro

# ================= Parseo =================
def parse_dirs(s):
    try:
        parts = s.split(';')
        d1 = d2 = None
        for p in parts:
            kv = p.split(':', 1)
            if len(kv) != 2: continue
            k = kv[0].strip().upper(); v = kv[1].strip()
            if k == "JST1": d1 = f"JST1: {v}"
            if k == "JST2": d2 = f"JST2: {v}"
        if d1 or d2: return (d1 or "JST1:?"), (d2 or "JST2:?")
    except: pass
    return ("JST1:?", s)

# ================= Bucle principal =================
ultimo_l1, ultimo_l2 = "JST1: ?", "JST2: ?"
last_rx_ms = utime.ticks_ms()
KEEPALIVE_TIMEOUT_MS = 1200

last_servo_angle = 90
last_speed = 0

while True:
    st = reg_read(STATUS)

    if st & RX_DR:
        # Leer último payload
        latest = None
        while True:
            csn(0); spi.write(bytearray([R_RX_PAYLOAD])); data = spi.read(PAYLOAD); csn(1)
            reg_write(STATUS, RX_DR)
            latest = data
            if reg_read(FIFO_STATUS) & 0x01: break

        msg = latest.rstrip(b"\x00")
        try: s = msg.decode("utf-8")
        except: s = str(msg)

        l1, l2 = parse_dirs(s)
        ultimo_l1, ultimo_l2 = l1, l2
        last_rx_ms = utime.ticks_ms()

        led.value(1)
        show(l1, l2, link_ok=True)
        led.value(0)

        # --- SERVO (JST1) ---
        try: dir1 = l1.split(":",1)[1].strip()
        except: dir1 = ""
        servo_write_deg(dir_to_angle_jst1(dir1))

        # --- MOTOR (JST2) ---
        try: dir2 = l2.split(":",1)[1].strip()
        except: dir2 = ""
        target_speed = dir_to_speed_jst2(dir2)
        esc_write_us(speed_to_us(target_speed))

    else:
        # Sin paquetes
        age = utime.ticks_diff(utime.ticks_ms(), last_rx_ms)
        if age > KEEPALIVE_TIMEOUT_MS:
            show("SIN ENLACE", ultimo_l2, link_ok=False)
            esc_write_us(ESC_MID_US)  # seguridad neutro
        else:
            show(ultimo_l1, ultimo_l2, link_ok=True)

    utime.sleep_ms(10)
