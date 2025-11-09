# TRANSMISOR
# NRF24L01 + OLED + 2 JOYSTICKS
from machine import Pin, SPI, I2C, ADC
from ssd1306 import SSD1306_I2C
import utime

# ===== Pines =====
# nRF24L01 por SPI0
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=1_000_000)
csn = Pin(5, Pin.OUT, value=1)
ce  = Pin(6, Pin.OUT, value=0)

# OLED SSD1306 por I2C1 (SCL=15, SDA=14)
i2c  = I2C(1, scl=Pin(15), sda=Pin(14), freq=400_000)
oled = SSD1306_I2C(128, 64, i2c)

def show(line1="", line2=""):
    oled.fill(0)
    oled.text(line1, 0, 0)
    oled.text(line2, 0, 16)
    oled.show()

# ===== Joysticks (ADC) =====
ADC0_PIN = 26   # JST1: Izquierda/Derecha
ADC1_PIN = 27   # JST2: Adelante/Reversa
jst1 = ADC(ADC0_PIN)
jst2 = ADC(ADC1_PIN)

# Calibración/umbrales (read_u16 -> 0..65535)
MID      = 32768
DEADZONE = 4000     # +/- zona muerta
LOW_T    = MID - DEADZONE
HIGH_T   = MID + DEADZONE

def dir_lr(val_u16):
    # Izquierda / Centro / Derecha
    if val_u16 < LOW_T:
        return "Izquierda"
    elif val_u16 > HIGH_T:
        return "Derecha"
    else:
        return "Centro"

def dir_fb(val_u16):
    # Reversa / Centro / Adelante
    if val_u16 < LOW_T:
        return "Reversa"
    elif val_u16 > HIGH_T:
        return "Adelante"
    else:
        return "Centro"

# ===== nRF: registros / comandos =====
CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR = 0x00,0x01,0x02,0x03,0x04
RF_CH, RF_SETUP, STATUS                        = 0x05,0x06,0x07
RX_ADDR_P0, TX_ADDR                            = 0x0A,0x10
RX_PW_P0, FIFO_STATUS                          = 0x11,0x17
RX_DR, TX_DS, MAX_RT                           = 0x40,0x20,0x10
W_REGISTER, R_REGISTER, REGISTER_MASK          = 0x20,0x00,0x1F
FLUSH_TX, FLUSH_RX, W_TX_PAYLOAD               = 0xE1,0xE2,0xA0

# ===== Parámetros RF (coinciden con el RX) =====
ADDR      = b'\xe7\xe7\xe7\xe7\xe7'  # 5 bytes
CHANNEL   = 40
PAYLOAD   = 32

# ===== utilidades de registro =====
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

# ===== Init & configuración por registros =====
utime.sleep_ms(300)

# Power-down inicial, luego configuración completa
reg_write(CONFIG, 0x0C)        # EN_CRC=1, PWR_UP=0
reg_write(SETUP_AW, 0x03)      # 5 bytes
reg_write(EN_AA, 0x01)         # Auto-ACK en pipe0
reg_write(EN_RXADDR, 0x01)     # habilita pipe0
reg_write(SETUP_RETR, 0x3F)    # max retries + delay
reg_write(RF_CH, CHANNEL)      # canal
# RF_SETUP: 250 kbps (bit5=1) + potencia=3 (bits2:1=11b)
reg_write(RF_SETUP, 0x20 | (3 << 1))
reg_write_bytes(TX_ADDR,   ADDR)
reg_write_bytes(RX_ADDR_P0, ADDR)   # necesario para ACK
reg_write(RX_PW_P0, PAYLOAD)

# Power up en PTX (PRIM_RX=0)
reg_write(CONFIG, 0x0A)        # EN_CRC=1, PWR_UP=1, PRIM_RX=0
utime.sleep_ms(2)
flush()

show("TX listo", f"ch {CHANNEL}")
utime.sleep_ms(500)

# ===== Bucle principal =====
last_payload = None
last_send_ms = utime.ticks_ms()
KEEPALIVE_MS = 500   # fuerza re-envío cada 500 ms
PERIOD_MS    = 120   # muestreo joysticks

def build_payload(d1, d2):
    # Texto corto y claro para RX y OLED
    # Ej: "JST1: Derecha; JST2: Adelante"
    msg = f"JST1:{d1};JST2:{d2}"
    s = msg.encode()
    if len(s) > PAYLOAD:
        s = s[:PAYLOAD]
    pad = s + b"\x00" * (PAYLOAD - len(s))
    return pad, msg

while True:
    t0 = utime.ticks_ms()

    v1 = jst1.read_u16()
    v2 = jst2.read_u16()
    d1 = dir_lr(v1)  # Izquierda/Centro/Derecha
    d2 = dir_fb(v2)  # Reversa/Centro/Adelante

    payload, msg = build_payload(d1, d2)

    # Mostrar en pantalla (siempre)
    show(f"JST1: {d1}", f"JST2: {d2}")

    # Sólo enviar si cambió o por keep-alive
    changed = (msg != last_payload)
    expired = utime.ticks_diff(t0, last_send_ms) >= KEEPALIVE_MS
    if changed or expired:
        # Cargar y transmitir
        csn(0); spi.write(bytearray([W_TX_PAYLOAD]) + payload); csn(1)
        ce(1); utime.sleep_us(15); ce(0)

        # Esperar TX_DS o MAX_RT (hasta 100 ms)
        t1 = utime.ticks_ms(); ok = False
        while utime.ticks_diff(utime.ticks_ms(), t1) < 100:
            st = reg_read(STATUS)
            if st & TX_DS:
                reg_write(STATUS, TX_DS)
                ok = True
                break
            if st & MAX_RT:
                reg_write(STATUS, MAX_RT)
                flush()
                ok = False
                break

        # Opcional: imprime para depurar
        print(f'TX: "{msg}" | ACK: {ok}')
        last_payload = msg
        last_send_ms = t0

    # Periodo de muestreo
    # (ajusta si quieres más/menos “fluidez”)
    # Usa ticks_diff para espera relativa precisa
    while utime.ticks_diff(utime.ticks_ms(), t0) < PERIOD_MS:
        utime.sleep_ms(1)
