# =================================================================
# WILLIAM LEON
# INGENIERIA EN TELECOMUNICACIONES
# CARRO
# Pines (RP2040 / Pico 2 W)
#   NRF24L01 : SPI0 SCK GP2, MOSI GP3, MISO GP4, CSN GP5, CE GP6
#   OLED     : I2C1 SCL GP15, SDA GP14
#   GPS      : UART1 TX GP8, RX GP9
#   SERVO    : GP16
#   ESC      : GP17
#   MPU6050  : I2C0 SDA GP0, SCL GP1 
#   TCRT     : GP26 (ADC0)  --- Laps por N-B-N-B
# RX: NRF24L01 + GPS + OLED + SERVO + ESC + LAPS (TCRT N-B-N-B) 
#====================================================================
from machine import Pin, SPI, I2C, PWM, UART, ADC
import utime
from ssd1306 import SSD1306_I2C
from micropyGPS import MicropyGPS

# ==================== OLED ====================
i2c  = I2C(1, scl=Pin(15), sda=Pin(14), freq=400_000)
oled = SSD1306_I2C(128, 32, i2c)

def draw_main(hora="--:--:--", laps=0, vel_txt="--", sats=0, tcrt_txt="--"):
    # 4 líneas de 8 px para 128x32
    oled.fill(0)
    oled.text(f"Hora: {hora}",        0, 0)
    oled.text(f"Laps:{laps} Vel:{vel_txt}", 0, 8)
    oled.text(f"Sats:{sats}  TCRT:{tcrt_txt}", 0, 16)
    oled.text("RF:OK",                0, 24)  # simple indicador
    oled.show()

def draw_lap_flash(lap_num, hora, lat, lon):
    oled.fill(0)
    oled.text(f"LAP {lap_num}", 32, 0)
    oled.text(f"{hora}",        18, 8)
    # Muestra lat/lon abreviadas (6 decimales)
    lat_s = "{:.6f}".format(lat) if lat is not None else "--"
    lon_s = "{:.6f}".format(lon) if lon is not None else "--"
    oled.text(f"Lat:{lat_s[:12]}", 0, 16)
    oled.text(f"Lon:{lon_s[:12]}", 0, 24)
    oled.show()

def oled_ganaste(prom_kmh=None):
    oled.fill(0)
    oled.text("!GANASTE!", 20, 6)
    if prom_kmh is not None:
        oled.text("Prom:{:.1f}km/h".format(prom_kmh), 0, 20)
    else:
        oled.text("Prom: -- km/h", 0, 20)
    oled.show()

# ==================== NRF24L01 (SPI0 fijo) ====================
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

# ==================== SERVO / ESC ====================
SERVO_PIN = 16
servo = PWM(Pin(SERVO_PIN)); servo.freq(50)
SERVO_MIN_US = 500; SERVO_MAX_US = 2400; PERIOD_US = 20000
def servo_write_deg(angle):
    angle = 0 if angle < 0 else (180 if angle > 180 else angle)
    us = int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0))
    duty = int(us * 65535 // PERIOD_US); servo.duty_u16(duty)

ESC_PIN = 17
esc = PWM(Pin(ESC_PIN)); esc.freq(50)
ESC_MIN_US = 1000; ESC_MID_US = 1500; ESC_MAX_US = 2000
def esc_write_us(us):
    us = ESC_MIN_US if us < ESC_MIN_US else (ESC_MAX_US if us > ESC_MAX_US else us)
    duty = int(us * 65535 // PERIOD_US); esc.duty_u16(duty)
def speed_to_us(percent):
    percent = 100 if percent > 100 else (-100 if percent < -100 else percent)
    return int(ESC_MID_US + (ESC_MAX_US - ESC_MID_US) * (percent / 100.0))

# Traducción de órdenes de radio
def dir_to_angle_jst1(txt):
    t = (txt or "").strip().lower()
    if "izquierda" in t: return 30
    if "derecha"   in t: return 150
    return 90
def dir_to_speed_jst2(txt):
    t = (txt or "").strip().lower()
    if "adelante" in t: return 100
    if "reversa"  in t: return -100
    return 0

# ==================== GPS ====================
uart_gps = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9), timeout=1000)
gps = MicropyGPS(-5)  # Zona horaria Bogotá

def fmt_hora(ts):
    try:
        h, m, s = ts; return "{:02d}:{:02d}:{:02d}".format(int(h), int(m), int(s))
    except: return "--:--:--"

def conv_grados(secciones):
    try:
        g, m, hemi = secciones[0], secciones[1], secciones[2]
    except: return None
    if g == 0 and m == 0: return None
    dec = g + (m/60.0)
    if hemi in ('S','W'): dec = -dec
    return dec

def vel_kmh(v):
    if v is None: return None
    try:
        if isinstance(v,(list,tuple)):
            if len(v)>=3: return float(v[2])      # km/h (MicropyGPS)
            if len(v)>=1: return float(v[0])*1.852
        return float(v)
    except: return None

# ==================== TCRT (LAPS) ROBUSTO ====================
adc_tcrt = ADC(26)          # GP26 / ADC0

# Histéresis
UMBRAL_CENTRO   = 40000     # referencia central
H_MARGEN        = 2000      # margen de histéresis
TH_B2N          = UMBRAL_CENTRO + H_MARGEN  # subir: BLANCO->NEGRO
TH_N2B          = UMBRAL_CENTRO - H_MARGEN  # bajar: NEGRO->BLANCO

# Filtro y tiempos
ALPHA_FILTER    = 0.25      # 0..1
REBOTE_FASE_MS  = 100       # antirrebote de fase
MIN_SEG_MS      = 120       # permanencia mínima por color
VENTANA_SEQ_MS  = 6000      # ventana total para N-B-N-B
VUELTAS_META    = 6

# Estado filtro/color
filt_v          = None
color_actual    = None      # True=NEGRO, False=BLANCO
ultimo_cambio_ms= 0

# FSM
secuencia_esperada = [True, False, True, False]
fase = 0
inicio_ventana_ms = 0
ultimo_ms_fase = 0

# Laps/estadísticas
vueltas = 0
meta = False
velocidades_vuelta = []
last_lap_hora = "--:--:--"
last_lap_lat  = None
last_lap_lon  = None
lap_flash_until = 0

def _update_filtro(v_raw):
    global filt_v
    if filt_v is None:
        filt_v = v_raw
    else:
        filt_v = filt_v + ALPHA_FILTER * (v_raw - filt_v)
    return int(filt_v)

def _update_color_estable(now_ms, v_filtrado):
    """
    Devuelve (hubo_transicion, color_estable(True=NEGRO), texto_color)
    con histéresis y dwell mínimo.
    """
    global color_actual, ultimo_cambio_ms
    if color_actual is None:
        color_actual = (v_filtrado >= UMBRAL_CENTRO)
        ultimo_cambio_ms = now_ms
        return (False, color_actual, "NEGRO" if color_actual else "BLANCO")

    nuevo = color_actual
    if color_actual is False:  # BLANCO -> ¿NEGRO?
        if v_filtrado >= TH_B2N and (now_ms - ultimo_cambio_ms) >= MIN_SEG_MS:
            nuevo = True
    else:                      # NEGRO -> ¿BLANCO?
        if v_filtrado <= TH_N2B and (now_ms - ultimo_cambio_ms) >= MIN_SEG_MS:
            nuevo = False

    if nuevo != color_actual:
        color_actual = nuevo
        ultimo_cambio_ms = now_ms
        return (True, color_actual, "NEGRO" if color_actual else "BLANCO")
    else:
        return (False, color_actual, "NEGRO" if color_actual else "BLANCO")

# ==================== Parseo payload RF ====================
def parse_dirs(s):
    try:
        parts = s.split(';')
        d1 = d2 = None
        for p in parts:
            kv = p.split(':', 1)
            if len(kv) != 2: continue
            k = kv[0].strip().upper(); v = kv[1].strip()
            if k == "JST1": d1 = v
            if k == "JST2": d2 = v
        return d1, d2
    except:
        return None, None

# ==================== Inicialización ====================
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
draw_main("INICIAL", 0, "--", 0, "--")
utime.sleep_ms(1000)

# Seguridad de enlace RF
last_rx_ms = utime.ticks_ms()
KEEPALIVE_TIMEOUT_MS = 1200

# ==================== Bucle principal ====================
last_print = utime.ticks_ms()

while True:
    now = utime.ticks_ms()

    # ----- GPS no bloqueante -----
    n = uart_gps.any()
    if n:
        data = uart_gps.read(n)
        if data:
            for b in data:
                try: gps.update(chr(b))
                except: pass

    # ----- RF: recibe órdenes y gobierna SERVO/ESC -----
    st = reg_read(STATUS)
    if st & RX_DR:
        latest = None
        while True:
            csn(0); spi.write(bytearray([R_RX_PAYLOAD])); data = spi.read(PAYLOAD); csn(1)
            reg_write(STATUS, RX_DR)
            latest = data
            if reg_read(FIFO_STATUS) & 0x01: break

        msg = latest.rstrip(b"\x00")
        try: s = msg.decode("utf-8")
        except: s = str(msg)

        j1, j2 = parse_dirs(s)
        # SERVO (dirección)
        servo_write_deg(dir_to_angle_jst1(j1))
        # ESC (tracción)
        esc_write_us(speed_to_us(dir_to_speed_jst2(j2)))

        last_rx_ms = now
        led.toggle()

    # Seguridad: si se pierde RF, neutro en ESC
    if utime.ticks_diff(now, last_rx_ms) > KEEPALIVE_TIMEOUT_MS:
        esc_write_us(ESC_MID_US)

    # ----- LAPS con TCRT: filtro + histéresis + FSM N-B-N-B -----
    v_raw = adc_tcrt.read_u16()
    v_fil = _update_filtro(v_raw)
    transicion, es_negro, txt_color = _update_color_estable(now, v_fil)

    if not meta:
        # timeout de secuencia si estamos a mitad
        if fase != 0 and utime.ticks_diff(now, inicio_ventana_ms) > VENTANA_SEQ_MS:
            fase = 0

        if transicion:
            if utime.ticks_diff(now, ultimo_ms_fase) >= REBOTE_FASE_MS:
                ultimo_ms_fase = now
                if fase == 0:
                    # Debe iniciar en NEGRO
                    if es_negro is True:
                        fase = 1
                        inicio_ventana_ms = now
                else:
                    if es_negro == secuencia_esperada[fase]:
                        fase += 1
                        if fase == 4:
                            # ===== VUELTA COMPLETA =====
                            vueltas += 1

                            # Captura inmediata de GPS
                            last_lap_hora = fmt_hora(gps.timestamp)
                            last_lap_lat  = conv_grados(gps.latitude)
                            last_lap_lon  = conv_grados(gps.longitude)
                            spd_now = vel_kmh(getattr(gps, "speed", None))
                            if spd_now is not None:
                                velocidades_vuelta.append(spd_now)

                            print("\n=== VUELTA {} ===".format(vueltas))
                            print("Hora:", last_lap_hora, "Lat:", last_lap_lat, "Lon:", last_lap_lon,
                                  "Vel(km/h):", "{:.1f}".format(spd_now) if spd_now is not None else "--")
                            print("====================")

                            # Flash OLED 1s con datos de la vuelta
                            lap_flash_until = utime.ticks_add(now, 1000)

                            # Meta
                            if vueltas >= VUELTAS_META:
                                meta = True
                                prom = (sum(velocidades_vuelta)/len(velocidades_vuelta)) if velocidades_vuelta else None
                                esc_write_us(ESC_MID_US)   # seguridad al ganar
                                oled_ganaste(prom)
                                print("=== FIN DE CARRERA: GANASTE ===")
                            # Reinicia FSM
                            fase = 0
                    else:
                        # Color inesperado: resincroniza
                        if es_negro is True:
                            fase = 1
                            inicio_ventana_ms = now
                        else:
                            fase = 0

    # ----- OLED -----
    if not meta:
        if lap_flash_until and utime.ticks_diff(lap_flash_until, now) > 0:
            draw_lap_flash(vueltas, last_lap_hora, last_lap_lat, last_lap_lon)
        else:
            hora = fmt_hora(gps.timestamp)
            sats = gps.satellites_in_use
            spd = vel_kmh(getattr(gps, "speed", None))
            vtxt = "{:.1f}".format(spd) if spd is not None else "--"
            draw_main(hora, vueltas, vtxt, sats, txt_color)
            lap_flash_until = 0  # fin del flash
    # si meta, se mantiene la pantalla de GANASTE

    # ----- Log (1 s) -----
    if utime.ticks_diff(now, last_print) > 1000 and not meta:
        last_print = now
        hora = fmt_hora(gps.timestamp)
        sats = gps.satellites_in_use
        spd = vel_kmh(getattr(gps, "speed", None))
        vtxt = "{:.1f}".format(spd) if spd is not None else "--"
        print(f"[GPS] {hora}  Sats:{sats}  Vel:{vtxt} km/h  Laps:{vueltas}  TCRT:{txt_color}")

    utime.sleep_ms(10)
