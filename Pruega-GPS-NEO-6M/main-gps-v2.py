# ====== Pico 2 W: GPS (NEO-6M) + TCRT5000 + OLED SSD1306 ======
# WILLIAM LEON
# INGENIERIA EN TELECOMUNICACIONES
# OLED I2C1: GP14 SDA, GP15 SCL
# GPS UART1: GP4 TX, GP5 RX
# TCRT: A0 -> GP26 (ADC0). Umbral fijo. Vuelta = secuencia N-B-N-B
#=================================================================

from machine import Pin, UART, I2C, ADC
import time
from ssd1306 import SSD1306_I2C
from micropyGPS import MicropyGPS

# -------------------- CONFIGURACIÓN --------------------
ALTURA_OLED = 32
DIRECCION_OLED = 0x3C
DESFASE_TZ = -5            # Bogotá (UTC-5)

PIN_ADC = 26               # GP26 / ADC0
UMBRAL_BLANCO = 40000      # <40000=BLANCO ; >=40000=NEGRO

# Tiempos
REBOTE_FASE_MS  = 100      # antirrebote entre cambios N<->B
VENTANA_SEQ_MS  = 6000     # tiempo máx. para completar N-B-N-B
REFRESCO_MS     = 120      # refresco OLED
VUELTAS_META    = 6

# -------------------- HARDWARE --------------------
i2c  = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
oled = SSD1306_I2C(128, ALTURA_OLED, i2c, addr=DIRECCION_OLED)

uart_gps = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), timeout=1000)
gps = MicropyGPS(DESFASE_TZ)

adc  = ADC(PIN_ADC)

# -------------------- FUNCIONES AUXILIARES --------------------
def formatear_hora(ts):
    try:
        h, m, s = ts
        return "{:02d}:{:02d}:{:02d}".format(int(h), int(m), int(s))
    except:
        return "00:00:00"

def convertir_grados(secciones):
    try:
        g, m, hemi = secciones[0], secciones[1], secciones[2]
    except:
        return None
    if g == 0 and m == 0:
        return None
    dec = g + (m/60.0)
    if hemi in ('S','W'):
        dec = -dec
    return dec

def velocidad_kmh(valor_speed):
    if valor_speed is None:
        return None
    try:
        if isinstance(valor_speed, (list, tuple)):
            if len(valor_speed) >= 3:
                return float(valor_speed[2])
            if len(valor_speed) >= 1:
                return float(valor_speed[0]) * 1.852
        return float(valor_speed)
    except:
        return None

def oled_en_carrera(txt_hora, sats, hdop, txt_color, vueltas, vel_prom):
    oled.fill(0)
    oled.text("Hora: {}".format(txt_hora), 0, 0)
    oled.text("Sats:{} H:{}".format(sats, "-" if hdop is None else hdop), 0, 8)
    oled.text("TCRT: {}".format(txt_color), 0, 16)
    oled.text("Lap:{} Vel:{:.1f}".format(vueltas, vel_prom), 0, ALTURA_OLED - 8)
    oled.show()

def oled_ganaste(vel_prom_kmh):
    oled.fill(0)
    oled.text("!GANASTE!", 10, 8 if ALTURA_OLED==32 else 16)
    txt = "Prom:{:.1f}km/h".format(vel_prom_kmh if vel_prom_kmh is not None else 0)
    oled.text(txt, 0, ALTURA_OLED - 8)
    oled.show()

# -------------------- ESTADO ----------------------
# Secuencia esperada (True=NEGRO, False=BLANCO): N, B, N, B
secuencia_esperada = [True, False, True, False]
fase = 0                      # índice del próximo color esperado
ultimo_ms_fase = 0            # último cambio aceptado (debounce)
inicio_ventana_ms = 0         # inicio de ventana de secuencia

ultimo_color = None           # color previo (True/False)
meta = False
vueltas = 0
ultimo_refresco_ms = 0

velocidades_vuelta = []       # km/h en cada vuelta (muestreada al cerrar secuencia)
horas_vuelta = []             # hora GPS en cada vuelta
lat_vuelta = []
lon_vuelta = []

# -------------------- BUCLE PRINCIPAL ----------------
while True:
    ahora = time.ticks_ms()

    # Alimentar parser del GPS
    n = uart_gps.any()
    if n:
        datos = uart_gps.read(n)
        if datos:
            for b in datos:
                try:
                    gps.update(chr(b))
                except:
                    pass

    # Lectura TCRT y color actual
    valor_adc = adc.read_u16()
    es_negro = (valor_adc >= UMBRAL_BLANCO)   # True=NEGRO, False=BLANCO
    texto_color = "NEGRO" if es_negro else "BLANCO"

    # ---------- FSM N-B-N-B ----------
    if not meta:
        # Timeout de secuencia
        if fase != 0 and time.ticks_diff(ahora, inicio_ventana_ms) > VENTANA_SEQ_MS:
            fase = 0

        # Detectar flanco (cambio de color)
        if ultimo_color is None:
            ultimo_color = es_negro
        elif es_negro != ultimo_color:
            # antirrebote por fase
            if time.ticks_diff(ahora, ultimo_ms_fase) >= REBOTE_FASE_MS:
                ultimo_ms_fase = ahora
                ultimo_color = es_negro

                if fase == 0:
                    # Debe iniciar en NEGRO
                    if es_negro == secuencia_esperada[0]:
                        fase = 1
                        inicio_ventana_ms = ahora
                else:
                    # Verificar si el color actual es el esperado en esta fase
                    if es_negro == secuencia_esperada[fase]:
                        fase += 1
                        if fase == 4:
                            # Secuencia completa N-B-N-B => cuenta vuelta
                            hora_txt = formatear_hora(gps.timestamp)
                            vel = velocidad_kmh(getattr(gps, "speed", None))
                            lat = convertir_grados(gps.latitude)
                            lon = convertir_grados(gps.longitude)

                            velocidades_vuelta.append(vel if vel else 0)
                            horas_vuelta.append(hora_txt)
                            lat_vuelta.append(lat)
                            lon_vuelta.append(lon)

                            vueltas += 1
                            prom = (sum(velocidades_vuelta)/len(velocidades_vuelta)) if velocidades_vuelta else 0

                            # Consola: SOLO al completar secuencia
                            print("\n=== VUELTA {} ===".format(vueltas))
                            print("Color:", texto_color)  # termina en BLANCO
                            print("Hora:", hora_txt)
                            print("Lat:", lat)
                            print("Lon:", lon)
                            print("Velocidad:", "{:.1f} km/h".format(vel if vel else 0))
                            print("====================")

                            # ¿META?
                            if vueltas >= VUELTAS_META:
                                meta = True
                                prom_final = (sum(velocidades_vuelta)/len(velocidades_vuelta)) if velocidades_vuelta else 0
                                print("\n=== FIN DE CARRERA ===")
                                print("Vueltas: {}".format(vueltas))
                                print("Horas: {}".format(horas_vuelta))
                                print("Promedio velocidad: {:.1f} km/h".format(prom_final))
                                print("======================")
                                oled_ganaste(prom_final)
                            # Reiniciar para nueva secuencia
                            fase = 0
                    else:
                        # Color inesperado: resincronizar (si llega NEGRO, tomar como inicio)
                        if es_negro == secuencia_esperada[0]:
                            fase = 1
                            inicio_ventana_ms = ahora
                        else:
                            fase = 0

    # OLED durante carrera
    if not meta and time.ticks_diff(ahora, ultimo_refresco_ms) >= REFRESCO_MS:
        hora_txt = formatear_hora(gps.timestamp)
        sats = gps.satellites_in_use
        hdop = getattr(gps, "hdop", None)
        prom = (sum(velocidades_vuelta)/len(velocidades_vuelta)) if velocidades_vuelta else 0
        oled_en_carrera(hora_txt, sats, hdop, texto_color, vueltas, prom)
        ultimo_refresco_ms = ahora

    time.sleep_ms(2)


