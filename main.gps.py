from machine import Pin, UART, I2C
import time
from ssd1306 import SSD1306_I2C
from micropyGPS import MicropyGPS

# ---------------------------
# Configuración de hardware
# ---------------------------
# I2C0 en GP8 (SDA) y GP9 (SCL) a 400kHz
i2c = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)

# OLED: usa 128x64 (ajusta a 128x32 si tu panel es más pequeño)
oled = SSD1306_I2C(128, 32, i2c)

# GPS en UART1 -> TX=GP4 (Pico), RX=GP5 (Pico)
# TX del GPS va al RX del Pico (GP5) y RX del GPS al TX del Pico (GP4)
gps_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), timeout=1000)

# Zona horaria para MicropyGPS (offset de horas respecto a UTC)
ZONA_HORARIA = -3
gps = MicropyGPS(ZONA_HORARIA)

# ---------------------------
# Utilidades
# ---------------------------
def convertir_latlon(secciones):
    """
    Convierte la tupla de MicropyGPS (grados, minutos, 'N'/'S'/'E'/'W') a string decimal con 6 decimales.
    Retorna None si no hay datos.
    """
    if not secciones:
        return None
    grados = secciones[0]
    minutos = secciones[1]
    hemi = secciones[2] if len(secciones) >= 3 else None

    if grados == 0 and minutos == 0:
        return None

    dec = grados + (minutos / 60.0)
    if hemi in ('S', 'W'):
        dec = -dec
    return "{0:.6f}".format(dec)

def formatear_hora(ts):
    """
    ts = gps.timestamp -> (hh, mm, ss)
    Puede venir vacío al inicio; en tal caso devuelve '--:--:--'
    """
    try:
        h, m, s = ts
        return "{:02d}:{:02d}:{:02d}".format(int(h), int(m), int(s))
    except:
        return "--:--:--"

def mostrar_en_oled_y_consola(lineas):
    """
    lineas: lista de strings (máx 4 para 128x64 a 16px de altura aprox)
    Dibuja en OLED y también hace print() de lo mismo.
    """
    oled.fill(0)
    y = 0
    for linea in lineas:
        # Consola
        print(linea)
        # OLED (ajusta Y si tu fuente se pisa: 0, 16, 32, 48)
        oled.text(linea, 0, y)
        y += 16
    oled.show()

# ---------------------------
# Bucle principal
# ---------------------------
print("Iniciando lectura GPS... (cielo abierto acelera el fix)")

ultimo_refresco_ms = time.ticks_ms()
while True:
    # 1) Leer lo que haya en el UART y alimentar el parser NMEA byte a byte
    nbytes = gps_uart.any()
    if nbytes and nbytes > 0:
        data = gps_uart.read(nbytes)
        if data:
            for b in data:
                try:
                    gps.update(chr(b))
                except:
                    # ignora caracteres raros
                    pass

    # 2) Cada ~500 ms refrescamos pantalla/consola
    ahora = time.ticks_ms()
    if time.ticks_diff(ahora, ultimo_refresco_ms) < 500:
        continue
    ultimo_refresco_ms = ahora

    # 3) Obtener datos listos
    latitud = convertir_latlon(gps.latitude)
    longitud = convertir_latlon(gps.longitude)
    sats = gps.satellites_in_use
    hora_txt = formatear_hora(gps.timestamp)

    # 4) Mostrar según disponibilidad
    if latitud is None or longitud is None:
        lineas = [
            "Satelites: {}".format(sats),
            "Datos no",
            "disponibles",
            "Hora: {}".format(hora_txt)
        ]
        mostrar_en_oled_y_consola(lineas)
        # Pequeño sleep para no saturar CPU
        time.sleep_ms(50)
        continue

    # 5) Si hay datos válidos, mostramos todo
    lineas = [
        "Satelites: {}".format(sats),
        "Lat: " + latitud,
        "Lon: " + longitud,
        "Hora: " + hora_txt
    ]
    mostrar_en_oled_y_consola(lineas)

    time.sleep_ms(50)
