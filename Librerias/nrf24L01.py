# nrf24l01.py
# Módulo para controlar el transceptor nRF24L01(+) con MicroPython
# Funciona con Raspberry Pi Pico / Pico W


from machine import Pin, SPI
import utime

# ---- Constantes de registro ----
CONFIG      = 0x00
EN_AA       = 0x01
EN_RXADDR   = 0x02
SETUP_AW    = 0x03
SETUP_RETR  = 0x04
RF_CH       = 0x05
RF_SETUP    = 0x06
STATUS      = 0x07
OBSERVE_TX  = 0x08
RX_ADDR_P0  = 0x0A
RX_ADDR_P1  = 0x0B
TX_ADDR     = 0x10
RX_PW_P0    = 0x11
RX_PW_P1    = 0x12
FIFO_STATUS = 0x17

# ---- Comandos ----
R_REGISTER    = 0x00
W_REGISTER    = 0x20
REGISTER_MASK = 0x1F
R_RX_PAYLOAD  = 0x61
W_TX_PAYLOAD  = 0xA0
FLUSH_TX      = 0xE1
FLUSH_RX      = 0xE2
NOP           = 0xFF

# ---- Bits ----
MASK_RX_DR  = 0x40
MASK_TX_DS  = 0x20
MASK_MAX_RT = 0x10
EN_CRC      = 0x08
CRCO        = 0x04
PWR_UP      = 0x02
PRIM_RX     = 0x01

RX_DR       = 0x40
TX_DS       = 0x20
MAX_RT      = 0x10

# ---- Driver ----
class NRF24L01:
    def __init__(self, spi, csn, ce, channel=76, payload_size=32):
        self.spi = spi
        self.csn = csn
        self.ce = ce
        self.channel = channel
        self.payload_size = payload_size

        self.csn.init(Pin.OUT, value=1)
        self.ce.init(Pin.OUT, value=0)

        self.pipe0_read_addr = None
        self.pipe1_read_addr = None

        self.flush_tx()
        self.flush_rx()
        self.power_down()
        self.set_channel(channel)
        self.set_payload_size(payload_size)
        self.set_data_rate(1)  # 1 Mbps
        self.set_power(3)      # Máxima potencia
        self.enable_crc(True)
        self.auto_ack(False)
        self.retransmit_count(3, 15)
        self.power_up()
        self.stop_listening()

    # --- SPI helpers ---
    def _reg_read(self, reg):
        self.csn(0)
        buf = self.spi.read(2, R_REGISTER | (REGISTER_MASK & reg))
        self.csn(1)
        return buf[1]

    def _reg_write(self, reg, value):
        self.csn(0)
        self.spi.write(bytearray([W_REGISTER | (REGISTER_MASK & reg), value]))
        self.csn(1)

    def _flush(self, cmd):
        self.csn(0)
        self.spi.write(bytearray([cmd]))
        self.csn(1)

    # --- Configuración básica ---
    def power_up(self):
        cfg = self._reg_read(CONFIG)
        if not cfg & PWR_UP:
            self._reg_write(CONFIG, cfg | PWR_UP)
            utime.sleep_ms(2)

    def power_down(self):
        cfg = self._reg_read(CONFIG)
        self._reg_write(CONFIG, cfg & ~PWR_UP)

    def set_channel(self, ch):
        self._reg_write(RF_CH, ch & 0x7F)

    def set_payload_size(self, size):
        self.payload_size = min(32, size)

    def set_data_rate(self, rate):
        setup = self._reg_read(RF_SETUP) & ~0x28
        if rate == 2:
            setup |= 0x08
        elif rate == 250:
            setup |= 0x20
        self._reg_write(RF_SETUP, setup)

    def set_power(self, level):
        setup = self._reg_read(RF_SETUP) & ~0x06
        setup |= (min(3, max(0, level)) << 1)
        self._reg_write(RF_SETUP, setup)

    def enable_crc(self, enable=True):
        cfg = self._reg_read(CONFIG)
        if enable:
            cfg |= EN_CRC
        else:
            cfg &= ~EN_CRC
        self._reg_write(CONFIG, cfg)

    def auto_ack(self, enable=False):
        self._reg_write(EN_AA, 0x3F if enable else 0x00)

    def retransmit_count(self, delay, count):
        delay = min(15, delay)
        count = min(15, count)
        self._reg_write(SETUP_RETR, (delay << 4) | count)

    # --- Pipes ---
    def open_tx_pipe(self, address):
        self.csn(0)
        self.spi.write(bytearray([W_REGISTER | TX_ADDR]) + address)
        self.spi.write(bytearray([W_REGISTER | RX_ADDR_P0]) + address)
        self.csn(1)
        self._reg_write(EN_RXADDR, self._reg_read(EN_RXADDR) | 0x01)
        self.pipe0_read_addr = address

    def open_rx_pipe(self, pipe_id, address):
        if pipe_id == 0:
            self.pipe0_read_addr = address
        elif pipe_id == 1:
            self.pipe1_read_addr = address
        reg = RX_ADDR_P0 + pipe_id
        self.csn(0)
        self.spi.write(bytearray([W_REGISTER | reg]) + address)
        self.csn(1)
        self._reg_write(EN_RXADDR, self._reg_read(EN_RXADDR) | (1 << pipe_id))
        pw_reg = RX_PW_P0 + pipe_id
        self._reg_write(pw_reg, self.payload_size)

    # --- Modo RX/TX ---
    def start_listening(self):
        self._reg_write(STATUS, RX_DR | TX_DS | MAX_RT)
        cfg = self._reg_read(CONFIG) | PRIM_RX | PWR_UP
        self._reg_write(CONFIG, cfg)
        self.ce(1)
        utime.sleep_us(130)

    def stop_listening(self):
        self.ce(0)
        cfg = self._reg_read(CONFIG) & ~PRIM_RX
        self._reg_write(CONFIG, cfg)
        self.flush_tx()
        self.flush_rx()

    # --- Envío / Recepción ---
    def any(self):
        """Devuelve True si hay datos en RX FIFO."""
        status = self._reg_read(STATUS)
        return bool(status & RX_DR)

    def recv(self):
        """Lee un payload (hasta 32 bytes)."""
        self.csn(0)
        self.spi.write(bytearray([R_RX_PAYLOAD]))
        data = self.spi.read(self.payload_size)
        self.csn(1)
        self._reg_write(STATUS, RX_DR)
        return data

    def send(self, buf, timeout=500):
        """Envía un buffer; espera confirmación o timeout (ms)."""
        self.stop_listening()
        self._flush(FLUSH_TX)
        self.csn(0)
        self.spi.write(bytearray([W_TX_PAYLOAD]) + buf)
        self.csn(1)
        self.ce(1)
        utime.sleep_us(15)
        self.ce(0)

        t0 = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), t0) < timeout:
            status = self._reg_read(STATUS)
            if status & TX_DS:
                self._reg_write(STATUS, TX_DS)
                return True
            elif status & MAX_RT:
                self._reg_write(STATUS, MAX_RT)
                self.flush_tx()
                return False
        self.flush_tx()
        return False

    # --- Limpieza ---
    def flush_tx(self):
        self._flush(FLUSH_TX)

    def flush_rx(self):
        self._flush(FLUSH_RX)
