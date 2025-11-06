# WILLIAM ANDRES LEON BETANCOURT
# COD:1401459
# COMUNICACION DIGITAL
# Protocolo digital de comunicación I2C
# UMNG
# mpu6050.py — MicroPython (I2C)
from machine import I2C
import struct, time

class MPU6050:
    def __init__(self, i2c: I2C, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        # Despertar
        self._write(0x6B, 0x00)   # PWR_MGMT_1 = 0
        time.sleep_ms(100)
        # Rango ±2g y ±250 dps
        self._write(0x1C, 0x00)   # ACCEL_CONFIG
        self._write(0x1B, 0x00)   # GYRO_CONFIG

    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def read_raw(self):
        # 14 bytes: ACCEL_X/Y/Z, TEMP, GYRO_X/Y/Z (16 bits)
        data = self._read(0x3B, 14)
        ax, ay, az, t, gx, gy, gz = struct.unpack('>hhhhhhh', data)
        return ax, ay, az, t, gx, gy, gz

    def read(self):
        ax, ay, az, t, gx, gy, gz = self.read_raw()
        ax_g, ay_g, az_g = ax/16384, ay/16384, az/16384      # g
        gx_dps, gy_dps, gz_dps = gx/131, gy/131, gz/131      # °/s
        temp_c = (t/340) + 36.53
        return {
            'ax_g': ax_g, 'ay_g': ay_g, 'az_g': az_g,
            'gx_dps': gx_dps, 'gy_dps': gy_dps, 'gz_dps': gz_dps,
            'temp_c': temp_c
        }
