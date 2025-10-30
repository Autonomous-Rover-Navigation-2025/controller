#!/usr/bin/env python3
"""
MPU6050 IMU Driver for ROS2

Provides:
 - I2C access to MPU6050
 - Safe reading of accelerometer/gyroscope
 - Calibration and offset correction (in both g and m/s²)
 - Clean logging using ROS2 node's logger

Author: adapted for Sahil
License: MIT
"""

import time
from typing import Tuple
from smbus2 import SMBus

# -----------------------------
# MPU6050 Constants
# -----------------------------
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT0 = 0x41
WHO_AM_I = 0x75

GRAVITY_MS2 = 9.80665


class MPU6050Driver:
    """
    ROS2-compatible MPU6050 IMU driver.
    """

    def __init__(self, node, bus_number: int = 1, address: int = MPU6050_ADDR):
        self.node = node
        self.logger = node.get_logger()
        self.bus_number = bus_number
        self.address = address

        # Scales for conversion
        self.accel_scale = 16384.0  # ±2g
        self.gyro_scale = 131.0     # ±250°/s

        # Offsets (both in g and m/s²)
        self.accel_offset_g = None     # [ax_off_g, ay_off_g, az_off_g]
        self.accel_offset_ms2 = None   # [ax_off_ms2, ay_off_ms2, az_off_ms2]
        self.gyro_offset = None        # [gx_off_deg_s, gy_off_deg_s, gz_off_deg_s]

        try:
            self.bus = SMBus(self.bus_number)
            self.logger.info(f"✅ SMBus initialized on bus {self.bus_number}, MPU6050 address 0x{self.address:02X}")
        except Exception as e:
            self.logger.error(f"❌ Failed to initialize SMBus on bus {self.bus_number}: {e}")
            raise

    # ----------------------------
    # Initialization / configuration
    # ----------------------------
    def initialize(self, accel_range: int = 0, gyro_range: int = 0, auto_calibrate: bool = True):
        """Wake sensor, set ranges, and optionally auto-calibrate accelerometer and gyroscope."""
        try:
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
            time.sleep(0.05)
            self.set_accel_range(accel_range)
            self.set_gyro_range(gyro_range)

            who = self.bus.read_byte_data(self.address, WHO_AM_I)
            self.logger.info(f"MPU6050 initialize: WHO_AM_I = 0x{who:02x}")

            if auto_calibrate:
                self.calibrate_accelerometer(samples=100)
                self.calibrate_gyroscope(samples=100)

        except Exception as e:
            self.logger.error(f"Failed to initialize MPU6050: {e}")
            raise

    def set_accel_range(self, setting: int = 0):
        """Set accelerometer range and update scale."""
        scales = [16384.0, 8192.0, 4096.0, 2048.0]
        ranges = ['±2g', '±4g', '±8g', '±16g']
        if not (0 <= setting <= 3):
            raise ValueError("Invalid accelerometer range (0–3)")

        try:
            self.bus.write_byte_data(self.address, ACCEL_CONFIG, setting << 3)
            time.sleep(0.01)
            self.accel_scale = scales[setting]
            read_back = (self.bus.read_byte_data(self.address, ACCEL_CONFIG) >> 3) & 0x03
            if read_back != setting:
                self.logger.warning(f"⚠️ Accel range mismatch: set {ranges[setting]}, read {ranges[read_back]}")
            else:
                self.logger.info(f"Accelerometer range set to {ranges[setting]} (scale={self.accel_scale})")
        except Exception as e:
            self.logger.error(f"Failed to set accelerometer range: {e}")
            raise

    def set_gyro_range(self, setting: int = 0):
        """Set gyroscope range and update scale."""
        scales = [131.0, 65.5, 32.8, 16.4]
        ranges = ['±250°/s', '±500°/s', '±1000°/s', '±2000°/s']
        if not (0 <= setting <= 3):
            raise ValueError("Invalid gyroscope range (0–3)")

        try:
            self.bus.write_byte_data(self.address, GYRO_CONFIG, setting << 3)
            time.sleep(0.01)
            self.gyro_scale = scales[setting]
            read_back = (self.bus.read_byte_data(self.address, GYRO_CONFIG) >> 3) & 0x03
            if read_back != setting:
                self.logger.warning(f"⚠️ Gyro range mismatch: set {ranges[setting]}, read {ranges[read_back]}")
            else:
                self.logger.info(f"Gyroscope range set to {ranges[setting]} (scale={self.gyro_scale})")
        except Exception as e:
            self.logger.error(f"Failed to set gyroscope range: {e}")
            raise

    # ----------------------------
    # Low-level reads
    # ----------------------------
    def _read_word(self, reg: int) -> int:
        """Read signed 16-bit word from register."""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def _read_accel_raw(self) -> Tuple[float, float, float]:
        """Raw accelerometer reading → g-units."""
        ax = self._read_word(ACCEL_XOUT_H) / self.accel_scale
        ay = self._read_word(ACCEL_XOUT_H + 2) / self.accel_scale
        az = self._read_word(ACCEL_XOUT_H + 4) / self.accel_scale
        return ax, ay, az

    def _read_gyro_raw(self) -> Tuple[float, float, float]:
        """Raw gyroscope reading → deg/s."""
        gx = self._read_word(GYRO_XOUT_H) / self.gyro_scale
        gy = self._read_word(GYRO_XOUT_H + 2) / self.gyro_scale
        gz = self._read_word(GYRO_XOUT_H + 4) / self.gyro_scale
        return gx, gy, gz

    # ----------------------------
    # Public read functions
    # ----------------------------
    def read_accel(self, g: bool = True, samples: int = 20, delay_s: float = 0.0) -> Tuple[float, float, float]:
        """Return averaged accelerometer readings in g or m/s² with offset correction."""
        ax_total = ay_total = az_total = 0.0
        for _ in range(max(1, int(samples))):
            ax_g, ay_g, az_g = self._read_accel_raw()
            ax_total += ax_g
            ay_total += ay_g
            az_total += az_g
            if delay_s:
                time.sleep(delay_s)

        ax, ay, az = ax_total / samples, ay_total / samples, az_total / samples

        # Apply offset
        if g:
            if self.accel_offset_g is not None:
                ax -= self.accel_offset_g[0]
                ay -= self.accel_offset_g[1]
                az -= self.accel_offset_g[2]
            return round(ax, 1), round(ay, 1), round(az, 1)
        else:
            ax_ms2 = ax * GRAVITY_MS2
            ay_ms2 = ay * GRAVITY_MS2
            az_ms2 = az * GRAVITY_MS2
            if self.accel_offset_ms2 is not None:
                ax_ms2 -= self.accel_offset_ms2[0]
                ay_ms2 -= self.accel_offset_ms2[1]
                az_ms2 -= self.accel_offset_ms2[2]
            return round(ax_ms2, 1), round(ay_ms2, 1), round(az_ms2, 1)

    def read_gyro(self, samples: int = 10, delay_s: float = 0.0) -> Tuple[float, float, float]:
        """Return averaged gyroscope readings (deg/s) with offset correction."""
        gx_total = gy_total = gz_total = 0.0
        for _ in range(max(1, int(samples))):
            gx, gy, gz = self._read_gyro_raw()
            gx_total += gx
            gy_total += gy
            gz_total += gz
            if delay_s:
                time.sleep(delay_s)

        gx, gy, gz = gx_total / samples, gy_total / samples, gz_total / samples
        if self.gyro_offset is not None:
            gx -= self.gyro_offset[0]
            gy -= self.gyro_offset[1]
            gz -= self.gyro_offset[2]
        return round(gx, 1), round(gy, 1), round(gz, 1)

    # ----------------------------
    # Calibration
    # ----------------------------
    def calibrate_accelerometer(self, samples: int = 100, delay_s: float = 0.0):
        """Calibrate accelerometer (computes offset in both g and m/s²)."""
        self.logger.info(f"Starting accelerometer calibration ({samples} samples)... Keep sensor stationary.")
        ax_t = ay_t = az_t = 0.0
        for _ in range(samples):
            ax, ay, az = self._read_accel_raw()
            ax_t += ax
            ay_t += ay
            az_t += az
            if delay_s:
                time.sleep(delay_s)

        ax_m, ay_m, az_m = ax_t / samples, ay_t / samples, az_t / samples

        expected = (0.0, 0.0, 1.0)
        ax_off_g = ax_m - expected[0]
        ay_off_g = ay_m - expected[1]
        az_off_g = az_m - expected[2]

        self.accel_offset_g = [ax_off_g, ay_off_g, az_off_g]
        self.accel_offset_ms2 = [ax_off_g * GRAVITY_MS2,
                                 ay_off_g * GRAVITY_MS2,
                                 az_off_g * GRAVITY_MS2]

        self.logger.info(f"✅ Accelerometer calibrated:")
        self.logger.info(f"   Offset (g)   = {self.accel_offset_g}")
        self.logger.info(f"   Offset (m/s²)= {self.accel_offset_ms2}")
        return self.accel_offset_g, self.accel_offset_ms2

    def calibrate_gyroscope(self, samples: int = 200, delay_s: float = 0.0):
        """Calibrate gyroscope (offsets in deg/s)."""
        self.logger.info(f"Starting gyroscope calibration ({samples} samples)... Keep sensor stationary.")
        gx_t = gy_t = gz_t = 0.0
        for _ in range(samples):
            gx, gy, gz = self._read_gyro_raw()
            gx_t += gx
            gy_t += gy
            gz_t += gz
            if delay_s:
                time.sleep(delay_s)

        gx_m, gy_m, gz_m = gx_t / samples, gy_t / samples, gz_t / samples
        self.gyro_offset = [gx_m, gy_m, gz_m]
        self.logger.info(f"✅ Gyroscope calibrated: Offset (deg/s) = {self.gyro_offset}")
        return self.gyro_offset

    # ----------------------------
    # Cleanup
    # ----------------------------
    def close(self):
        try:
            self.bus.close()
            self.logger.info("✅ SMBus closed. MPU6050 connection terminated.")
        except Exception as e:
            self.logger.error(f"Error closing SMBus: {e}")
