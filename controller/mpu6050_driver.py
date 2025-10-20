#!/usr/bin/env python3
"""
MPU6050 IMU Driver for ROS2

This module provides a ROS2-compatible driver for the MPU6050 6-axis IMU sensor.
It handles I2C communication, sensor configuration, and data reading with proper
ROS2 logging and parameter support.

Author: ROS2 MPU6050 Driver
License: MIT
"""

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import math
import logging
from typing import Tuple

# MPU6050 Register Map
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75
TEMP_OUT0 = 0x41

class MPU6050Driver:
    """
    ROS2-compatible MPU6050 IMU driver.
    
    This class provides a clean interface for reading data from the MPU6050
    6-axis IMU sensor with ROS2 logging and parameter support.
    """
    
    def __init__(self, node: Node, bus_number: int = 1, address: int = MPU6050_ADDR):
        """
        Initialize the MPU6050 driver.
        
        Args:
            node: ROS2 node instance for logging and parameters
            bus_number: I2C bus number (default: 1)
            address: MPU6050 I2C address (default: 0x68)
        """
        self.node = node
        self.bus_number = bus_number
        self.address = address
        self.logger = node.get_logger()
        
        # Initialize I2C bus
        try:
            self.bus = SMBus(self.bus_number)
            self.logger.info(f"SMBus initialized on bus {self.bus_number}, MPU6050 address 0x{self.address:02X}")
        except Exception as e:
            self.logger.error(f"Failed to initialize SMBus on bus {self.bus_number}: {e}")
            raise

    def initialize(self, accel_range: int = 1, gyro_range: int = 1):
        """
        Initialize the MPU6050 sensor.
        
        Args:
            accel_range: Accelerometer range setting (0-3)
                        Default 1 (±4g) - good for rover applications
            gyro_range: Gyroscope range setting (0-3)
                       Default 1 (±500°/s) - good for rover applications
        """
        try:
            # Wake up the sensor
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)
            self.logger.info("MPU6050 woken up from sleep mode")
            
            # Set sensor ranges
            self.set_accel_range(accel_range)
            self.set_gyro_range(gyro_range)
            
            # Verify sensor communication
            who_am_i = self.bus.read_byte_data(self.address, WHO_AM_I)
            self.logger.info(f"MPU6050 initialized successfully (WHO_AM_I: 0x{who_am_i:02x})")

            # Read initial sensor data for verification
            ax, ay, az = self.read_accel(g=True)
            gx, gy, gz = self.read_gyro()
            imu_temp = self.read_temp()
            self.logger.info(f"Initial accelerometer data: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}")
            self.logger.info(f"Initial gyroscope data: gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}")
            self.logger.info(f"Initial temperature: {imu_temp:.3f}°C")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize MPU6050: {e}")
            raise


    def set_accel_range(self, setting: int = 0):
        """
        Set accelerometer range and verify by reading back.
        
        Args:
            setting: Range setting (0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g)
        """
        ranges = ['±2g', '±4g', '±8g', '±16g']
        scales = [16384.0, 8192.0, 4096.0, 2048.0]

        if setting < 0 or setting > 3:
            error_msg = f"Invalid accelerometer range setting: {setting}. Must be 0-3."
            self.logger.error(error_msg)
            raise ValueError(error_msg)

        try:
            # Write the range
            self.bus.write_byte_data(self.address, ACCEL_CONFIG, setting << 3)
            self.accel_scale = scales[setting]

            # Read back and verify
            read_back = (self.bus.read_byte_data(self.address, ACCEL_CONFIG) >> 3) & 0x03
            if read_back == setting:
                self.logger.info(f"Accelerometer range set to {ranges[setting]} (scale: {self.accel_scale})")
            else:
                self.logger.warn(f"Accelerometer range mismatch! Set: {ranges[setting]}, Read: {ranges[read_back]}")
                
        except Exception as e:
            self.logger.error(f"Failed to set accelerometer range: {e}")
            raise

    def set_gyro_range(self, setting: int = 0):
        """
        Set gyroscope range and verify by reading back.
        
        Args:
            setting: Range setting (0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s)
        """
        ranges = ['±250°/s', '±500°/s', '±1000°/s', '±2000°/s']
        scales = [131.0, 65.5, 32.8, 16.4]

        if setting < 0 or setting > 3:
            error_msg = f"Invalid gyroscope range setting: {setting}. Must be 0-3."
            self.logger.error(error_msg)
            raise ValueError(error_msg)

        try:
            # Write the range
            self.bus.write_byte_data(self.address, GYRO_CONFIG, setting << 3)
            self.gyro_scale = scales[setting]

            # Read back and verify
            read_back = (self.bus.read_byte_data(self.address, GYRO_CONFIG) >> 3) & 0x03
            if read_back == setting:
                self.logger.info(f"Gyroscope range set to {ranges[setting]} (scale: {self.gyro_scale})")
            else:
                self.logger.warn(f"Gyroscope range mismatch! Set: {ranges[setting]}, Read: {ranges[read_back]}")
                
        except Exception as e:
            self.logger.error(f"Failed to set gyroscope range: {e}")
            raise


    def _read_word(self, reg: int) -> int:
        """
        Read a 16-bit signed value from the specified register.
        
        Args:
            reg: Register address to read from
            
        Returns:
            16-bit signed integer value
        """
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            value = (high << 8) + low
            if value >= 0x8000:
                value = -((65535 - value) + 1)
            return value
        except Exception as e:
            self.logger.error(f"Failed to read word from register 0x{reg:02X}: {e}")
            raise
    
    def read_temp(self) -> float:
        """
        Read the temperature from the onboard temperature sensor.
        
        Returns:
            Temperature in degrees Celsius
        """
        try:
            raw_temp = self._read_word(TEMP_OUT0)
            # Convert using formula from MPU-6050 Register Map and Descriptions revision 4.2, page 30
            actual_temp = (raw_temp / 340.0) + 36.53
            return actual_temp
        except Exception as e:
            self.logger.error(f"Failed to read temperature: {e}")
            raise
    def read_accel(self, g: bool = True, samples: int = 20) -> Tuple[float, float, float]:
        """
        Read accelerometer data from the MPU6050 and return the average of multiple samples.
        
        Args:
            g: If True, return data in g-force units. If False, return in m/s²
            samples: Number of samples to average (default=20)
            
        Returns:
            Tuple of (ax, ay, az) acceleration values averaged over the specified samples
        """
        try:
            ax_total = 0.0
            ay_total = 0.0
            az_total = 0.0
            
            for _ in range(samples):
                ax_total += (self._read_word(ACCEL_XOUT_H) / self.accel_scale) + 0.065
                ay_total += (self._read_word(ACCEL_XOUT_H + 2) / self.accel_scale) + 0.05
                az_total += (self._read_word(ACCEL_XOUT_H + 4) / self.accel_scale) + 0.05
                # Optional small delay for sensor stability
                # time.sleep(0.01)
            
            # Compute average
            ax = round(ax_total / samples, 2)
            ay = round(ay_total / samples, 2)
            az = round(az_total / samples, 2)
            
            if not g:
                # Convert g-force to m/s²
                GRAVITY_MS2 = 9.80665
                ax = round(ax * GRAVITY_MS2, 2)
                ay = round(ay * GRAVITY_MS2, 2)
                az = round(az * GRAVITY_MS2, 2)
                
            return ax, ay, az
            
        except Exception as e:
            self.logger.error(f"Failed to read accelerometer data: {e}")
            raise

    def read_gyro(self) -> Tuple[float, float, float]:
        """
        Read gyroscope data from the MPU6050.
        
        Returns:
            Tuple of (gx, gy, gz) angular velocity values in degrees/second
        """
        try:
            gx = self._read_word(GYRO_XOUT_H) / self.gyro_scale
            gy = self._read_word(GYRO_XOUT_H + 2) / self.gyro_scale
            gz = self._read_word(GYRO_XOUT_H + 4) / self.gyro_scale
            return gx, gy, gz
            
        except Exception as e:
            self.logger.error(f"Failed to read gyroscope data: {e}")
            raise

    def close(self):
        """Close the I2C bus connection."""
        try:
            self.bus.close()
            self.logger.info("SMBus closed and MPU6050 connection terminated")
        except Exception as e:
            self.logger.error(f"Error closing SMBus: {e}")


# Backward compatibility alias
MPU6050 = MPU6050Driver
