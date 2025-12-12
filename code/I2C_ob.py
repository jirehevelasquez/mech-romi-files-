# -*- coding: utf-8 -*-
"""
Created on Thu Oct 30

@author: Aubrey
"""
# Import methods
from pyb import Pin, Timer, ADC, I2C, delay
import struct

_TYPE_MAP = {
    'u8': {'tc': 'b', 'bits': 8},  # 0..255
    'u16': {'tc': 'h', 'bits': 16},  # 0..65535
}


class i2c_object:
    '''An I2C object class to create objects for devices on romi that use I2C communication, in this case, the IMU'''

    # @param: clock_pin = pin connected to the clock the IMU will use as datum timer for data transfer
    # @param: tim_id = setting which timer the pin will use
    # @param: data_pin = pin in which the 8 bit data will be sent back and forth to do stuff
    # @param: reset_pin = resets the IMU
    def __init__(self, i2c, reset_pin=None):
        self.i2c = i2c  # << use the object passed in
        # keep your register constants exactly as you had them
        self.OPR_MODE = 0x3D
        self.IMU_addr = 0x28
        self.ACC_OFFSET = 0x55
        self.GYR_OFFSET = 0x61
        self.RAD_OFFSET = 0x67
        self.CALIB_STAT = 0x35
        self.YAW = 0x1A
        self.YAW_RATE = 0x18

    # --------------- Helpers -------------------
    def _read(self, reg, n):
        # read n bytes from a register, returns a bytearray
        buf = bytearray(n)
        self.i2c.mem_read(buf, self.IMU_addr, reg)
        return buf

    # --------------- Methods -------------------

    def config_mode(self):
        # switch to CONFIGMODE (
        self.i2c.mem_write(0x00, self.IMU_addr, self.OPR_MODE)

    def imu_mode(self):
        # switch to IMU mode (
        self.i2c.mem_write(0x08, self.IMU_addr, self.OPR_MODE)

    def cal_status(self):
        s = self._read(self.CALIB_STAT, 1)[0]  # one byte
        sys = (s >> 6) & 0x03
        gyr = (s >> 4) & 0x03
        acc = (s >> 2) & 0x03
        mag = (s >> 0) & 0x03
        return sys, gyr, acc, mag

    def cal_coef(self):
        # write 00000000 into OP MODE to enable CONFIGMODE
        self.i2c.mem_write(0x00, self.IMU_addr, self.OPR_MODE)
        delay(25)
        # Setup byte array
        cal_coef_buf = bytearray(14)
        mv = memoryview(cal_coef_buf)
        # First put in the accelerometer calibration data (first 6 bytes of byte array)
        self.i2c.mem_read(mv[0:6], self.IMU_addr, self.ACC_OFFSET)
        # Next put in the gyroscope calibration data (next 6 bytes of byte array)
        self.i2c.mem_read(mv[6:12], self.IMU_addr, self.GYR_OFFSET)
        # Lastly put in the acceleration radius calibration coefficients (last 2 bytes)
        self.i2c.mem_read(mv[12:14], self.IMU_addr, self.RAD_OFFSET)
        # return to IMU mode
        self.i2c.mem_write(0x08, self.IMU_addr, self.OPR_MODE)
        delay(25)
        return cal_coef_buf

    def write_cal_coef(self, cal_coef_vals):

        if len(cal_coef_vals) != 14:
            raise ValueError("cal_coef_buf must be exactly 14 bytes")

        mv = memoryview(cal_coef_vals)  # zero-copy view

        # Enter CONFIGMODE
        self.i2c.mem_write(0x00, self.IMU_addr, self.OPR_MODE)
        delay(25)

        # Write slices to their correct regions
        self.i2c.mem_write(mv[0:6], self.IMU_addr, self.ACC_OFFSET)
        self.i2c.mem_write(mv[6:12], self.IMU_addr, self.GYR_OFFSET)
        self.i2c.mem_write(mv[12:14], self.IMU_addr, self.RAD_OFFSET)

        # Back to IMU mode
        self.i2c.mem_write(0x08, self.IMU_addr, self.OPR_MODE)
        delay(25)

        # try to fix this was previous
        # self.i2c.mem_write(0x00, self.IMU_addr, self.OPR_MODE)
        # # put in new calibration coefficients
        # self.i2c.mem_write(cal_coef_vals, self.IMU_addr, self.ACC_OFFSET)
        # # return to IMU mode
        # self.i2c.mem_write(0x08, self.IMU_addr, self.OPR_MODE)

    def read_yaw(self):
        # use the yaw reading from the IMU directly
        yaw_data = bytearray(2)
        self.i2c.mem_read(yaw_data, self.IMU_addr, self.YAW)
        yaw_data_lsb = struct.unpack('<h', yaw_data)[0]
        yaw_data_rad = float(yaw_data_lsb) / 900.0
        return yaw_data_rad  # returns in units of radians

    def read_yaw_rate(self):
        # use the yaw reading from the IMU directly, only yaw because pitch and roll is pointless for Romi
        yaw_rate_data = bytearray(2)
        self.i2c.mem_read(yaw_rate_data, self.IMU_addr, self.YAW_RATE)  # units of LSB
        yaw_rate_data_lsb = struct.unpack('<h', yaw_rate_data)[0]
        yaw_rate_rps = float(yaw_rate_data_lsb) / 900.0
        return yaw_rate_rps  # returns in units of rad/s
