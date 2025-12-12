# -*- coding: utf-8 -*-
"""
Created on Thu Oct 30

@author: Aubrey
"""
# Import methods
from pyb import Pin, Timer, ADC

_TYPE_MAP = {
    'u8':  {'tc': 'B', 'bits': 8},      # 0..255
    'u16': {'tc': 'H', 'bits': 16},     # 0..65535
}

class SingleSensor:
    '''A line sensor driver class to establish an object related to one IR sensor on Romi'''
# @param SensorPin: The pin on the Nucleo corresponding to the output of one sensor
# @param black_cal: The calibration value for a black surface
# @param white_cal: The calibration value for a white surface
# @param typecode: Defines size of sensor value reading (NOT DOING ANYTHING YET)
    def __init__(self, sensor_pin, black_cal=0, white_cal=65535, typecode='u16'):
        if typecode not in _TYPE_MAP:
            raise ValueError("typecode must be one of: " + ", ".join(_TYPE_MAP))

        self._adc = ADC(Pin(sensor_pin))
        self.black_cal = black_cal
        self.white_cal = white_cal
        self.sen_val = 0

    def read(self):
        v = self._adc.read()
        self.sen_val = v
        return v

    def read_normalized(self):
        v = self.read()  # latest ADC value (int)
        denom = float(self.black_cal - self.white_cal)
        if denom == 0:
            return 0.0
        n = (float(v) - float(self.white_cal)) / denom
        # clamp to [0,1] in case of noise
        return 0.0 if n < 0.0 else 1.0 if n > 1.0 else n

    def calibrate_black(self, value):
        self.black_cal = int(value)

    def calibrate_white(self, value):
        self.white_cal = int(value)

    def clear(self):
        self.sen_val.value(0)