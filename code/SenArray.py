# -*- coding: utf-8 -*-
"""
Created on Thu Oct 30

@author: Aubrey
"""
# Import methods
from pyb import Pin, Timer, ADC
from LineSensor import SingleSensor

_TYPE_MAP = {
    'u8':  {'tc': 'B', 'bits': 8},      # 0..255
    'u16': {'tc': 'H', 'bits': 16},     # 0..65535
}

class SensorArray:
    '''A driver class to establish an array of line sensors'''
# Make sure to have a list of pins in main before calling this class? ex: pins = [B2, B3, C0, ...]
# @param pins: All the pins used in the sensor array
# @param sensor_cls: Just a rename for calling the SingeSensor class in LineSensor.py
# @param **sensor_kwargs: Just python language for "any other inputs". It fowards those inputs into self.sensors
    def __init__(self, pins, *, SingleSensor=SingleSensor, **sensor_kwargs):
        self.pins = list(pins)
        self.sensors = [SingleSensor(pin, **sensor_kwargs) for pin in self.pins]

    def __len__(self):
        return len(self.sensors)

    def __getitem__(self, i):
        return self.sensors[i]            # returns the object

    def read_sensor(self, i):
        if not (0 <= i < len(self.sensors)):
            raise IndexError("sensor index out of range")
        return self.sensors[i].read()     # returns one value

    def read_all(self):
        return [s.read() for s in self.sensors]  # list of values

    def read_all_normalized(self):
        return [s.read_normalized() for s in self.sensors]

    def calibrate_all_black(self, black_vals):
        if len(black_vals) != len(self.sensors):
            raise ValueError("black_vals length must match number of sensors")
        for s, v in zip(self.sensors, black_vals):
            s.calibrate_black(v)
        return [s.black_cal for s in self.sensors]

    def calibrate_all_white(self, white_vals):
        if len(white_vals) != len(self.sensors):
            raise ValueError("white_vals length must match number of sensors")
        for s, v in zip(self.sensors, white_vals):
            s.calibrate_white(v)
        return [s.white_cal for s in self.sensors]
