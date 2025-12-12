# -*- coding: utf-8 -*-
"""
Created on Fri Oct  3 15:40:31 2025

@author: jireh
"""

# Import methods
from time import ticks_us, ticks_diff
from pyb import Pin, Timer

class Encoder:
    def __init__(self, tim_id, chA_pin, chB_pin, period=0xFFFF): # Period = Auto Reload Value
        self.position = 0 # Total accumulated position of the encoder
        self.prev_count = 0 # Counter value from the most recent update
        self.delta = 0 # Change in count between last two updates
        self.dt = 1e-3 # Amount of time between last two updates
        self.dt_us = 1 # Amount of time between last two updates in microseconds
        self._t_prev = ticks_us() # Counter value of constant timer
        self.mod = period + 1 # Used to detect wrap around (AR+1)
        self.half = self.mod >> 1 # Used to detect wrap around (AR+1/2)

        self.tim = Timer(tim_id, prescaler=0, period=period) # Sets timer for each encoder
        self.ch1 = self.tim.channel(1, Timer.ENC_AB, pin=Pin(chA_pin)) # Sets first channel for each encoder
        self.ch2 = self.tim.channel(2, Timer.ENC_AB, pin=Pin(chB_pin)) # Sets second channel for each encoder

        self.tim.counter(0) # Set timer counter to 0

# Establishing whether the change in encoder is negative or positive outside of update()
    def _signed_delta(self, new, old):
        d = new - old
        if d >  self.half - 1: d -= self.mod
        if d < -self.half:     d += self.mod
        return d

# update() method that is able to be within an ISR:
# to be ISR safe the method cannot have allocations, arithmetic, or prints
    def update(self):
        now = ticks_us()
        dt = ticks_diff(now, self._t_prev)
        if dt <= 0: dt = 1
        self._t_prev = now
        self.dt_us = dt

        cnt = self.tim.counter()
        d = self._signed_delta(cnt, self.prev_count)
        self.prev_count = cnt
        self.delta = d
        self.position += d

# Method to return the position of the encoder in encoder counts
    def get_position(self):
        return self.position

# Method to return the velocity of the motor in encoder counts per second (I think)
    def get_velocity(self):
        return self.delta / (self.dt_us * 1e-6) if self.dt_us > 0 else 0.0 # added change here

# Method to zero out encoder info
    def zero(self):
        self.position = 0
        self.prev_count = 0
        self.tim.counter(0)
