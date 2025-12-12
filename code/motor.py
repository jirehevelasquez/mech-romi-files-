# -*- coding: utf-8 -*-
"""
Created on Thu Oct  2 08:47:33 2025

@author: jireh
"""
# Import methods
from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''

    def __init__(self, PWM, DIR, nSLP, timer_id, timer_ch, pwm_freq=20000):

        self.dir_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)      # direction
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)    # sleep (active high)
        self.pwm_pin = Pin(PWM, mode=Pin.AF_PP)

        if isinstance(timer_id, Timer):
            self.timer = timer_id
        else:
            self.timer = Timer(timer_id, freq=pwm_freq)
        self.ch = self.timer.channel(timer_ch, Timer.PWM, pin=self.pwm_pin)

        # State
        self._effort = 0
        self.enabled = False

# Define effort scale of -100 to 100
    def set_effort(self, effort):
        """Set motor effort between -100 and 100."""
        effort = max(-100, min(100, effort))  # clamp
        self._effort = effort

        if effort >= 0:
            self.dir_pin.low()
            duty = effort
        else:
            self.dir_pin.high()
            duty = -effort

        # apply duty if enabled
        if self.enabled:
            self.ch.pulse_width_percent(int(duty))
        else:
            self.ch.pulse_width_percent(0)

# Method to turn on motors
    def enable(self):
        """Wake up the driver and apply last effort."""
        self.nSLP_pin.high()
        self.enabled = True
        self.set_effort(self._effort)

# Method to turn off motors
    def disable(self):
        """Put driver in sleep (coast)."""
        self.enabled = False
        self.ch.pulse_width(0)
        self.nSLP_pin.low()