# -*- coding: utf-8 -*-
"""
Created on Thu Oct 23 09:03:32 2025

@author: jireh
"""

# create incremental PI control for this lab we do not need deriviative so keep it simple
# created using motor architecture
# create a new task that manages PI control
#create new shared varaible to connect PI to motor effort tasks
# rewire old tasks
# create 2 indivudla effort shared varaibels
# we are in units of encoder ticks


from time import ticks_us, ticks_diff  # MicroPython provides these

class PI_control:
    """Pure PI controller with saturation + (optional) directional anti-windup.
       Keep sensors/actuators outside so this stays reusable.
    """
    __slots__ = (
        "kp", "ki", "umin", "umax", "directional_aw",
        "integral", "setpoint", "mode", "_last_u", "_last_t_us",
)

    def __init__(self, kp=0.0, ki=0.0, umin=-100.0, umax=100.0, directional_aw=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.umin = umin
        self.umax = umax
        self.directional_aw = bool(directional_aw)
        # state
        self.integral = 0.0          # stores (ki * ∫ e dt)
        self.setpoint = 0.0          # stores setpoint
        self.mode = "AUTO"           # or "MANUAL"
        self._last_u = 0.0
        self._last_t_us = None

    # Sets new output limits if need be
    def set_output_limits(self, umin=None, umax=None):
        self.umin = umin
        self.umax = umax

    # sets new tuning coefficients if need be
    def set_tunings(self, kp=None, ki=None):
        if kp is not None: self.kp = float(kp)
        if ki is not None: self.ki = float(ki)

    # Do we need see output?
    def reset(self, u=0.0):
        """Clear integral and (optionally) seed output."""
        self.integral = 0.0
        self._last_u = float(u)
        self._last_t_us = None

    def set_mode(self, mode="AUTO", manual_output=None, meas_for_bumpless=None):
        mode = mode.upper()
        # Make sure you're in one of the two mode options
        if mode not in ("AUTO", "MANUAL"):
            raise ValueError("mode must be 'AUTO' or 'MANUAL'")
        # Manual -> Auto: gather latest P term and preload integral to avoid jumps during switch
        if self.mode == "MANUAL" and mode == "AUTO" and (meas_for_bumpless is not None):
            e = self.setpoint - float(meas_for_bumpless)
            P = self.kp * e
            self.integral = self._clamp(self._last_u - P)
        self.mode = mode
        # Auto -> Manual: gather and store last output
        if mode == "MANUAL" and manual_output is not None:
            self._last_u = self._clamp(float(manual_output))

    # ---- core update ----> Have the options of defining a new setpoint, change of time(dt), timestamp(t_us), and feed-forward value(ff)
    def update(self, meas, dt=None, setpoint=None, ff=0.0, t_us=None):
        """Compute PI output with conditional anti-windup."""
        # If want to define new setpoint, do it here
        if setpoint is not None:
            self.setpoint = float(setpoint)

        # If mode is manual, just return latest output
        if self.mode == "MANUAL":
            return self._last_u

        # If you do not define a change of time, does it here automatically
        if dt is None:
            if t_us is None:
                t_us = ticks_us()
            if self._last_t_us is None:
                self._last_t_us = t_us
                return self._last_u
            dt = ticks_diff(t_us, self._last_t_us) * 1e-6
            self._last_t_us = t_us
        else:
            dt = float(dt)
        if dt <= 0.0:
            return self._last_u

        # Compute error, use it to find proportional gain and integral gain
        e = self.setpoint - float(meas)
        P = self.kp * e
        i_new = self.integral + self.ki * e * dt

        # Saturate requested effort with _clamp
        u_req = P + i_new + ff
        u_sat  = self._clamp(u_req)

        # Set allow_i flag = true to turn on integral if u_req (requested effort) is not saturated
        allow_i = True
        if self.directional_aw:
            sat_hi = (self.umax is not None) and (u_req > self.umax)
            sat_lo = (self.umin is not None) and (u_req < self.umin)
            if (sat_hi and e > 0) or (sat_lo and e < 0):
                allow_i = False
        else:
            if u_req != u_sat:
                allow_i = False

        # u_req not saturated? Keep integral control on
        if allow_i:
            self.integral = i_new
            u_out = u_sat
        else: # u_req saturated? Turn integral control off
            u_out = self._clamp(P + self.integral + ff)

        self._last_u = u_out
        return u_out

    # sets limits for requested effort, returns max/ min effort if requested effort saturates
    def _clamp(self, u):
        if (self.umin is not None) and (u < self.umin):
            return self.umin
        if (self.umax is not None) and (u > self.umax):
            return self.umax
        return u




# convert encoder counts-per-second to rad/s (H)
def cps_to_rad_s(cps, cpr):
    # 2*pi is tau in MicroPython 1.20+, otherwise use 6.283185307179586
    return (cps / cpr) * 6.283185307179586

pi = PI_control(kp=0.8, ki=4.0, umin=-100, umax=100)
pi.setpoint = 50.0  # rad/s

# # in your 1 kHz loop:
# cps = enc.read_cps()
# w_meas = cps_to_rad_s(cps, CPR)
# u = pi.update(meas=w_meas, dt=0.001)   # pass exact dt in seconds
# motor.set_duty(u)                       # sends duty [-100..100]



