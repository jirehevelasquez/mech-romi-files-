
import time
# from pyb import UART, repl_uart  # only to ensure REPL stays on USB if needed
import SenArray
import closed_loop_task
import pyb

# ---- Sensor array wiring ----
# Same pin set used in UI_gen.py so both modules see identical geometry
_PINS = ["PC3", "PB1", "PA4", "PA5", "PA6"]
sensor_array = SenArray.SensorArray(_PINS)


# ---- Helpers ----
def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _centroid(weights, positions):
    """Return weighted centroid or None if no weight."""
    sw = 0.0
    swp = 0.0
    for w, p in zip(weights, positions):
        sw += w
        swp += w * p
    if sw <= 1e-9:
        return None
    return swp / sw


def Light_PI(Shares):  # task 6
    """
    Shares tuple order (from main.py):
        (run_flag, Done, L_setpoint, R_setpoint, Line_dir,
         BASE_EFFORT, c_share, KP_STEER, KI_STEER,
         black_readings, white_readings)
    """
    state6 = 0
    count = 0
    tick7 = 0
    (run_flag, Done, L_setpoint, R_setpoint, Line_dir,
     BASE_EFFORT, c_share, KP_STEER, KI_STEER,
     black_readings, white_readings, PATH_STATE) = Shares

    # ---- Constants / steering model ----
    V_MAX = 6000.0  # cps
    STEER_SIGN = 1.0  # flip to -1.0 if steering is reversed
    HEADROOM_FRAC = 0.5
    N_SENS = len(_PINS)

    # Steering PI outputs a differential speed "u" in cps
    steerPI = closed_loop_task.PI_control(
        kp=float(KP_STEER.get()),
        ki=float(KI_STEER.get()),
        umin=-V_MAX, umax=V_MAX,
        directional_aw=True ,)

    steerPI.setpoint = 0.0 # fixed spacing

    last_t = None
    cal_applied = False

    # Precompute symmetric sensor positions: [-mid .. +mid]
    mid = (N_SENS - 1) / 2.0
    positions = [(i - mid) for i in range(N_SENS)]

    # added new pi didnt break the code....

    HeadingPI = closed_loop_task.PI_control(kp = 100, ki = 50, # update later
                                            umin=-V_MAX, umax=V_MAX,
                                            directional_aw=True ,)
    # ---------------------------------------------------------------------------------------------
    while True:

        # Keep tunings in sync with UI every tick (cheap)
        try:
            steerPI.set_tunings(kp=float(KP_STEER.get()), ki=float(KI_STEER.get()))
        except Exception:
            pass

        # ---- Apply calibration when BOTH queues have N values (non-blocking) ----
        try:
            if (black_readings.num_in() >= N_SENS) and (white_readings.num_in() >= N_SENS):
                blk = [black_readings.get() for _ in range(N_SENS)]
                wht = [white_readings.get() for _ in range(N_SENS)]
                try:
                    sensor_array.calibrate_all_black(blk)
                    sensor_array.calibrate_all_white(wht)
                    cal_applied = True
                except Exception:
                    # If lengths mismatch or other error, ignore and continue
                    pass
        except Exception:
            # If queues not initialized yet, just continue
            pass

        # ---- Compute dt safely ----
        t = time.ticks_us()
        if last_t is None:
            dt = 0.01  # first pass default
        else:
            dt = time.ticks_diff(t, last_t) / 1_000_000.0
            if dt <= 0.0:
                dt = 0.001
        last_t = t

        v_fwd = float(BASE_EFFORT.get())
        # ----------------------------------------------------------------------------------------------
        # If not running, just yield (keep last setpoints as-is)
        if int(run_flag.get()) == 0:
            # reset PI integrator so we don't carry windup across starts
            steerPI.reset(0.0)
            yield state6
            continue

        # ---- Read sensors and compute centroid ----
        try:
            norms = sensor_array.read_all_normalized()  # list of [0..1]
        except Exception:
            norms = [0.0] * N_SENS

        c_val = _centroid(norms, positions)

        # Optionally publish centroid (int8 Share). Scale by 10 to keep precision.
        try:
            if c_share is not None:
                if c_val is None:
                    c_share.put(0)
                else:
                    q = int(_clamp(round(c_val * 10.0), -127, 127))  # (_clamp(round(c_val * 10.0), -127, 127))
                    c_share.put(q)
        except Exception:
            pass

        # ---- Lost line? Drive straight. ---- might need to update this section later
        if c_val is None:
            vL = v_fwd
            vR = v_fwd
            L_setpoint.put(float(_clamp(vL, 0.0, V_MAX)))
            R_setpoint.put(float(_clamp(vR, 0.0, V_MAX)))
            yield state6
            continue

        # -------------------TESTS 1 ---------------------------------

        # turning right
        if PATH_STATE.get() == 1:  # checks if we are scewing the number
            c_val = c_val + 2
        elif PATH_STATE.get() == 2: # normal line follow for state 2
            c_val = c_val
        elif PATH_STATE.get() == 3: # go straight for state 3
            c_val = 0

        elif PATH_STATE.get() == 4: # line follow
            c_val = c_val

        elif PATH_STATE.get() == 5: # go straight
            c_val = -0.3
        elif PATH_STATE.get() == 6: # line follow
            c_val = c_val
        elif PATH_STATE.get() == 7:
            c_val = 0
        elif PATH_STATE.get() == 8:
            c_val = 0
        elif PATH_STATE.get() == 9: # follow line
            c_val = 0
        elif PATH_STATE.get() == 10:
            c_val = 0


        # ---------------------------------------------------------

        # when your on the line
        # didnt help...

        # room_plus = V_MAX - v_fwd  # how much the faster wheel can increase
        # room_minus = v_fwd  # how much the slower wheel can decrease
        # u_cap = min(HEADROOM_FRAC * V_MAX, room_plus, room_minus)
        # steerPI.set_output_limits(-u_cap, +u_cap)


        # # ---- PI steering about c) ----
        # u = steerPI.update(meas=c_val * STEER_SIGN, dt=dt)
        # # ---- Mix to wheel setpoints and clamp to forward-only [0, V_MAX] ----
        # vL = _clamp(v_fwd - u, 0, V_MAX)
        # vR = _clamp(v_fwd + u, 0, V_MAX)
        # # turns of light follwoing by no longer setting set points
        # L_setpoint.put(float(vL))
        # R_setpoint.put(float(vR))
        # yield state6

        if PATH_STATE.get() in (0 ,1 ,2 ,3 ,4 ,5 ,6):
            u = steerPI.update(meas=c_val * STEER_SIGN, dt=dt)

            # ---- Mix to wheel setpoints and clamp to forward-only [0, V_MAX] ----
            vL = _clamp(v_fwd - u, 0, V_MAX)
            vR = _clamp(v_fwd + u, 0, V_MAX)

            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))
            yield state6

        elif PATH_STATE.get() == 7:
            tick7 =tick7 +1
            print(tick7)

            if tick7 < 25:
                vL = -500
                vR = 500
            else:
                vL = 0
                vR = 0
                tick7 = 0
                PATH_STATE.put(8)
                print("outstate7", PATH_STATE.get())

            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))
            yield state6


        elif PATH_STATE.get() == 8:
            vL = v_fwd
            vR = v_fwd

            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))
            yield state6


        elif PATH_STATE.get() == 9:
            tick7 = tick7 + 1
            print(tick7)

            if tick7 < 150:
                vL = 500
                vR = -500
            else:
                vL = 0
                vR = 0
                PATH_STATE.put(10)
                print("outstate9", PATH_STATE.get())

            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))
            yield state6

        elif PATH_STATE.get() == 10:
            vL = v_fwd
            vR = v_fwd

            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))
            yield state6

        elif PATH_STATE.get() == 11:
            vL= -v_fwd
            vR = -v_fwd
            L_setpoint.put(float(vL))
            R_setpoint.put(float(vR))