import gc
from pyb import Pin, Timer, ADC, UART, repl_uart, ExtInt
import time
import closed_loop_task

def PI_controller_gen(Shares):  # task 5
    """
    Computes PI control for both motors and updates L_Effort/R_Effort shares.
    Wiring matches your block diagram:
      e = r - x̂  -> Kp + ∫Ki -> saturate [-100,100] -> motor effort

    Shares tuple order (must match the Task(...) call):
        (run_flag, Done, L_setpoint, R_setpoint, L_Effort, R_Effort, VLshare, VRshare)
    """
    state5 = 0
    (run_flag, Done, L_setpoint, R_setpoint, L_Effort, R_Effort, VLshare, VRshare, KP_L, KP_R, KI_L, KI_R) = Shares

    # persistent controllers (one per side)/ creating object
    LPI = closed_loop_task.PI_control(kp=KP_L.get(), ki=KI_L.get(), umin=-100, umax=100, directional_aw=True)
    RPI = closed_loop_task.PI_control(kp=KP_R.get(), ki=KI_R.get(), umin=-100, umax=100, directional_aw=True)

    # timing
    last_us = time.ticks_us()
    # If your VL/VR are already in rad/s, leave H_SCALE = 1.0
    H_SCALE = 1.0  # sensor scaling H (cps->rad/s, if needed). Adjust if your velocity units differ.

    while True:
        if state5 == 0:
            # idle / waiting
            if run_flag.get():
                # reset integrators on (re)start for clean step
                LPI.reset(u=L_Effort.get())
                RPI.reset(u=R_Effort.get())
                last_us = time.ticks_us()
                state5 = 1
                yield state5
            else:
                # keep motors off when not running
                L_Effort.put(0)
                R_Effort.put(0)
                state5 = 0
                yield state5

        elif state5 == 1:

            LPI.kp = float(KP_L.get())
            LPI.ki = float(KI_L.get())
            RPI.kp = float(KP_R.get())
            RPI.ki = float(KI_R.get())
            # compute precise dt
            now = time.ticks_us()
            dt = time.ticks_diff(now, last_us) * 1e-6
            last_us = now
            if dt <= 0.0 or dt > 0.2:  # sanity clamp against pauses
                dt = 0.01  # fall back to ~10 ms (task period)

            # read setpoints (ensure they're in the same units as VL/VR after H_SCALE) -> [r]
            Lsp = float(L_setpoint.get())
            Rsp = float(R_setpoint.get())

            # read measured velocities -> [x]
            vL = float(VLshare.get()) * H_SCALE  # H is currently 1
            vR = float(VRshare.get()) * H_SCALE

            # PI updates -> [a_star]
            # calls out PI tasks / functions
            uL = LPI.update(meas=vL, dt=dt, setpoint=Lsp)
            uR = RPI.update(meas=vR, dt=dt, setpoint=Rsp)

            # After: uL = LPI.update(...); uR = RPI.update(...)
            # DEBUG: check if we ever hit 100% duty
            if (uL >= 99.0 or uR >= 99.0):
                # pace output to avoid spam (every ~250 ms)
                if (time.ticks_ms() // 250) % 2 == 0:
                    # replace with your BT/USB write helper if you want it visible
                    # bt_writeline("SAT: uL=%d uR=%d  vL=%.0f vR=%.0f" % (int(uL), int(uR), vL, vR))
                    pass

            # write efforts (shares are int 'h'; PI already clamps to [-100,100])
            L_Effort.put(int(uL))
            R_Effort.put(int(uR))

            # stay active until told to stop
            if not run_flag.get():
                L_Effort.put(0)
                R_Effort.put(0)
                state5 = 0

        yield state5
