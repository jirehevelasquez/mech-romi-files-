# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
# -*- coding: utf-8 -*-
"""
updated 10/28/25

@author: Jireh and Aubrey
"""

"ME 405 Lab 0x03"

import gc
from pyb import Pin, Timer, ADC, UART, repl_uart, ExtInt
import uos
import cotask
import task_share
import pyb, micropython, time
import Light, PI_con, UI_gen, data_col, 
import right_ME_gen, left_ME_gen, Track
from math import pi, cos, sin
from I2C_ob import i2c_object
from I2C_ob import i2c_object
from UI_gen import imu

""" updates 1. new matrix to account for time
            2. new SL, and SR to be in mm/sec and not rad/sec
            3. added Y to measure error propagation
            4. added new display word "position"
            5. working on task 8 track
            """

# --- Bluetooth REPL on UART3 ---
# u = UART(3, 115200)  # switches the REPL from USB to bluetooth
# u.init(115200, timeout=0)  # non-blocking reads
# repl_uart(u)
# repl back to usb code below, incase things go bad
repl_uart(None)  # ensure REPL is on USB, not on a UART
u = pyb.USB_VCP()

micropython.alloc_emergency_exception_buf(256)  # helps degugging in putty


# ---------------------------------------------------------------------------


def IMU(Shares):  # task 7, state estimation
    # users interface has commands to read imu settings

    state7 = 0
    global imu
    (run_flag, L_Effort, R_Effort, VLshare, VRshare,
     Xhat1, Xhat2, Xhat3, Xhat4, Xpos, Ypos, Yhat1, Yhat2, Yhat3, Yhat4) = Shares

    # --------------------------------------------------------------------------
    def mul4x4_vec4(M, v):
        # returns length-4 tuple
        return (
            M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2] + M[0][3] * v[3],
            M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2] + M[1][3] * v[3],
            M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2] + M[2][3] * v[3],
            M[3][0] * v[0] + M[3][1] * v[1] + M[3][2] * v[2] + M[3][3] * v[3],
        )

    def mul4x6_vec6(M, v):
        # returns length-4 tuple
        return (
            M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2] + M[0][3] * v[3] + M[0][4] * v[4] + M[0][5] * v[5],
            M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2] + M[1][3] * v[3] + M[1][4] * v[4] + M[1][5] * v[5],
            M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2] + M[2][3] * v[3] + M[2][4] * v[4] + M[2][5] * v[5],
            M[3][0] * v[0] + M[3][1] * v[1] + M[3][2] * v[2] + M[3][3] * v[3] + M[3][4] * v[4] + M[3][5] * v[5],
        )

    def add4(a, b):
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3])

    # ---------------------------------------------------------------------------
    ticks_per_edge = 12
    Counts_per_rev = 119.7576
    Wheel_radius_mm = 35
    VBAT_VOLTS = 9  # revise later update
    TS = 0.05

    """ notes, need to update matrices based on period so pick one closet to period"""

    mm_per_tick = (2 * pi * Wheel_radius_mm) / (ticks_per_edge * Counts_per_rev)
    mm_per_ticks_per_sec = mm_per_tick * 1000000
    rad_per_tick = (2.0 * pi) / (ticks_per_edge * Counts_per_rev)  # spits out

    # calculated these matrices with mp = 5% and tau = .1134 seconds, and T_step = 0.01 seconds
    #  A_D = (
    #     ( 0.0012,  0.0012, -0.0004,  0.0000),
    #     ( 0.0012,  0.0012, -0.0004,  0.0000),
    #     ( 0.0000,  0.0000,  0.0025,  0.0000),
    #     (-0.0000, -0.0000,  0.0000,  0.0000))

    #  B_d = (
    #     ( 0.0483,  0.0417,  0.0002,  0.0002, -0.0000, -2.0102),
    #     ( 0.0417,  0.0483,  0.0002,  0.0002,  0.0000,  2.0102),
    #     ( 0.0027,  0.0027,  0.4988,  0.4988, -0.0000,  0.0000),
    #     ( 0.0000,  0.0000, -0.0071,  0.0071,  0.0001,  0.0002))

    # # C_d kept in case you want y_hat for debugging
    #  C_d = (
    #     ( 0.0,     0.0,     1.0,   -70.5000),
    #     ( 0.0,     0.0,     1.0,    70.5000),
    #     ( 0.0,     0.0,     0.0,     1.0000),
    #     (-0.2482,  0.2482,  0.0,     0.0000))

    #  D_d = (
    #  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    # calculated these matrices with mp = 5% and tau = .1134 seconds, and T_step = 0.1 seconds
    # A_D = (
    #     (0.4868e-13, 0.4868e-13, -0.6987e-13, 0.0),
    #     (0.4868e-13, 0.4868e-13, -0.6987e-13, 0.0),
    #     (0.0023e-13, 0.0023e-13, 0.8961e-13, 0.0),
    #     (-0.0, -0.0, 0.0, 0.0))

    # B_d = (
    #     (0.0484, 0.0418, 0.0000, 0.0000, -0.0000, -2.0102),
    #     (0.0418, 0.0484, 0.0000, 0.0000, 0.0000, 2.0102),
    #     (0.0027, 0.0027, 0.5000, 0.5000, -0.0000, 0.0000),
    #     (0.0000, 0.0000, -0.0071, 0.0071, 0.0001, 0.0002))

    # C_d = (
    #     (0.0, 0.0, 1.0, -70.5000),
    #     (0.0, 0.0, 1.0, 70.5000),
    #     (0.0, 0.0, 0.0, 1.0000),
    #     (-0.2482, 0.2482, 0.0, 0.0))

    # D_D = (
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    #     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    # calculated with T_step = 0.05 or 20 hz
    A_D = [
        [-1.561e-7, -1.561e-7, 1.142e-7, 0.0],
        [-1.561e-7, -1.561e-7, 1.142e-7, 0.0],
        [-4.000e-10, -4.000e-10, -2.995e-7, 0.0],
        [0.0, 0.0, 0.0, 0.0], ]

    B_d = [
        [0.0484, 0.0418, -0.0000, -0.0000, -0.0000, -2.0102],
        [0.0418, 0.0484, -0.0000, -0.0000, 0.0000, 2.0102],
        [0.0027, 0.0027, 0.5000, 0.5000, -0.0000, 0.0000],
        [0.0000, 0.0000, -0.0071, 0.0071, 0.0001, 0.0002], ]

    C_d = [
        [0.0, 0.0, 1.0000, -70.5000],
        [0.0, 0.0, 1.0000, 70.5000],
        [0.0, 0.0, 0.0, 1.0000],
        [-0.2482, 0.2482, 0.0, 0.0], ]

    D_D = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], ]

    # Estimated state vector xhat = [OmegaL, OmegaR, S, psi]
    xhat = (0.0, 0.0, 0.0, 0.0)
    # Integrated position (mm)
    x_mm = 0.0
    y_mm = 0.0

    while True:

        if state7 == 0:
            try:
                uos.stat('calibration.txt')  # imports the calibration text in theory
                with open('calibration.txt', 'r') as f:
                    txt = f.read()
                for ch in ',;\n\t':
                    txt = txt.replace(ch, ' ')
                toks = [t for t in txt.split(' ') if t]
                vals = [int(t, 0) for t in toks]
                cal = bytearray(vals)
                imu.write_cal_coef(cal)
                yield state7
            except OSError:
                cal = None
                state7 = 0  # jumps to state 0
            state7 = 1
            yield state7
        # ----------------------------------------------------------------------------------------
        elif state7 == 1:  # idle only begin state estimation once run begins
            # Reset estimates when robot isn't running
            if int(run_flag.get()) == 0:
                xhat = (0.0, 0.0, 0.0, 0.0)
                x_mm = 0.0;
                y_mm = 0.0
                Xhat1.put(0.0);
                Xhat2.put(0.0);
                Xhat3.put(0.0);
                Xhat4.put(0.0);
                Xpos.put(0.0);
                Ypos.put(0.0);
                yield state7
                continue
            else:
                state7 = 2
                yield state7
        # -----------------------------------------------------------------------------------------
        elif state7 == 2:
            # create U star varaibale which is u and Y concatinated
            try:
                uL = L_Effort.get() * VBAT_VOLTS / 100
                uR = R_Effort.get() * VBAT_VOLTS / 100
            except Exception:
                uL = 0.0;
                uR = 0.0;

            # Encoder wheel speeds -> rad/s -> to mm/s
            try:
                sL = float(PLshare.get()) * rad_per_tick * Wheel_radius_mm
                sR = float(PRshare.get()) * rad_per_tick * Wheel_radius_mm
            except Exception:
                sL = 0.0;
                sR = 0.0;

            # IMU yaw (rad) and yaw rate (rad/s)
            try:
                psi = imu.read_yaw()
                psi_dot = imu.read_yaw_rate()
            except Exception:
                psi = 0.0;
                psi_dot = 0.0

            ustar = (uL, uR, sL, sR, psi, psi_dot)

            # --------- Observer update ---------
            # calculate X_hat nad Y_hat
            Ax = mul4x4_vec4(A_D, xhat)
            yhat = mul4x4_vec4(C_d, xhat)
            Bu = mul4x6_vec6(B_d, ustar)
            xhat = add4(Ax, Bu)

            # Optionally compute predicted outputs (for debugging)
            # y_hat = mul4x4_vec4(C_d, xhat)

            # Publish estimates
            Xhat1.put(xhat[0]);  # omega l velocity
            Xhat2.put(xhat[1]);  # omega R velocity
            Xhat3.put(xhat[2]);  # arc length
            Xhat4.put(xhat[3])  # heading

            # place Yhats into shares
            Yhat1.put(yhat[0]);  # dis omega L, should be in ticks to compare with encoder
            Yhat2.put(yhat[1]);  # dis omega L, should be in ticks to compare with encoder
            Yhat3.put(yhat[2]);  # heading
            Yhat4.put(yhat[3]);  # seta dot

            # --------- Dead-reckon pose (mm) ---------
            # If state3 is vehicle forward speed S (m/s or mm/s?), we assume mm/s here.
            # If S is in rad/s, you can switch to using wheel speeds instead:

            # v_mm_s = 0.5 * ((VLshare.get()) + (VRshare.get())) * mm_per_tick  # at instant
            # dx = v_mm_s * TS * cos(xhat[3])
            # dy = v_mm_s * TS * sin(xhat[3])

            wL = float(Xhat1.get())  # rad/s
            wR = float(Xhat2.get())  # rad/s

            v_mm_s = 0.5 * (wL + wR) * Wheel_radius_mm  # mm/s

            dx = v_mm_s * cos(xhat[3])
            dy = v_mm_s * sin(xhat[3])

            Xpos.put(Xpos.get() + dx)
            Ypos.put(Ypos.get() + dy)

            yield state7


#
# def menial(Shares):  # task 8
#     pass


# for when we convert from CPR to rad/sec
"""
CPR   = 1632              # example
H     = 6.28318530718/CPR # rad/s per cps

Lsp = float(L_setpoint.get()) * H
Rsp = float(R_setpoint.get()) * H
vL  = float(VLshare.get())    * H
vR  = float(VRshare.get())    * H

kp_rad = kp_cps / H
ki_rad = ki_cps / H"""

# NOTE ORDER MATTERS, FIRST: DEF/GENERATORS, SECOND: SHARE, LAST: TASK
# now going to define a share and task
# -------------------------------------------------------------------------------
# Share variable flags
run_flag = task_share.Share('h', thread_protect=False, name="run_flag")
L_Effort = task_share.Share('h', thread_protect=False, name="Left Effort")
R_Effort = task_share.Share('h', thread_protect=False, name="Right Effort")  # (NEW)
Done = task_share.Share('h', thread_protect=False, name="Done")
L_setpoint = task_share.Share('f', thread_protect=False, name="L_setpoint")
R_setpoint = task_share.Share('f', thread_protect=False, name="R_setpoint")

KP_L = task_share.Share('f', thread_protect=False, name="KP_L")
KP_R = task_share.Share('f', thread_protect=False, name="KP_R")
KI_L = task_share.Share('f', thread_protect=False, name="KI_L")
KI_R = task_share.Share('f', thread_protect=False, name="KI_R")
KP_STEER = task_share.Share('f', thread_protect=False, name="KP_Light")
KI_STEER = task_share.Share('f', thread_protect=False, name="KI_Light")

white_cal = task_share.Share('f', thread_protect=False, name="white_cal")
black_cal = task_share.Share('f', thread_protect=False, name="black_cal")
KP_Light = task_share.Share('f', thread_protect=False, name="KP_Light")
KI_Light = task_share.Share('f', thread_protect=False, name="KI_Light")
Line_dir = task_share.Share('f', thread_protect=False, name="Line_dir")  # dimensionless number -1 to 1 / heading
LINE_MODE = task_share.Share('h', thread_protect=False, name="LineMode")
BASE_EFFORT = task_share.Share('h', thread_protect=False, name="BaseEffort")

c = task_share.Share('b', thread_protect=False,
                     name="Centroid")  # 'b' because centroid should only range from -5.5 to 5.5

PATH_STATE = task_share.Share('h', thread_protect=False, name="PathState")
PATH_STATE.put(0)

Xhat1 = task_share.Share('f', thread_protect=False, name="OmegaL_hat")
Xhat2 = task_share.Share('f', thread_protect=False, name="OmegaR_hat")
Xhat3 = task_share.Share('f', thread_protect=False, name="S_hat")
Xhat4 = task_share.Share('f', thread_protect=False, name="psi_hat")

Yhat1 = task_share.Share('f', thread_protect=False, name="S_hat_L")
Yhat2 = task_share.Share('f', thread_protect=False, name="S_hat_R")
Yhat3 = task_share.Share('f', thread_protect=False, name="S_hat_y")
Yhat4 = task_share.Share('f', thread_protect=False, name="psi_hat_y")

Xpos = task_share.Share('f', thread_protect=False, name="X_mm")
Ypos = task_share.Share('f', thread_protect=False, name="Y_mm")

Xpos.put(0);
Ypos.put(0);

# Initialize (pick anything—these are just defaults you can change from the UI)
KP_L.put(0.01);
KP_R.put(0.01)
KI_L.put(0.4);
KI_R.put(0.4);
KP_STEER.put(350);  # for 500 set kp to 50, for 1000 set kp to 100
KI_STEER.put(50);

# made setpoint varaible to be read later


# Initialize share variable flags
run_flag.put(0)
L_Effort.put(0)
R_Effort.put(0)
Done.put(0)
L_setpoint.put(0)
R_setpoint.put(0)

# ---------..................-------------------................----------------
# SHARES: Left side -> TL = time left; PL = position left; VL = velocity left
TLshare = task_share.Share('I', thread_protect=False, name="L_Time_Share")
PLshare = task_share.Share('H', thread_protect=False, name="L_Pos_Share")
VLshare = task_share.Share('f', thread_protect=False, name="L_Vel_Share")  # got ERRORS converting to F to see if fixes
# SHARES: Right side -> TR = time right; PR = position right; VR = velocity right
TRshare = task_share.Share('I', thread_protect=False, name="R_Time_Share")
PRshare = task_share.Share('H', thread_protect=False, name="R_Pos_Share")
VRshare = task_share.Share('f', thread_protect=False, name="R_Vel_Share")
# -----------------------------------------------------------------------------
# QUEUES: Two trio of queues, one for each data type of data, for left and right
# -> changed number of data points to 1500 (3s worth of data at 500Hz)
# TLqueue = task_share.Queue('I', 150, thread_protect=False, overwrite=False, name="L_Time_Queue")
# PLqueue = task_share.Queue('H', 150, thread_protect=False, overwrite=False, name="L_Pos_Queue")
# VLqueue = task_share.Queue('f', 150, thread_protect=False, overwrite=False, name="L_Vel_Queue")
# TRqueue = task_share.Queue('I', 150, thread_protect=False, overwrite=False, name="R_Time_Queue")
# PRqueue = task_share.Queue('H', 150, thread_protect=False, overwrite=False, name="R_Pos_Queue")
# VRqueue = task_share.Queue('f', 150, thread_protect=False, overwrite=False, name="R_Vel_Queue")

# was: 150 and overwrite=False
TLqueue = task_share.Queue('I', 50, thread_protect=False, overwrite=True, name="L_Time_Queue")
PLqueue = task_share.Queue('H', 50, thread_protect=False, overwrite=True, name="L_Pos_Queue")
VLqueue = task_share.Queue('f', 50, thread_protect=False, overwrite=True, name="L_Vel_Queue")
TRqueue = task_share.Queue('I', 50, thread_protect=False, overwrite=True, name="R_Time_Queue")
PRqueue = task_share.Queue('H', 50, thread_protect=False, overwrite=True, name="R_Pos_Queue")
VRqueue = task_share.Queue('f', 50, thread_protect=False, overwrite=True, name="R_Vel_Queue")
# queues for calibrating IR sensors (because it has 6 values, one for each sensor)
black_readings = task_share.Queue('h', 6, thread_protect=False, overwrite=True, name="black_readings")
white_readings = task_share.Queue('h', 6, thread_protect=False, overwrite=True, name="white_readings")


# ----------------------------------------------------------------------------------

# -----------------------------------------------------------------------------------
# Interrupt section
def _kill(_):
    # do the real work outside the ISR
    try:
        L_setpoint.put(0);
        R_setpoint.put(0)
        run_flag.put(0);
        Done.put(1)

    except:
        pass


# active-low switch: internal pull-up + falling-edge interrupt
bump_int1 = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_UP, lambda line: micropython.schedule(_kill, 0))

# -------------------------------------------------------------------------------------

# TASKS: Have five tasks, labeled tasks 1-5.
# period = miliseconds

task1 = cotask.Task(data_col.data_collec_gen, name="Task_1", priority=1, period=10, profile=True, trace=False,
                    shares=(TLshare, PLshare, VLshare, TRshare, PRshare, VRshare,
                            TLqueue, PLqueue, VLqueue, TRqueue, PRqueue, VRqueue, run_flag, Done))

task2 = cotask.Task(left_ME_gen.left_MOEN_gen, name="Task_2", priority=2, period=10, profile=True, trace=False,
                    shares=(TLshare, PLshare, VLshare, run_flag, L_Effort, Done))

task3 = cotask.Task(right_ME_gen.right_MOEN_gen, name="Task_3", priority=2, period=10, profile=True, trace=False,
                    shares=(TRshare, PRshare, VRshare, run_flag, R_Effort, Done))

task4 = cotask.Task(UI_gen.User_Inter_gen, name="Task_4", priority=2, period=30, profile=True, trace=False,
                    # period use to be 20 testing?
                    shares=(TLqueue, PLqueue, VLqueue, TRqueue, PRqueue, VRqueue, run_flag,
                            Done, L_setpoint, R_setpoint, KP_L, KP_R, KI_L, KI_R, KP_STEER,
                            KI_STEER, black_readings, white_readings, BASE_EFFORT, Xpos, Ypos, Xhat4, Xhat3, VLshare,
                            VRshare))  # PERIOD = NONE??

task5 = cotask.Task(PI_con.PI_controller_gen, name="Task_5", priority=2, period=10, profile=True, trace=False,
                    shares=(run_flag, Done, L_setpoint, R_setpoint, L_Effort, R_Effort, VLshare, VRshare, KP_L, KP_R,
                            KI_L, KI_R))

task6 = cotask.Task(Light.Light_PI, name="Task_6", priority=3, period=10, profile=True, trace=False,
                    shares=(run_flag, Done, L_setpoint, R_setpoint, Line_dir, BASE_EFFORT, c, KP_STEER, KI_STEER,
                            black_readings, white_readings, PATH_STATE))

task7 = cotask.Task(IMU, name="Task_7", priority=3, period=50, profile=True, trace=False,
                    shares=(run_flag, L_Effort, R_Effort, VLshare, VRshare, Xhat1, Xhat2, Xhat3, Xhat4, Xpos, Ypos,
                            Yhat1, Yhat2, Yhat3, Yhat4))

task8 = cotask.Task(Track.Track, name="Task_8", priority=3, period=100, profile=True, trace=False,
                    shares=(run_flag, L_Effort, R_Effort, VLshare, VRshare, Xhat1, Xhat2, Xhat3, Xhat4, Xpos, Ypos,
                            Yhat1, Yhat2, Yhat3, Yhat4, PATH_STATE))

cotask.task_list.append(task1)
cotask.task_list.append(task2)
cotask.task_list.append(task3)
cotask.task_list.append(task4)
cotask.task_list.append(task5)  # (NEW)
cotask.task_list.append(task6)  # (New)
cotask.task_list.append(task7)
cotask.task_list.append(task8)  # (NEW)

gc.collect()
# when done is triggered we might need to clear the queues for a future run TBD
# Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed


while True:

    try:

        cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        run_flag.put(0)
        L_Effort.put(0)
        R_Effort.put(0)

        break
    except:
        L_Effort.put(0)
        R_Effort.put(0)
        raise

# Print a table of task data and a table of shared information data
print('\n' + str(cotask.task_list))
print(task_share.show_all())
print(task1.get_trace())
print('')

# takes 8 sec to run
#
# smaple lower, around 100hz or

