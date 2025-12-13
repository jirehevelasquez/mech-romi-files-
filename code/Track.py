# -*- coding: utf-8 -*-
"""
Created on Mon Nov 24 22:47:05 2025

@author: jireh
"""
"""need to save memeory"""
# import pyb, micropython, time
# from pyb import I2C
# from I2C_ob import i2c_object
# from I2C_ob import i2c_object
# from pyb import UART, repl_uart
# import SenArray
from math import pi
import pyb

# phase = ph = state equivalent

NORMAL_path = 0  # start, follow line normally to the split

SPLIT_RIGHT_path = 1  # near split, bias to right branch -> (~)

NORMAL_path2 = 2  # back to line following to get CP 1, 2, 3

thru_diamond = 3  # go through diamond

NORMAL_path3 = 4  # follow line

tight_turn_path = 5  # go through tight turns

NORMAL_path4 = 6 # line follow

Orient1 = 7 # Reorientate to go through garage

Garage_path = 8  # line disappears, keep driving straight

Turn_right_path = 9  # rotate 90 degrees

Normal_path5 = 10  # line follow out of the garage

wall_path = 11  # around wall attempt


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    while a > pi:
        a -= 2.0 * pi
    while a < -pi:
        a += 2.0 * pi
    return a


def Track(shares):  # task 7
    (run_flag, L_Effort, R_Effort, VLshare, VRshare,
     Xhat1, Xhat2, Xhat3, Xhat4, Xpos, Ypos,
     Yhat1, Yhat2, Yhat3, Yhat4, PATH_STATE) = shares

    state8 = NORMAL_path  # preload to basic line following
    ### WORKS FOR VOLTAGE 1.47 AVERAGE ###
    # 1) where split lives:
    # SPLIT_X, SPLIT_Y = 535.0, 0.0  # coordinates where the split is og worked 535 & 0
    # ENTER_SPLIT_R, EXIT_SPLIT_R = 50.0, 25.0  # how close we are till be trigger and exit trigger
    SPLIT_DIS = 660
    SPLIT_DIS_E = 670

    # STATE2_EXIT_X, STATE2_EXIT_Y = 650.0, -150.0
    # STATE2_EXIT_R = 100.0  # radius (mm) you consider "close enough"
    STATE2_EXIT_DIS = SPLIT_DIS_E + 200

    # STATE3_EXIT_X, STATE3_EXIT_Y = 650.0, -225.0
    # STATE3_EXIT_R = 100.0  # radius (mm) you consider "close enough"
    STATE3_EXIT_DIS = STATE2_EXIT_DIS + 150

    # STATE4_EXIT_X, STATE4_EXIT_Y = 1050, -500.0  # readings 1007 -543 coordinates where turns start
    # STATE4_EXIT_R = 100.0
    STATE4_EXIT_DIS = STATE3_EXIT_DIS + 2080

    # STATE5_EXIT_X, STATE5_EXIT_Y = 750, -500.0  # end of turns location
    # # og vaues 750 -500, measure was 853.22, -503
    # STATE5_EXIT_R = 70.0
    STATE5_EXIT_DIS = STATE4_EXIT_DIS + 350

    # STATE6_EXIT_X, STATE6_EXIT_Y = 400, -500.0  # entrance of garage
    # STATE6_EXIT_R = 50.0
    STATE6_EXIT_DIS = STATE5_EXIT_DIS + 200 #(3526)

    # STATE7_EXIT_X, STATE7_EXIT_Y = 100, -500.0  # garage stright away end
    # STATE7_EXIT_R = 50.0
    STATE7_EXIT_DIS = STATE6_EXIT_DIS + 10

    # STATE8_EXIT_X, STATE8_EXIT_Y = 600, -600.0 # garage turn
    # STATE8_EXIT_R = 50.0
    STATE8_EXIT_DIS = STATE6_EXIT_DIS + 725

    STATE9_EXIT_X, STATE9_EXIT_Y = 100, -100.0  # garage turn
    STATE9_EXIT_R = 50.0

    STATE10_EXIT_DIS = STATE8_EXIT_DIS + 200

    count = 0

    heading_at_wall = None  # will be set when we hit the wall f

    def dist2(x, y, xp, yp):
        dx = x - xp
        dy = y - yp
        return dx * dx + dy * dy

    while True:
        if run_flag.get() == 0:
            PATH_STATE.put(NORMAL_path)
            state8 = NORMAL_path
            yield state8
            continue

        # --- grab position safely ---
        try:
            x = Xpos.get()
            y = Ypos.get()
            psi = (Xhat4.get())  # heading [rad] from IMU/observer
            net_dis = Xhat3.get()
        except Exception:
            x = y = psi = net_dis = 0.0

        # ================== state LOGIC ==================
        # ------- Starting stage: from starting point to split in the path -------

        if state8 == NORMAL_path:  # 0
            PATH_STATE.put(NORMAL_path)
            print("state0", net_dis)
            # when close to the split -> switch to "bias right"
            # condition 1: XY-based (distance to split point)
            # near_split_xy = dist2(x, y, SPLIT_X, SPLIT_Y) < (ENTER_SPLIT_R ** 2)

            # condition 2: odometer-based (net distance traveled)
            # near_split_dis = net_dis > SPLIT_S_ENTER

            if net_dis >= SPLIT_DIS:
                state8 = SPLIT_RIGHT_path
                PATH_STATE.put(SPLIT_RIGHT_path)

        # --------- Split right stage: skews centroid to turn right briefly --------

        elif state8 == SPLIT_RIGHT_path:  # 1
            PATH_STATE.put(SPLIT_RIGHT_path)
            print("state1", net_dis)
            # past_split_dis = net_dis > SPLIT_S_EXIT

            if net_dis >= SPLIT_DIS_E:
                state8 = NORMAL_path2
                PATH_STATE.put(NORMAL_path2)

        # -------- Stage after splitting right: back to line following  --------

        elif state8 == NORMAL_path2:  # 2
            PATH_STATE.put(NORMAL_path2)
            print("state2", net_dis)
            # past_split_dis = net_dis > SPLIT_S_EXIT

            if net_dis >= STATE2_EXIT_DIS:
                state8 = thru_diamond
                PATH_STATE.put(thru_diamond)
        # ---------------------------------------------------------------------

        elif state8 == thru_diamond:  # 3
            PATH_STATE.put(thru_diamond)
            print("state3", net_dis)

            if net_dis >= STATE3_EXIT_DIS:
                state8 = NORMAL_path3
                PATH_STATE.put(NORMAL_path3)

        # -------------------------------------------------------------------
        elif state8 == NORMAL_path3:  # 4
            PATH_STATE.put(NORMAL_path3)
            print("state4", net_dis)

            if net_dis >= STATE4_EXIT_DIS:
                state8 = tight_turn_path
                PATH_STATE.put(tight_turn_path)

        # ---------------------------------------------------------------------------------

        elif state8 == tight_turn_path:  # 5
            PATH_STATE.put(tight_turn_path)
            print("state5", net_dis)

            if net_dis >= STATE5_EXIT_DIS:
                state8 = NORMAL_path4
                PATH_STATE.put(NORMAL_path4)

        # ------------------------------------------------------------------------------

        elif state8 == NORMAL_path4:  # 6
            PATH_STATE.put(NORMAL_path4)
            print("state6", net_dis)

            if net_dis >= STATE6_EXIT_DIS:
                state8 = Orient1
                PATH_STATE.put(Orient1)

        # ----------------------------------------------------------------------------------

        elif state8 == Orient1 :  # 7
            #PATH_STATE.put(Orient1)
            print("state7", net_dis,PATH_STATE.get())

            if PATH_STATE.get() == 8:
                state8 = 8

        # -----------------------------------------------------------------------------------

        elif state8 == Garage_path or PATH_STATE.get() == 8:  # 8
            state8 = Garage_path
            PATH_STATE.put(Garage_path)
            print("state8", net_dis)

            if net_dis >= STATE8_EXIT_DIS:
                state8 = Turn_right_path
                PATH_STATE.put(Turn_right_path)

        # -------------------------------------------------------------------------

        elif state8 == Turn_right_path:  # 9
            #PATH_STATE.put(Turn_right_path)
            print("state9", net_dis,PATH_STATE.get())

            if PATH_STATE.get() == 10:
                state8 = 10

        # ---------------------------------------------------------------------
        elif state8 == Normal_path5 or PATH_STATE.get() == 10:  # 10
            state8 = Normal_path5
            PATH_STATE.put(Normal_path5)
            print("state10", net_dis)

            if net_dis >= STATE10_EXIT_DIS:
                state8 = wall_path
                PATH_STATE.put(wall_path)

        # ----------------------------------------------------------------------
        elif state8 == wall_path:  # --- STATE 11: rotate 90° clockwise in place ---
            PATH_STATE.put(wall_path)  # optional: just for debugging/visibility
            print("state10", x, y)
            # First time in this state: record starting heading
            if heading_at_wall is None:
                heading_at_wall = psi  # psi is your yaw from Xhat[3]/Xhat4

            # Target heading = current heading - 90° (clockwise turn)
            desired = heading_at_wall - (pi / 2.0)

            # Error between target and current, wrapped to [-pi, pi]
            e = wrap_angle(desired - psi)

            if abs(e) > 0.1:  # still need to turn (~ > 6 degrees)
                # spin in place clockwise:
                # left wheel forward, right wheel backward
                L_Effort.put(10.0)
                R_Effort.put(-10.0)
            else:
                # We're close enough to the target heading → stop turning
                L_Effort.put(0.0)
                R_Effort.put(0.0)

                # Clear the latched heading so next time we re-latch
                heading_at_wall = None

                # Move on to STATE 9
                state8 = wall_path
                PATH_STATE.put(wall_path)

            " " " coded up to here "
            # elif state8 == NORMAL_path4:
        #     PATH_STATE.put(NORMAL_path4)

        # First time in this phase: record heading at wall
        # if heading_at_wall is None:
        #     heading_at_wall = psi

        #     desired = heading_at_wall - (pi / 2.0)  # right turn ~ -90°
        #     e = wrap_angle(desired - psi)
        #
        #     if abs(e) > 0.1:  # still need to turn
        #         # in-place right turn
        #         L_Effort.put(2000.0)
        #         R_Effort.put(-2000.0)
        #     else:
        #         # finished turn, clear state and move on
        #         heading_at_wall = None
        #         state8 = PH_AROUND_OBSTACLE
        #
        # elif state8 == PH_AROUND_OBSTACLE:
        #     PATH_STATE.put(PH_AROUND_OBSTACLE)
        #
        #     # TODO: implement your obstacle-around behavior
        #     # e.g., short forward drive, then wait for line reacquisition
        #     # For example:
        #     # if line_sensor_share.get() != 0:
        #     #     state8 = PH_NORMAL
        #     pass

        # Let scheduler run other tasks
        yield state8