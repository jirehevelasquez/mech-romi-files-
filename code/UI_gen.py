import pyb, micropython, time
from pyb import I2C
from I2C_ob import i2c_object
from I2C_ob import i2c_object
from pyb import UART, repl_uart
import SenArray

repl_uart(None)  # ensure REPL is on USB, not on a UART
u = pyb.USB_VCP()

micropython.alloc_emergency_exception_buf(256)  # helps degugging in putty

# Initialize pins to use in User_Inter_gen and Light_PI
pins = ["PC3", "PB1", "PA4", "PA5", "PA6"]
sensor_array = SenArray.SensorArray(pins)
black_readings_list = []
white_readings_list = []
i2c2 = I2C(2, I2C.MASTER, baudrate=400000)
imu = i2c_object(i2c2)


def User_Inter_gen(Shares):  # task 4
    """
    Shares (in order):
      (TLq, PLq, VLq, TRq, PRq, VRq, run_flag, Done,
       L_setpoint, R_setpoint, KP_L, KP_R, KI_L, KI_R,
       KP_STEER, KI_STEER)
    """
    import pyb, time
    global sensor_array
    (TLq, PLq, VLq, TRq, PRq, VRq,
     run_flag, Done, L_setpoint, R_setpoint, KP_L, KP_R, KI_L, KI_R,
     KP_STEER, KI_STEER, black_readings, white_readings, BASE_EFFORT, Xpos, Ypos, Xhat4, Xhat3, VLshare,
     VRshare) = Shares

    # ---- I/O over the global UART 'u' you set up in main.py ----
    bt = u
    buf = b""

    # ---- pacing knobs (tune if needed) ----
    AUTO_DUMP_MS = 1000
    LINE_UDELAY_US = 7000  # between data lines in 'dump'
    MENU_UDELAY_US = 3000  # between menu lines
    STATUS_UDELAY_US = 2500  # between status lines (show/cal)
    MSG_UDELAY_US = 1500  # after short one-off messages
    PROMPT_UDELAY_US = 1200  # before showing ">"
    auto_tstart = 0

    # ---------- writers (with throttling) ----------
    def bt_write(b: bytes):
        mv = memoryview(b)
        while mv:
            n = bt.write(mv)
            if not n:
                pyb.delay(2)  # yield until TX has room
                continue
            mv = mv[n:]

    def bt_writeline(s: str):
        bt_write(s.encode() + b"\r\n")

    def line_menu(s: str):
        bt_writeline(s);
        pyb.udelay(MENU_UDELAY_US)

    def line_status(s: str):
        bt_writeline(s);
        pyb.udelay(STATUS_UDELAY_US)

    def line_msg(s: str):
        bt_writeline(s);
        pyb.udelay(MSG_UDELAY_US)

    def prompt():
        bt_write(b"> ");
        pyb.udelay(PROMPT_UDELAY_US)

    # ---------- reader ----------
    def try_readline():
        nonlocal buf
        n = bt.any()
        if n:
            incoming = bt.read(min(128, n))
            if incoming:
                cleaned = incoming.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
                buf += bytes(cleaned)
        if b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            return line.decode("utf-8", "ignore").strip()
        return None

    # ---------- queues ----------
    def clear_q(q):
        while q.any():
            try:
                q.get()
            except:
                break

    def clear_all_queues():
        clear_q(TLq);
        clear_q(PLq);
        clear_q(VLq)
        clear_q(TRq);
        clear_q(PRq);
        clear_q(VRq)

    def list_to_queue(values, q, clear=True):  # writes values from a list into a queue
        if clear:
            while q.any():
                try:
                    q.get()
                except:
                    break
        n = 0
        for v in values:
            try:
                q.put(v)
                n += 1
            except:
                break
        return n

    def print_and_clear():
        try:
            n = min(TLq.num_in(), PLq.num_in(), VLq.num_in(),
                    TRq.num_in(), PRq.num_in(), VRq.num_in())
        except:
            n = min(TLq.any(), PLq.any(), VLq.any(), TRq.any(), PRq.any(), VRq.any())
        if n == 0:
            line_status("");
            line_status("(no samples)");
            line_status("")
            return
        line_status("")
        line_status("Samples (idx, tL, vL, tR, vR):")
        for i in range(n):
            tL = TLq.get();
            _pL = PLq.get();
            vL = VLq.get()
            tR = TRq.get();
            _pR = PRq.get();
            vR = VRq.get()
            bt_write(("[%4d] %d  %.3f  %d  %.3f\r\n" % (i, tL, vL, tR, vR)).encode())
            pyb.udelay(LINE_UDELAY_US)
            if (i & 0x3F) == 0:  # every 64 lines, give extra breathing room
                pyb.delay(1)
        line_status("(end samples)")
        line_status("")

    # ---------- helpers ----------
    def parse_setpoints(s: str):
        s = s.replace(",", " ").replace("\t", " ")
        toks = [t for t in s.split() if t]
        L = R = None;
        i = 0
        while i < len(toks):
            t = toks[i].lower()
            if "=" in t:
                k, v = t.split("=", 1)
                try:
                    val = float(v)
                except:
                    return None
                if k in ("l", "left"):
                    L = val
                elif k in ("r", "right"):
                    R = val
                else:
                    return None
                i += 1;
                continue
            if t in ("l", "left"):
                i += 1
                if i >= len(toks): return None
                try:
                    L = float(toks[i])
                except:
                    return None
                i += 1;
                continue
            if t in ("r", "right"):
                i += 1
                if i >= len(toks): return None
                try:
                    R = float(toks[i])
                except:
                    return None
                i += 1;
                continue
            if L is None:
                try:
                    L = float(t)
                except:
                    return None
            elif R is None:
                try:
                    R = float(t)
                except:
                    return None
            else:
                return None
            i += 1
        return (L, R) if (L is not None and R is not None) else None

    def clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def parse_steer(line: str):
        toks = [t for t in line.replace(",", " ").split() if t]
        kp = ki = None
        for t in toks:
            tl = t.lower()
            if tl.startswith("kp="):
                try:
                    kp = float(t.split("=", 1)[1])
                except:
                    return None
            elif tl.startswith("ki="):
                try:
                    ki = float(t.split("=", 1)[1])
                except:
                    return None
        if kp is not None or ki is not None:
            return kp, ki
        nums = []
        for t in toks:
            try:
                nums.append(float(t))
            except:
                pass
        if len(nums) >= 2:
            return nums[0], nums[1]
        return None

    def show_status():
        line_status("")
        try:
            line_status("  BASE_EFFORT=%d cps" % int(BASE_EFFORT.get()))
        except:
            line_status("  BASE_EFFORT=<unset>")
        line_status("  Motor PI gains:  KP=%.5f  KI=%.5f" % (KP_L.get(), KI_L.get()))
        line_status("  STEER gains   :  KP_STEER=%.5f  KI_STEER=%.5f" %
                    (KP_STEER.get(), KI_STEER.get()))
        line_status("  L_sp=%.3f cps  R_sp=%.3f cps" % (L_setpoint.get(), R_setpoint.get()))
        line_status("")

    def show_calibration():
        try:
            line_status("black: " + " ".join(str(v) for v in black_readings_list))
            line_status("white: " + " ".join(str(v) for v in white_readings_list))
        except Exception as e:
            line_status("calibration values not available (%s)" % e)
        prompt()

    # -------------------------------------------------------------------
    def read_imu_cal_stat():
        sysc, gyr, acc, mag = imu.cal_status()
        line_status("calibration status:")
        bt_write(b"gyr=%d\r\n" % int(gyr))
        bt_write(b"acc=%d\r\n" % int(acc))
        bt_write(b"mag=%d\r\n" % int(mag))

    def change_to_config_mode():
        imu.config_mode()
        line_status("Successfully changed mode to CONFIG mode")

    def change_to_imu_mode():
        imu.imu_mode()
        line_status("Successfully changed mode to IMU mode")

    def read_imu_cal_coef():
        cal = imu.cal_coef()  # bytearray(22)
        line_status("IMU coefficient values (u8):")
        for i, b in enumerate(cal):  # b is 0..255
            line_status("coef[%02d] = %3d" % (i, b))

    def read_imu_yaw():
        yaw_data_rad = imu.read_yaw()
        line_status("Yaw value:")
        bt_write((f"{yaw_data_rad:.3f}\r\n").encode())

    def read_imu_yaw_rate():
        yaw_rate_data_rad = imu.read_yaw_rate()
        line_status("Yaw rate value:")
        bt_write((f"{yaw_rate_data_rad:.3f}\r\n").encode())

    # --------------------------------------------------------------

    def print_menu():
        lines = [
            "",
            "Commands:",
            "  base <0..6000>     : set forward BASE_EFFORT (starts/stops robot)",
            "  steer kp=<v> ki=<v>: set steering gains (also: 'steer <kp> <ki>')",
            "  kp_steer <v>       : set steering Kp only",
            "  ki_steer <v>       : set steering Ki only",
            "  calrun             : calibration wizard (b -> w -> done)",
            "  cal                : show last captured black/white values",
            "  dump               : print & clear recent samples",
            "  show, help, stop|q",
            "  (optional) raw setpoints, e.g. '3000 3000' for bench test",
            ""
        ]
        for ln in lines:
            line_menu(ln)
        show_status()
        prompt()

    # ---------- state machine ----------
    state4 = 0
    while True:
        if state4 == 0:
            KP_L.put(0.01);
            KP_R.put(0.01)  # motor Ki, KP
            KI_L.put(0.40);
            KI_R.put(0.40)

            if KP_STEER.get() == 0.0:
                KP_STEER.put(100)
            if KI_STEER.get() == 0.0:
                KI_STEER.put(0.10)

            Done.put(0)
            run_flag.put(0)
            L_setpoint.put(0.0)
            R_setpoint.put(0.0)

            try:
                black_readings_list
            except NameError:
                black_readings_list = []
            try:
                white_readings_list
            except NameError:
                white_readings_list = []

            print_menu()
            state4 = 1
            yield state4
        # ------------------------------------------------
        elif state4 == 1:
            s = try_readline()
            if s is None:
                if run_flag.get() and AUTO_DUMP_MS > 0 and auto_tstart:
                    if time.ticks_diff(time.ticks_ms(), auto_tstart) >= AUTO_DUMP_MS:
                        print_and_clear();
                        auto_tstart = 0
                yield state4;
                continue

            low = s.lower().strip()

            if low in ("q", "quit", "exit", "stop"):
                run_flag.put(0);
                Done.put(1)
                L_setpoint.put(0.0);
                R_setpoint.put(0.0)
                try:
                    BASE_EFFORT.put(0)
                except:
                    pass
                line_msg("Stopped.");
                prompt()

            elif low in ("help", "h", "?"):
                print_menu()

            elif low.startswith("show"):
                show_status();
                prompt()

            elif low.startswith("dump"):
                print_and_clear();
                prompt()

            elif low.startswith("base"):
                rest = s[4:].strip()
                try:
                    val = int(float(rest))
                except:
                    line_msg("Usage: base <forward_speed_cps>");
                    prompt()
                else:
                    if val < 0: val = 0
                    if val > 6000: val = 6000
                    try:
                        BASE_EFFORT.put(val)
                    except Exception as e:
                        line_msg("Failed to set BASE_EFFORT (%s)" % e);
                        prompt()
                        continue
                    if val > 0:
                        run_flag.put(1);
                        Done.put(0)
                    else:
                        run_flag.put(0);
                        Done.put(1)
                        L_setpoint.put(0.0);
                        R_setpoint.put(0.0)
                    line_msg("BASE_EFFORT set to %d" % val);
                    prompt()

            elif low.startswith("steer"):
                parsed = parse_steer(s[5:].strip())
                if parsed is None:
                    line_msg("Usage: steer kp=<v> ki=<v>  OR  steer <kp> <ki>")
                    line_msg("Current: KP_STEER=%.5f  KI_STEER=%.5f" %
                             (KP_STEER.get(), KI_STEER.get()))
                    prompt()
                else:
                    kp, ki = parsed
                    if kp is not None:
                        KP_STEER.put(clamp(float(kp), 0.0, 100000))
                    if ki is not None:
                        KI_STEER.put(clamp(float(ki), 0.0, 100000))
                    line_msg("STEER gains -> KP_STEER=%.5f  KI_STEER=%.5f" %
                             (KP_STEER.get(), KI_STEER.get()))
                    prompt()

            elif low.startswith("kp_steer"):
                rest = s.split(None, 1)[1] if len(s.split()) > 1 else ""
                try:
                    v = clamp(float(rest), 0.0, 3000)  # do not clamp less than 300 becuase kp is 100
                    KP_STEER.put(v)
                    line_msg("KP_STEER=%.5f" % KP_STEER.get())
                except:
                    line_msg("Usage: kp_steer <value>")
                prompt()

            elif low.startswith("ki_steer"):
                rest = s.split(None, 1)[1] if len(s.split()) > 1 else ""
                try:
                    v = clamp(float(rest), 0.0, 3000)
                    KI_STEER.put(v)
                    line_msg("KI_STEER=%.5f" % KI_STEER.get())
                except:
                    line_msg("Usage: ki_steer <value>")
                prompt()

            elif low.startswith("calrun"):
                line_menu("")
                line_menu("Calibration wizard:")
                line_menu("  Place on BLACK and type: b")
                line_menu("  Then place on WHITE and type: w")
                line_menu("  Type: done   (to apply)")
                line_menu("")
                prompt()
                cal_state = 0
                while True:
                    line = try_readline()
                    if line is None:
                        yield;
                        continue
                    l = line.lower().strip()
                    if l == "b":
                        try:
                            black_readings_list = list(sensor_array.read_all())
                            line_status("Captured BLACK: " + " ".join(str(v) for v in black_readings_list))
                            line_status("Now place on WHITE and type 'w'.")
                            list_to_queue(black_readings_list, black_readings, clear=True)
                            cal_state = 1
                        except Exception as e:
                            line_msg("Error reading BLACK: %s" % e)
                        prompt()
                    elif l == "w" and cal_state >= 1:
                        try:
                            white_readings_list = list(sensor_array.read_all())
                            line_status("Captured WHITE: " + " ".join(str(v) for v in white_readings_list))
                            line_status("Type 'done' to apply.")
                            list_to_queue(white_readings_list, white_readings, clear=True)
                            cal_state = 2
                        except Exception as e:
                            line_msg("Error reading WHITE: %s" % e)
                        prompt()
                    elif l == "done" and cal_state == 2:
                        try:
                            sensor_array.calibrate_all_black(black_readings_list)
                            sensor_array.calibrate_all_white(white_readings_list)
                            line_msg("Applied calibration to sensor_array.")
                        except Exception as e:
                            line_msg("Calibration apply failed: %s" % e)
                        show_calibration()
                        break
                    else:
                        line_msg("Type b (black), w (white), or done.")
                        prompt()

            elif low.startswith("cal"):
                show_calibration()
            # --------------------------------------------------------------------------------
            elif low.startswith("imu_cal_stat"):
                read_imu_cal_stat()
                prompt()

            elif low.startswith("config_mode"):
                change_to_config_mode()
                prompt()

            elif low.startswith("imu_mode"):
                change_to_imu_mode()
                prompt()

            elif low.startswith("imu_cal_coef"):
                read_imu_cal_coef()
                prompt()

            elif low.startswith("yaw"):
                read_imu_yaw()
                prompt()

            elif low.startswith("yrate"):
                read_imu_yaw_rate()
                prompt()

            # ------------------------------------------------------------------------------
            elif low.startswith("xdis"):
                line_status("X=%.1f  S_hat=%.3f" % (Xpos.get(), Xhat3.get()))
                prompt()
            elif low.startswith("ydis"):
                line_status("Y=%.1f" % (Ypos.get()))
                prompt()
            elif low.startswith("position"):
                line_status("Xpos=%.1f  Ypos=%.1f  Heading_aprox=%.1f  Dis_aprox=%.1f "
                            % (Xpos.get(), Ypos.get(), Xhat4.get(), Xhat3.get()))
                line_status("true omegaL=%.1f  true omegaR=%.1f  " % (VLshare.get(), VRshare.get()))
                prompt()

            # --------------------------------------------------------------------------------

            else:
                lr = parse_setpoints(s)
                if lr is None:
                    line_msg("Bad input. Type 'help' for examples.");
                    prompt()
                else:
                    Lsp, Rsp = lr
                    L_setpoint.put(float(Lsp));
                    R_setpoint.put(float(Rsp))
                    run_flag.put(1);
                    Done.put(0)
                    clear_all_queues()
                    auto_tstart = time.ticks_ms()
                    line_msg("OK: L_sp=%.3f  R_sp=%.3f (cps). Capturing ~%d ms..." %
                             (Lsp, Rsp, AUTO_DUMP_MS))
                    prompt()

        yield state4
