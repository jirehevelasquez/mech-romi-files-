import gc
from pyb import Pin, Timer, ADC, UART, repl_uart, ExtInt
import time
import motor
import encoder

def right_MOEN_gen(Shares):  # task 3
    state3 = 0  # use try
    # count3 = 0 # on example it is used to track # of runs through this task
    my_TRshare, my_PRshare, my_VRshare, run_flag, R_Effort, Done = Shares

    while True:
        if state3 == 0:

            right_enc = encoder.Encoder(2, Pin.cpu.A0, Pin.cpu.A1, period=0xFFFF)
            right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.A8, Pin.cpu.C0, timer_id=3,
                                      timer_ch=3)  # dont need P because pin is called
            right_enc.zero()
            state3 = 1
            yield state3
            # must call timer id as timer_id= ect, and pwm_freq = 20000
            # timer_id is timer block
            # timer_ch is which channel
            # version 2

        elif state3 == 1:
            state3 = 2 if run_flag.get() else 1
            right_enc.zero()
            yield state3

        elif state3 == 2:  # Start up the left motor and get reference point for tim3
            right_motor.enable()
            Rtstart = time.ticks_ms()
            state3 = 3
            yield state3

        elif state3 == 3:  # Gather encoder data and time data, put them in PLshare, VLshare, and TLshare, respectively
            right_motor.set_effort(R_Effort.get())
            right_enc.update()
            my_PRshare.put(right_enc.get_position())
            my_VRshare.put(right_enc.get_velocity())
            Rt = time.ticks_ms() - Rtstart
            my_TRshare.put(Rt)

            if not run_flag.get():
                right_motor.disable()
                Done.put(1)
                state3 = 1
            else:
                state3 = 3

        # elif state3 == 4:
        #     right_motor.disable()

        yield state3
