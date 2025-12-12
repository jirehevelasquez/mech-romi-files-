import gc
from pyb import Pin, Timer, ADC, UART, repl_uart, ExtInt
import time
import motor
import encoder

def left_MOEN_gen(Shares):  # Task 2 creating gen for left motor task

    state2 = 0  # use try
    # count2 = 0 # on example it is used to track # of runs through this task
    my_TLshare, my_PLshare, my_VLshare, run_flag, L_Effort, Done = Shares

    while True:
        if state2 == 0:  # Charlie said no initialization stages??

            left_enc = encoder.Encoder(4, Pin.cpu.B6, Pin.cpu.B7, period=0xFFFF)
            left_motor = motor.Motor(Pin.cpu.C6, Pin.cpu.B8, Pin.cpu.B9, timer_id=3, timer_ch=1)
            left_enc.zero()
            state2 = 1
            yield state2  # added to speed up timing


        elif state2 == 1:
            state2 = 2 if run_flag.get() else 1
            left_enc.zero()
            yield state2

        elif state2 == 2:  # Start up the left motor and get reference point for time
            left_motor.enable()
            Ltstart = time.ticks_ms()
            state2 = 3
            yield state2

        elif state2 == 3:  # Gather encoder data and time data, put them in PLshare, VLshare, and TLshare, respectively
            left_motor.set_effort(L_Effort.get())
            left_enc.update()

            my_PLshare.put(left_enc.get_position())
            my_VLshare.put(left_enc.get_velocity())

            Lt = time.ticks_ms() - Ltstart
            my_TLshare.put(Lt)

            #         if Lt > 4000 or not run_flag.get():  # stops motor after 4 ish seconds to see
            #             left_motor.disable()
            #             Done.put(1)
            #             run_flag.put(0)
            #             state2 = 1   # back to waiting
            # # else: stay in state 3 and keep sampling
            # run stoped at 929 entrys becuase time hit 4000 limit, dont know why/.....

            if not run_flag.get():  # checks if not run so if not true
                left_motor.disable()
                Done.put(1)
                state2 = 1  # sends back to beginning

        # elif state2 == 4:
        #     left_motor.disable()

        yield state2


