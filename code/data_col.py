import gc
from pyb import Pin, Timer, ADC, UART, repl_uart, ExtInt
def data_collec_gen(Shares):  # task 1 generator/ sub-states
    """
    Shares:
      (TLshare, PLshare, VLshare, TRshare, PRshare, VRshare,
       TLqueue, PLqueue, VLqueue, TRqueue, PRqueue, VRqueue, run_flag, Done)
    Behavior:
      - Collects samples while run_flag==1
      - If any queue is full, drop one oldest item from ALL queues to keep alignment
      - Never clears run_flag / Done on its own (you control start/stop from UI)
    """
    state1 = 0
    (my_TLshare, my_PLshare, my_VLshare, my_TRshare, my_PRshare, my_VRshare,
     my_TLqueue, my_PLqueue, my_VLqueue, my_TRqueue, my_PRqueue, my_VRqueue,
     run_flag, Done) = Shares

    while True:
        if state1 == 0:
            # Wait for user to start
            if run_flag.get():
                state1 = 1
                yield state1  # added for time sake

        elif state1 == 1:
            # Grab a coherent snapshot
            tL = my_TLshare.get();
            pL = my_PLshare.get();
            vL = my_VLshare.get()
            tR = my_TRshare.get();
            pR = my_PRshare.get();
            vR = my_VRshare.get()

            try:
                # Try fast path first
                my_TLqueue.put(tL);
                my_PLqueue.put(pL);
                my_VLqueue.put(vL)
                my_TRqueue.put(tR);
                my_PRqueue.put(pR);
                my_VRqueue.put(vR)
            except Exception:
                # Queues likely full -> drop one from ALL queues to keep alignment, then retry once
                try:
                    if my_TLqueue.any(): my_TLqueue.get()
                    if my_PLqueue.any(): my_PLqueue.get()
                    if my_VLqueue.any(): my_VLqueue.get()
                    if my_TRqueue.any(): my_TRqueue.get()
                    if my_PRqueue.any(): my_PRqueue.get()
                    if my_VRqueue.any(): my_VRqueue.get()
                except Exception:
                    pass
                # Retry; if it still fails, skip this sample but KEEP RUNNING
                try:
                    my_TLqueue.put(tL);
                    my_PLqueue.put(pL);
                    my_VLqueue.put(vL)
                    my_TRqueue.put(tR);
                    my_PRqueue.put(pR);
                    my_VRqueue.put(vR)
                except Exception:
                    pass

            # Only exit when user stops
            if not run_flag.get():
                state1 = 0
                # Do not touch Done here; UI handles it

        yield state1
