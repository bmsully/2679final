#!usr/bin/env python3

import time
import observe
import orient
import decide
import act

# Primary raspberry pi code for driving OODA loop

# consider arduino-cli for controlling the uno from here too if I need different sensing controls

if __name__ == "__main__":
    while True: 
        start = time.time()
        print(observe.observe())
        print(orient.orient())
        print(decide.decide())
        print(act.act())
        keep_going = input("continue? (y/n) ")
        if keep_going == "n":
            break
        else: 
            end = time.time()
            print(f"time: {end-start}")
            continue

