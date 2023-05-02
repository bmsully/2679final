#!usr/bin/env python3

import time
import observe
import orient
import decide
import act

# Primary raspberry pi code for driving OODA loop

"""
Primary function to run StATV robot
Observe: Using camera to locate air vehicle - use an AprilTag and quadcopter or mount to ceiling
Orient: Detect if target in view - if so begin/continue MDP policy, otherwise perform spiral search

"""
def runStATV():
    pass

class StATV():
    def __init__():
        pass

    def checkRep():
        pass

    def run():
        pass

    def runSpiral():
        pass

    def runMDP():
        pass

    def driveMotors():
        pass

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

