#!/usr/bin/env python3

from time import time, sleep

import math

time_start = time()

def tic():
    global time_start

    time_start = time()

def toc():
    global time_start

    now = time()
    return now - time_start

#===============================================================================
def main():

    tic()
    print("Hello PSR")

    for i in range(0, 500000000):
        ret = math.sqrt(i)

    elapsed = toc()
    print(f"time: {elapsed:.10f}")

if __name__ == '__main__':
    main()
