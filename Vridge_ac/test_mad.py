from time import sleep

from construct import *
from construct import Int32ul, Float32l, Int16sl

import numpy as np


from dualsense_controller import DualSenseController, UpdateLevel

from madgwickahrs import MadgwickAHRS

from math import *
import math
sensor = Struct(
   "timestamp" / Int32ul,      # only 24bits used, then rolls, usec
   "gyro" / Array(3, Int16sl), # +/-250 degree/second allegdely
   "acc" / Array(3, Int16sl),  # only 12bits used, >> 4
)

ACC_FACTOR = np.array((1, 1, -1))
GYRO_FACTOR = np.array((1, 1, -1)) * ((1998.0/(1<<15)) * (np.pi/180.0))

time2 = 0
data_timestamp = 0
next_output_time = 0
gcomp = np.array((0,0,0))
comptime = 0

while True:
    d = sensor.build(dict(timestamp=0,gyro=[0,0,0],acc=[0,0,0]))
    frame = sensor.parse(d)
    time1 = frame["timestamp"]
    if time1 < time2:
        time2 = time2 - (1 << 24)
    if time2:
        delta = (time1 - time2) / 1000000
    else:
        # Assume 500us for first sample
        delta = 500 / 1000000

    # Sum gyro movements
    gcomp = gcomp + np.array(frame["gyro"])
    comptime = comptime + delta

    d = sensor.build(dict(timestamp=0,gyro=[0,0,0],acc=[0,0,0]))
    frame = sensor.parse(d)

    time2 = frame["timestamp"]
    if time2 < time1:
        time1 = time1 - (1 << 24)
    delta = (time2 - time1) / 1000000

    gcomp = gcomp + np.array(frame["gyro"])
    comptime = comptime + delta

    if comptime > 5:
        gcomp = gcomp * GYRO_FACTOR / comptime
        print("Gcomp:", gcomp)
        break