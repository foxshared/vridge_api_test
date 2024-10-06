from time import sleep
import time

from construct import *
from construct import Int32ul, Float32l, Int16sl

import numpy as np


from dualsense_controller import DualSenseController, UpdateLevel

from madgwickahrs import MadgwickAHRS

from math import *
import math

# Requires:
# https://github.com/KieranWynn/pyquaternion
from pyquaternion import Quaternion


# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

# flag, which keeps program alive
is_running = True

# create an instance, use fiŕst available device
controller = DualSenseController(
    gyroscope_threshold=0.0001,
    accelerometer_threshold=0.0001,
    orientation_threshold=0.001,

    # update_level=UpdateLevel.PAINSTAKING,
    # update_level=UpdateLevel.HAENGBLIEM,
    update_level=UpdateLevel.DEFAULT,

    microphone_initially_muted=False,
    microphone_invert_led=False,
)
# enable/connect the device
controller.activate()

# switches the keep alive flag, which stops the below loop


def stop():
    global is_running
    is_running = False


def on_gyroscope_change(gyroscope):
    global gx, gy, gz
    gx, gy, gz = gyroscope.x, gyroscope.y, gyroscope.z
    pass


def on_accelerometer_change(accelerometer):
    global ax, ay, az
    ax, ay, az = accelerometer.x, accelerometer.y, accelerometer.z
    pass


def on_orientation_change(orientation):
    global x
    # print(f'on_orientation_change: {orientation}')
    x = orientation.roll


controller.gyroscope.on_change(on_gyroscope_change)
controller.accelerometer.on_change(on_accelerometer_change)
# controller.orientation.on_change(on_orientation_change)


controller.lightbar.set_color(88, 10, 200)

# controller.left_trigger.effect.no_resistance()

# on_gyroscope_change: Gyroscope(x=8, y=7, z=0)
# on_accelerometer_change: Accelerometer(x=-25, y=8052, z=1228)
gx, gy, gz = 0, 0, 0
ax, ay, az = 0, 0, 0

sensor = Struct(
   "timestamp" / Int32ul,      # only 24bits used, then rolls, usec
   "gyro" / Array(3, Int16sl),  # +/-250 degree/second allegdely
   "acc" / Array(3, Int16sl),  # only 12bits used, >> 4
)

ACC_FACTOR = np.array((1, 1, -1))
GYRO_FACTOR = np.array((1, 1, -1)) * ((1998.0/(1 << 15)) * (np.pi/180.0))

time2 = 0
data_timestamp = 0
next_output_time = 0
gcomp = np.array((0, 0, 0))
comptime = 0

while True:
    gyro = [gx, gy, gz]
    accel = [ax, ay, az]
    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[0, 0, 0]))
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

    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[0, 0, 0]))
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


# -------------------------------

# start keep alive loop, controller inputs and callbacks are handled in a second thread
# -------------------------------
# Initiate AHRS

refq = Quaternion(1, 0, 0, 0)  # look front
# refq = Quaternion(axis=[0,0,1],angle=np.pi/2) # look right
# refq = Quaternion(axis=[1,0,0],angle=np.pi/2) # look front, roll 90' left-down
# refq = Quaternion(axis=[0,1,0],angle=np.pi/2) # look up

ahrs = MadgwickAHRS(sampleperiod=0.0005, beta=0.1, quaternion=refq)
# heading = MadgwickAHRS()
while is_running:
    # sleep(0.1)
    # # print(gx, gy, gz,ax,ay,az)
    # gyro = [gx, gy, gz]
    # accel = [ax, ay, az]
    # heading.update_imu(gyro, accel)
    # ahrs = heading.quaternion.yaw_pitch_roll
    # roll = ahrs[0]
    # pitch = ahrs[1]
    # yaw = ahrs[2]
    # print(ahrs)


    gyro = [gx, gy, gz]
    accel = [ax, ay, az]


    data_timestamp = time.time()
    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[ax, ay, az]))
    frame = sensor.parse(d)
    time1  = frame["timestamp"]
    if time1 < time2:
        time2 = time2 - (1 << 24)
    if time2:
        delta = (time1 - time2) / 1000000
    else:
        # Assume 500us for first sample
        delta = 500 / 1000000

    # Compute headset rotation - first data block
    ahrs.update_imu((np.array(frame["gyro"])-gcomp)*GYRO_FACTOR, \
            (np.array(frame["acc"])>>4)*ACC_FACTOR , delta)
    
    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[ax, ay, az]))
    frame = sensor.parse(d)
    time2  = frame["timestamp"]
    if time2 < time1:
        time1 = time1 - (1 << 24)
    delta = (time2 - time1) / 1000000

    # Compute headset rotation - second data block
    ahrs.update_imu((np.array(frame["gyro"])-gcomp)*GYRO_FACTOR, \
            (np.array(frame["acc"])>>4)*ACC_FACTOR , delta)
    
    ahrs1 = ahrs.quaternion.yaw_pitch_roll
    roll = ahrs1[0]
    pitch = ahrs1[1]
    yaw = ahrs1[2]
    print(ahrs1)


# disable/disconnect controller device

controller.deactivate()

# =====================================================
