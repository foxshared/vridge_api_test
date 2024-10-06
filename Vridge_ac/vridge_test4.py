# Vridge api controller control with dualsense
# Head Vridge from phone
# Resualt bad heading for dualsense

from time import sleep

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *
from construct import Int32ul, Float32l, Int16sl


# http://zeromq.org
# ZeroMQ-4.0.4-miru1.0-x86.exe
#
# https://www.nico-maas.de/?p=1081
# https://github.com/zeromq/pyzmq
# pyzmq-16.0.2.zip

import zmq
import time
from math import radians
from pyrr import Matrix44, Vector3
import numpy as np
import math
import struct

from vridge_api_pb2 import HeadTrackingRequest, HeadTrackingResponse, ControllerStateRequest, ControllerStateResponse, VRController

from dualsense_controller import DualSenseController, UpdateLevel

from madgwickahrs import MadgwickAHRS

from math import *

# Requires:
# https://github.com/KieranWynn/pyquaternion
from pyquaternion import Quaternion


# ================================================================

# convert euler_to_quaternion(r) https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr


def euler_to_quaternion(r):

    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


# ================================================================

# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

# create an instance, use fiŕst available device
controller_ps5 = DualSenseController(
    gyroscope_threshold=0.002,
    accelerometer_threshold=0.002,
    orientation_threshold=0.002,

    # update_level=UpdateLevel.PAINSTAKING,
    # update_level=UpdateLevel.HAENGBLIEM,
    update_level=UpdateLevel.DEFAULT,

    microphone_initially_muted=False,
    microphone_invert_led=False,
)

# flag, which keeps program alive
is_running = True


def stop():
    global is_running
    is_running = False


# enable/connect the device
controller_ps5.activate()

controller_ps5.lightbar.set_color(88, 10, 200)

# ================================================================

addr = 'tcp://localhost:38219'
# -------------------------------
# connect to the control channel
print("Connecting to", addr)
ctx = zmq.Context()

master = ctx.socket(zmq.REQ)
master.connect(addr)

master.send_json({"ProtocolVersion": 1, "Code": 2})
answer = master.recv_json()

print(answer)

# ================================================================

master.send_json({"RequestedEndpointName": "HeadTracking",
                 "ProtocolVersion": 3, "Code": 1})
answer = master.recv_json()
print(answer)

if answer['ProtocolVersion'] != 3 or answer['Code'] != 0:
    print("HeadTracking: incompatible protocol, or not available")
    exit(0)

headset_addr = answer['Port']
headset_addr = "tcp://localhost:{}".format(headset_addr)
# headset_addr = answer['EndpointAddress']

print("Found Head")
print("Headset", headset_addr)

# ================================================================

master.send_json({"RequestedEndpointName": "Controller",
                 "ProtocolVersion": 3, "Code": 1})
answer = master.recv_json()
print(answer)

if answer['ProtocolVersion'] != 3 or answer['Code'] != 0:
    print("Controller: incompatible protocol, or not available")
    exit(0)

controller_addr = answer['Port']
controller_addr = "tcp://localhost:{}".format(controller_addr)
# controller_addr = answer['EndpointAddress']

print("Found Controller")
print("Controller", controller_addr)


# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)
headset.connect(headset_addr)

print("connect headset")

# -------------------------------
# Connect to Controller
controller = ctx.socket(zmq.REQ)
controller.connect(controller_addr)

print("connect Controller")


# ================================================================ Headtrack send read data
# Protobuf serialized vridge api
send_track = HeadTrackingRequest()

send_track.Version = 3
send_track.TaskType = 5
data = Struct("data"/Padded(64, Array(3, Float32l)))
data1 = dict(data=[0, 2, 0])

data_r = data.build(data1)
# print(data_r)
send_track.Data = data_r

serialized_track = send_track.SerializeToString()

print(serialized_track)

read_back = HeadTrackingRequest()
read_back.ParseFromString(serialized_track)
print(read_back)

# ================================================================
# Send byte data to vridge api
headset.send(serialized_track)

# Receive and read data from vridge api
answer1 = headset.recv()
print(answer1)
read_answer1 = HeadTrackingResponse()
read_answer1.ParseFromString(answer1)
print(read_answer1)
# ================================================================


def on_orientation_change(orientation):
    global pitch_, roll_
    pitch_ = orientation.pitch
    roll_ = orientation.roll
    pass


def on_gyroscope_change(gyroscope):
    global gx, gy, gz
    gx, gy, gz = gyroscope.x, gyroscope.y, gyroscope.z
    pass


def on_accelerometer_change(accelerometer):
    global ax, ay, az
    ax, ay, az = accelerometer.x, accelerometer.y, accelerometer.z
    pass


def on_cross_btn_pressed():
    print('cross button pressed')
    stop()


flag_ = True


def on_square_btn_pressed():
    print("Calibarte")
    global flag_
    flag_ = True
    pass

# callback, when unintended error occurs,
# i.e. physically disconnecting the controller during operation
# stop program


def on_error(error):
    print(f'Opps! an error occured: {error}')
    stop()


controller_ps5.gyroscope.on_change(on_gyroscope_change)
controller_ps5.accelerometer.on_change(on_accelerometer_change)
controller_ps5.orientation.on_change(on_orientation_change)
controller_ps5.btn_cross.on_down(on_cross_btn_pressed)
controller_ps5.btn_square.on_down(on_square_btn_pressed)

# -------------------------------

pitch_ = 0
roll_ = 0
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
# -------------------------------
# Calculate Gryo compensation (should be static)
while flag_:
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
        flag_ = False
        break
# -------------------------------
# Initiate AHRS

refq = Quaternion(1, 0, 0, 0)  # look front
# refq = Quaternion(axis=[0,0,1],angle=np.pi/2) # look right
# refq = Quaternion(axis=[1,0,0],angle=np.pi/2) # look front, roll 90' left-down
# refq = Quaternion(axis=[0,1,0],angle=np.pi/2) # look up

ahrs = MadgwickAHRS(sampleperiod=0.0005, beta=0.1, quaternion=refq)
# heading = MadgwickAHRS()
# ================================================================ Controller send read data
while is_running or flag_ is False:
    conps = ControllerStateRequest()
    conps.Version = 3
    # t = Padded(2,Byte)
    # t1 = t.build(1)
    # print(t1)
    conps.TaskType = 1
    conps.ControllerState.ControllerId = 2
    conps.ControllerState.Status = 0

    # pitch1 = radians(pitch_)
    # roll1 = radians(roll_)

    gyro = [gx, gy, gz]
    accel = [ax, ay, az]

    data_timestamp = time.time()
    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[ax, ay, az]))
    frame = sensor.parse(d)
    time1 = frame["timestamp"]
    if time1 < time2:
        time2 = time2 - (1 << 24)
    if time2:
        delta = (time1 - time2) / 1000000
    else:
        # Assume 500us for first sample
        delta = 500 / 1000000

    # Compute headset rotation - first data block
    ahrs.update_imu((np.array(frame["gyro"])-gcomp)*GYRO_FACTOR,
                    (np.array(frame["acc"]) >> 4)*ACC_FACTOR, delta)

    d = sensor.build(dict(timestamp=0, gyro=[gx, gy, gz], acc=[ax, ay, az]))
    frame = sensor.parse(d)
    time2 = frame["timestamp"]
    if time2 < time1:
        time1 = time1 - (1 << 24)
    delta = (time2 - time1) / 1000000

    # Compute headset rotation - second data block
    ahrs.update_imu((np.array(frame["gyro"])-gcomp)*GYRO_FACTOR,
                    (np.array(frame["acc"]) >> 4)*ACC_FACTOR, delta)

    ahrs1 = ahrs.quaternion.yaw_pitch_roll
    roll2 = ahrs1[0]
    pitch2 = ahrs1[1]
    yaw2 = ahrs1[2]
    print(ahrs1)

    qx1, qy1, qz1, qw1 = euler_to_quaternion([yaw2, pitch2, roll2])

    # # protobuf explain https://stackoverflow.com/questions/23726335/how-to-assign-to-repeated-field
    conps.ControllerState.Orientation.extend([qx1, qy1, qz1, qw1])
    conps.ControllerState.Position.extend([0, 1.5, -0.5])

    Serialize_conps = conps.SerializeToString()
    read_back = ControllerStateRequest()
    read_back.ParseFromString(Serialize_conps)
    # print(read_back)

    controller.send(Serialize_conps)
    con_ans = controller.recv()
    # con_res1 = con_res.ParseFromString(con_ans)

    # print(con_ans)

    # print(gyro, accel)
    # print((roll2,pitch2),(roll1,pitch1))

# ================================================================

controller_ps5.deactivate()
headset.close()
controller.close()
