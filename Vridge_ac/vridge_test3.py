# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *
from construct import Int32ul, Float32l


# http://zeromq.org
# ZeroMQ-4.0.4-miru1.0-x86.exe
#
# https://www.nico-maas.de/?p=1081
# https://github.com/zeromq/pyzmq
# pyzmq-16.0.2.zip

import zmq
import time
from math import radians
from pyrr import Quaternion, Matrix44, Vector3
import numpy as np
import math
import struct

from vridge_api_pb2 import HeadTrackingRequest, HeadTrackingResponse, ControllerStateRequest, ControllerStateResponse, VRController

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

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
# ================================================================ Controller send read data
conps = ControllerStateRequest()
conps.Version = 3
# t = Padded(2,Byte)
# t1 = t.build(1)
# print(t1)
conps.TaskType = 1
conps.ControllerState.ControllerId = 1
conps.ControllerState.Status = 0

pitch = radians(0)
roll = radians(180)
yaw = radians(90)

qx1, qy1, qz1, qw1= euler_to_quaternion([yaw,pitch,roll])

# # protobuf explain https://stackoverflow.com/questions/23726335/how-to-assign-to-repeated-field
conps.ControllerState.Orientation.extend([qx1, qy1, qz1, qw1])
conps.ControllerState.Position.extend([0,1.5,0])

Serialize_conps = conps.SerializeToString()
read_back = ControllerStateRequest()
read_back.ParseFromString(Serialize_conps)
print(read_back)

controller.send(Serialize_conps)
con_ans = controller.recv()
# con_res1 = con_res.ParseFromString(con_ans)

print(con_ans)

# ================================================================

headset.close()
controller.close()
