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

from vridge_api_pb2 import HeadTrackingRequest,HeadTrackingResponse


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


print("Found:")
print("Headset", headset_addr)


# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)
headset.connect(headset_addr)

print("connect headset")


# ================================================================
# Protobuf serialized vridge api
send_track = HeadTrackingRequest()

send_track.Version = 3
send_track.TaskType = 5
data = Struct("data"/Padded(64, Array(3, Float32l)))
data1 = dict(data=[2, 3, 0])

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

headset.close()

# ================================================================
