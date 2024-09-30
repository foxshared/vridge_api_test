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
                 "ProtocolVersion": 2, "Code": 1})
answer = master.recv_json()
print(answer)

if answer['ProtocolVersion'] != 3 or answer['Code'] != 0:
    print("HeadTracking: incompatible protocol, or not available")
    exit(0)

headset_addr = answer['EndpointAddress']

print("Found:")
print("Headset", headset_addr)


# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)
headset.connect(headset_addr)

print("connect headset")

# SendRadRotationAndPosition
anglesposition = Struct(
    Const(2, Int32ul),  # Version
    Const(3, Int32ul),  # SendRadRotationAndPosition
    Const(24, Int32ul),  # DataLength
    "data" / Padded(64, Array(6, Float32l)),
)

justposition = Struct(
    Const(2, Int32ul),  # Version
    Const(5, Int32ul),  # SendPosition
    Const(12, Int32ul),  # DataLength
    "data" / Padded(64, Array(3, Float32l)),
)

# # set the initial position
# if options.pitch or options.yaw or options.roll:
#     output = anglesposition.build(dict(data=[ \
# 		math.radians(options.pitch), \
# 		math.radians(options.yaw), \
# 		math.radians(options.roll), \
# 		options.X, \
# 		options.Y, \
# 		options.Z ]))
# else:
# 	output = justposition.build(dict(data=[ \
# 		options.X, \
# 		options.Y, \
# 		options.Z ]))

data1 = dict(data=[
    0.0,
    3,
    0])


# print(justposition.build(data1))


output = justposition.build(data1)

headset.send(output)
answer1 = headset.recv()

headset.close()
