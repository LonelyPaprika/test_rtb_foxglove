import os
import threading
import asyncio
from base64 import standard_b64encode
from types import ModuleType

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np

from foxglove_websocket import run_cancellable
from foxglove_base_msgs.msgs import FloatMsg_pb2, FloatsMsg_pb2


from foxglove_base_server.server import FoxgloveServerHandler
from foxglove_base_server.channel import Channel
from foxglove_base_server.msg import Msg

def get_schema(msg_module: ModuleType, msg_bin: str):
    with open(os.path.join(os.path.dirname(msg_module.__file__), msg_bin), "rb") as schema_bin:
        return standard_b64encode(schema_bin.read()).decode("ascii")

async def start_server_async(server_handler):
    await server_handler.start_server()

server_handler = FoxgloveServerHandler()
msg = Msg(FloatMsg_pb2.FloatMsg(data=10.0), Channel("manipulability", "protobuf", "FloatMsg", get_schema(FloatMsg_pb2, "FloatMsg.bin")))
joints = Msg(FloatsMsg_pb2.FloatsMsg(), Channel("joints", "protobuf", "FloatsMsg", get_schema(FloatsMsg_pb2, "FloatsMsg.bin")))
joints.msg.data.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

server_handler.add_msg(msg)
server_handler.add_msg(joints)

async def run_server():
    await start_server_async(server_handler)

thread = threading.Thread(target=asyncio.run, args=(run_server(),))
thread.start()

env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.2, 0.2, 0.45)

arrived = False
env.add(panda)

dt = 0.05

while not arrived:
    v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, 0.1)
    panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v
    joints = Msg(FloatsMsg_pb2.FloatsMsg(), Channel("joints", "protobuf", "FloatsMsg", get_schema(FloatsMsg_pb2, "FloatsMsg.bin")))
    joints.msg.data.extend([panda.q[0], panda.q[1], panda.q[2], panda.q[3], panda.q[4], panda.q[5]])
    server_handler.update_msg(joints)
    server_handler.update_msg(Msg(FloatMsg_pb2.FloatMsg(data=panda.manipulability(panda.q)), Channel("manipulability", "protobuf", "FloatMsg", get_schema(FloatMsg_pb2, "FloatMsg.bin"))))
    env.step(dt)