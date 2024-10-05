#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import websockets
import rospy
import json
from std_msgs.msg import String
import datetime
from saveDataDetails import log_management
from dsr_msgs.msg import *

async def send_message_callback(message):
    async with websockets.connect(uri) as websocket:
        rospy.loginfo(f"Connected to {uri}")
        try:
            while True:

                response = await websocket.recv()
                print("received Message",response)
                json_message = json.dumps(message.robot_state_str)
                print(json_message)
                send = await websocket.send(json_message)
                # await asyncio.sleep(0.0001)
                print(f"Sent: {json_message}")
                print("send",send)

        except websockets.ConnectionClosed:
            rospy.loginfo("Connection closed")   

def msg_callback(message):
    asyncio.run(send_message_callback(message))
    # asyncio.get_event_loop().run_until_complete(send_message_callback(message))

def msg_listener():
    rospy.init_node('websocket_sender', anonymous=True)
    rospy.Subscriber('/dsr01a0509/state', RobotState, msg_callback)
    rospy.spin()


if __name__ == '__main__':
        uri = "wss://websocket.atrehealthtech.com/royal/joystick/123"
        try:
            msg_listener()
        except rospy.ROSInterruptException:
            pass