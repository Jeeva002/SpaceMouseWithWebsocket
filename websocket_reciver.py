#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import websockets
import rospy
import json
from std_msgs.msg import String
import datetime
from saveDataDetails import log_management

# async def send_message_callback(websocket):
#     while True:
#         json_message = json.dumps(message_data)
#         feedback = await websocket.send(json_message)
#         await asyncio.sleep(0.0001)
#         # print(f"Sent: {message}")
# def callback(msg):
#     global message_data
#     message_data = {'robot_state_str': msg.robot_state_str}
#     return message_data

async def receive_message(uri):
    async with websockets.connect(uri) as websocket:
        rospy.loginfo(f"Connected to {uri}")

        while True:
            response = await websocket.recv()
            # print("received Message",response)
            dataToPublish(response)
            # print(f"Received: {response}")

def dataToPublish(message):
    try:
        parsed_message = json.loads(message)
        
        # Check if the parsed message is a dictionary
        if isinstance(parsed_message, dict):
            # Wrap the dictionary in a list
            data_to_send = [parsed_message]
            json_msg = json.dumps(data_to_send)
            timestamp = parsed_message['timeStamp']

            log_management.writeBeforeWebsocket(timestamp,message)
            log_management.writeAfterWebsocket(message)
            # Publish the JSON string to the ROS topic
            log_management.writeBeforeRos(json_msg)
            pub.publish(json_msg)
            # timestamp_as_datetime = datetime.datetime.fromtimestamp(timestamp)
           # print("timestamp",timestamp_as_datetime)
            rospy.loginfo(f"Published data: {timestamp}")
        else:
            rospy.logwarn(f"Expected a dictionary but received: {parsed_message}")
    except json.JSONDecodeError:
        rospy.logwarn(f"Received invalid JSON: {message}")


# async def main(uri):
#     async with websockets.connect(uri) as websocket:
#         rospy.loginfo(f"Connected to {uri}")
#         # Start sending and receiving concurrently
#         send_task = asyncio.create_task(send_message_callback(websocket))
#         receive_task = asyncio.create_task(receive_message(websocket))
        
#         # Wait until both tasks are done (they won't unless there's an error)
#         await asyncio.gather(send_task, receive_task)


if __name__ == "__main__":
    rospy.init_node('websocket_receiver', anonymous=True)
    pub = rospy.Publisher('spacemouseValues', String, queue_size=100)
    # rospy.Subscriber('/dsr01a0509/state', RobotState, callback)
    uri = "wss://websocket.atrehealthtech.com/royal/test"  # WebSocket server URI

# Start the event loop
    asyncio.get_event_loop().run_until_complete(receive_message(uri))
