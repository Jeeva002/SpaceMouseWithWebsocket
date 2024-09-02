#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import websockets
import rospy
import json
from std_msgs.msg import String
import datetime
from saveDataDetails import log_management
async def receive_data(uri):
    async with websockets.connect(uri) as websocket:
        rospy.loginfo(f"Connected to {uri}")
        
        try:
            while True:
                # Receive data from the server
                message = await websocket.recv()
                
                log_management.writeAfterWebsocket(message)
                print("received Message",message)
                dataToPublish(message)
                # Ensure that message is a JSON string and parse it

        except websockets.ConnectionClosed:
            rospy.loginfo("Connection closed")
def dataToPublish(message):
    try:
        parsed_message = json.loads(message)
        
        # Check if the parsed message is a dictionary
        if isinstance(parsed_message, dict):
            # Wrap the dictionary in a list
            data_to_send = [parsed_message]
            json_msg = json.dumps(data_to_send)
            timestamp = parsed_message['timeStamp']
            # Publish the JSON string to the ROS topic
            log_management.writeBeforeRos(json_msg)
            pub.publish(json_msg)
            timestamp_as_datetime = datetime.datetime.fromtimestamp(timestamp)
           # print("timestamp",timestamp_as_datetime)
            rospy.loginfo(f"Published data: {timestamp_as_datetime}")
        else:
            rospy.logwarn(f"Expected a dictionary but received: {parsed_message}")
    except json.JSONDecodeError:
        rospy.logwarn(f"Received invalid JSON: {message}")
if __name__ == "__main__":
    rospy.init_node('websocket_Receiver', anonymous=True)
    pub = rospy.Publisher('spacemouseValues', String, queue_size=100)
    uri = "wss://websocket.atrehealthtech.com/royal/test"  # WebSocket server URI

    # Start the asyncio event loop
    asyncio.get_event_loop().run_until_complete(receive_data(uri))
