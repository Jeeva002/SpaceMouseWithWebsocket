#!/usr/bin/env python3
import rospy
import asyncio
import websockets
import datetime
import time
from dsr_msgs.msg import *
from saveDataDetails import log_management

# Global variable to hold the most recent ROS message
latest_ros_msg = None
feedback_data = None

def ros_callback(msg):
    # now = datetime.datetime.now()
    # current_time = now.timestamp()
    # Time = datetime.datetime.fromtimestamp(current_time)
    # formatted_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    global latest_ros_msg
    latest_ros_msg = msg.robot_state_str
    # print(latest_ros_msg)
    global feedback_data
    # Adding timestamp to the feedback data 
    # feedback_data ={'status':latest_ros_msg,
    #                 'timestamp':formatted_time}

async def send_message(websocket):
    global latest_ros_msg
    # global feedback_data
    while True:
        if latest_ros_msg:
            formatted_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            feedback_data ={'status':latest_ros_msg,
                    'timestamp':formatted_time}
            # Send the latest ROS message to the WebSocket server
            log_management.feedbackGeneratedfromros(feedback_data)
            await websocket.send(str(feedback_data))
            log_management.feedbackAfterWebsocket(feedback_data)
            # print(f"Sent: {feedback_data}")
        await asyncio.sleep(1)  # Adjust the frequency of sending messages

async def main(uri):
    async with websockets.connect(uri) as websocket:
        # Start sending messages to WebSocket server
        await send_message(websocket)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('ros_websocket_sender', anonymous=True)
    
    # Subscribe to a ROS topic
    rospy.Subscriber('/dsr01a0509/state', RobotState, ros_callback)
    
    # WebSocket server URI
    uri = "wss://websocket.atrehealthtech.com/royal/joystick/123"

    # Run ros callback function and asyncio function in parallel
    loop = asyncio.get_event_loop()
    try:
        # Run ROS in a separate thread
        ros_thread = loop.run_in_executor(None, rospy.spin)

        # Run the WebSocket communication loop
        loop.run_until_complete(main(uri))
    finally:
        loop.close()

