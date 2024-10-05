#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from std_msgs.msg import String
import json
# Import ROS Python library
import rospy
# Import message type for publishing/subscribing Float64MultiArray
from std_msgs.msg import Float64MultiArray
# Import system-specific parameters and functions
import sys, os
# Prevents Python from writing .pyc files
sys.dont_write_bytecode = True
# Import time-related functions
import time
# Append custom module path for importing custom scripts
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../common/imp")))
from saveDataDetails import log_management
# Define robot ID and model
ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509"
# Import multiprocessing for handling multiple processes
from multiprocessing import Process
# Import datetime for timestamping
import datetime
# Import custom module for initializing robot
import DR_init
# Set robot ID and model in DR_init module
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
# Import specific modules for robot control
from DSR_ROBOT import *
from DRFC import *
# Import message types for communication
from dsr_msgs.msg import *
from dsr_msgs.srv import *
# Import custom module for logging
from generateLog import logFile
# Import rotation handling module
from scipy.spatial.transform import Rotation as R
# Import numerical operations library
import numpy as np
from logs import logFile

# Define class spacemouse
class spacemouse():
        
    def __init__(self):
        # Initialize spacemouse class
        logFile.writeDebug('space mouse robot control program initiated')
        # Initialize position and rotation variables
        self.pos = [0, 0, 0, 0, 0, 0]
        self.rx = 0
        self.ry = 0
        self.rz = 0
        # Flags for control flow
        self.only_once_flag = False
        self.posFlag = True
        self.permissionFlag = True
        # self.positionFile=open("/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/SpaceMousePositions.txt", "a")                        

        # Variables for storing rotational angles (roll, pitch, yaw)
        self.roll = 0
        self.yaw = 0
        self.pitch = 0
        # Previous rotational angles
        self.previousRx = 0
        self.previousRy = 0
        self.previousRz = 0

    def rotationCalculation(self, eulerAngle, relativeValue):
        # Calculate rotation based on current Euler angles and relative values
        currentEulerAngles = [eulerAngle[0] * 0.017444444, eulerAngle[1] * 0.017444444, eulerAngle[2] * 0.017444444]
        # Create rotation object from ZYZ Euler angles
        r = R.from_euler('zyz', currentEulerAngles)
        currentRotationMatrix = r.as_matrix()

        # Convert relative values to radians
        deltaRx = relativeValue[0] * 0.017444444
        deltaRy = relativeValue[1] * 0.017444444
        deltaRz = relativeValue[2] * 0.017444444

        # Define rotation matrices for each axis
        rxRotationMatrix = np.array([[1, 0, 0],
                                     [0, np.cos(deltaRx), -np.sin(deltaRx)],
                                     [0, np.sin(deltaRx), np.cos(deltaRx)]])

        ryRotationMatrix = np.array([[np.cos(deltaRy), 0, np.sin(deltaRy)],
                                     [0, 1, 0],
                                     [-np.sin(deltaRy), 0, np.cos(deltaRy)]])

        rzRotationMatrix = np.array([[np.cos(deltaRz), -np.sin(deltaRz), 0],
                                     [np.sin(deltaRz), np.cos(deltaRz), 0],
                                     [0, 0, 1]])

        # Apply rotations sequentially
        rx_mat = currentRotationMatrix.dot(rxRotationMatrix)
        rx_ry_mat = rx_mat.dot(ryRotationMatrix)
        rx_ry_rz_mat = rx_ry_mat.dot(rzRotationMatrix)

        # Modified rotation matrix
        modified_rotation_matrix = rx_ry_rz_mat

        # Get modified Euler angles from modified rotation matrix
        modified_euler_angles = R.from_matrix(modified_rotation_matrix).as_euler('zyz', degrees=False)

        # Convert modified Euler angles to degrees
        modified_euler_angles[0] = modified_euler_angles[0] * 57.324840764
        modified_euler_angles[1] = modified_euler_angles[1] * 57.324840764
        modified_euler_angles[2] = modified_euler_angles[2] * 57.324840764

        return modified_euler_angles

    def callback(self, data):
        # Callback function for processing spacemouse data
        global count
        print("call backed")
        try:
            # Parse the JSON string
            json_data_list = json.loads(data.data)
            log_management.writeAfterRos(json_data_list)
            # Check if it's a list and has at least one item
            if isinstance(json_data_list, list) and len(json_data_list) > 0:
                json_data = json_data_list[0]
                
                # Ensure json_data is a dictionary
                if isinstance(json_data, dict):
                    x = json_data.get('x', None)
                    y = json_data.get('y', None)
                    z = json_data.get('z', None)
                    roll = json_data.get('roll', None)
                    pitch = json_data.get('pitch', None)
                    yaw = json_data.get('yaw', None)
                    button0 = json_data.get('button0', None)
                    button1 = json_data.get('button1', None)
                    
                    data={"x":x,"y":y,"z":z,"roll":roll,"pitch":pitch,"yaw":yaw}
                    print("data",data["x"])
                    rospy.loginfo(f"x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}, button0: {button0}, button1: {button1}")
                    
                else:
                    rospy.logwarn("Expected a dictionary inside the list, but received a different type")
            else:
                rospy.logwarn("Expected a non-empty list but received a different type")
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to decode JSON: {e}")

        logFile.writeInfo('spacemouse data received')
       # self.positionFile.write(f'\n {datetime.datetime.now()} spaceMouse Received:{data.data} ')
    
        try:
            if data["x"] != 0 or data["y"] != 0 or data["z"] != 0 and self.permissionFlag == True:
                # Collect current robot position if flag is True
                if self.posFlag == True:
                    self.rob_pos = get_current_posx()
                    self.posFlag = False
                    print("posCollected")

                # Handle X-axis position data
                if 0.2 < data["x"] < 1.9:
                    x = data["x"] * 30
                    self.pos[0] = self.rob_pos[0][0] + x
                if -0.2 > data["x"] > -1.9:
                    x = data["x"] * 30
                    self.pos[0] = self.rob_pos[0][0] + x
                if -0.2 < data["x"] < 0.2:
                    self.pos[0] = self.rob_pos[0][0]

                # Handle Y-axis position data
                if 0.2 < data["y"] < 1.9:
                    y = data["y"] * 30
                    self.pos[1] = self.rob_pos[0][1] + y
                if -0.2 > data["y"] > -1.9:
                    y = data["y"] * 30
                    self.pos[1] = self.rob_pos[0][1] + y
                if -0.2 < data["y"] < 0.2:
                    self.pos[1] = self.rob_pos[0][1]

                # Handle Z-axis position data
                if 0.5 < data["z"] < 1.9:
                    z = data["z"] * 30
                    self.pos[2] = self.rob_pos[0][2] + z
                if -0.5 > data["z"] > -1.9:
                    z = data["z"] * 30
                    self.pos[2] = self.rob_pos[0][2] + z
                if -0.5 < data["z"] < 0.5:
                    self.pos[2] = self.rob_pos[0][2]

                # Handle roll rotation data
                if 0.7 < data["pitch"] < 1.9:
                    self.rx = data["pitch"] * 12
                    self.pos[3] = self.rob_pos[0][3]
                if -0.7 > data["pitch"] > -1.9:
                    self.rx = data["pitch"] * 12
                    self.pos[3] = self.rob_pos[0][3]
                if -0.7 < data["pitch"] < 0.7:
                    self.pos[3] = self.rob_pos[0][3]

                # Handle pitch rotation data
                if 0.7 < data["roll"] < 1.9:
                    self.ry = data["roll"] * 12
                    self.pos[4] = self.rob_pos[0][4]
                if -0.7 > data["roll"] > -1.9:
                    self.ry = data["roll"] * 12
                    self.pos[4] = self.rob_pos[0][4]
                if -0.7 < data["roll"] < 0.7:
                    self.pos[4] = self.rob_pos[0][4]

                # Handle yaw rotation data
                if 0.7 < data["yaw"] < 1.9:
                    self.rz = -data["yaw"] * 12
                    self.pos[5] = self.rob_pos[0][5]
                if -0.7 > data["yaw"] > -1.9:
                    self.rz = -data["yaw"] * 12
                    self.pos[5] = self.rob_pos[0][5]
                if -0.7 < data["yaw"] < 0.7:
                    self.pos[5] = self.rob_pos[0][5]

                # Check joint position
                checkPosJ = get_current_posj()
                print(self.rx, self.rz, self.ry)

                # Check if joint position within limits
                if -340 < checkPosJ[5] < 340:
                    self.only_once_flag = True
                    incrementValue = [self.rx, self.ry, self.rz]
                    computedEuler = self.rotationCalculation([self.pos[3], self.pos[4], self.pos[5]], incrementValue)
                    print(computedEuler)
                    set_velx(40, 20)
                    set_accx(40, 20)
                    print(self.pos[0], self.pos[1], self.pos[2], computedEuler[0], computedEuler[1], computedEuler[2])
                    posValues=[self.pos[0], self.pos[1], self.pos[2], computedEuler[0], computedEuler[1], computedEuler[2]]
                    log_management.robotTime(posValues)
                    amovel(posx(self.pos[0], self.pos[1], self.pos[2], computedEuler[0], computedEuler[1], computedEuler[2]))
                    self.rob_pos[0][0] = self.pos[0]
                    self.rob_pos[0][1] = self.pos[1]
                    self.rob_pos[0][2] = self.pos[2]
                    self.rob_pos[0][3] = computedEuler[0]
                    self.rob_pos[0][4] = computedEuler[1]
                    self.rob_pos[0][5] = computedEuler[2]
                    self.rx = 0
                    self.ry = 0
                    self.rz = 0
                else:
                    print("joint6")
                    jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)
                    self.permissionFlag = False
            elif -0.2 < data["x"] < 0.2 and -0.2 < data["y"] < 0.2 and -0.2 < data["z"] < 0.2 and self.only_once_flag == True:
                print("else")
                self.only_once_flag = False
                jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)
                logFile.writeInfo('robot stopped moving')

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(e, fname, exc_tb.tb_lineno)

    def errorLog(self, data):
        # Error logging function
        state = get_robot_state()
        MY_flag = False

        if state == 10 or state == 3 or state == 8:
            logFile.writeError('servo turned off')
            MY_flag = True
            if MY_flag == True:
                # file1 = open("/home/atre/spacemouseTeleoperation/src/doosan-robot/dsr_example/py/scripts/spaceMouse/safeoffErrorEvaluation.txt", "a")
                error = get_last_alarm()
                pos = get_current_posx()
                # file1.write(f'\n {datetime.datetime.now()} Targetpos:{data.data},vel:40,acc=40 \n {datetime.datetime.now()} currentpos:{pos[0]}')
                # file1.close()
                set_robot_control(CONTROL_RESET_SAFET_OFF)
                time.sleep(2)
                MY_flag = False

    def homePosition(self, button):
        # Home position handling function (not fully implemented in the provided script)
        pass
        # homePos = posj(0, 0, 90, 0, 90, 0)
        # if button.data[0] == 1:
        #     self.permissionFlag = False
        #     jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)
        #     movej(homePos, 20, 20)
        #     self.permissionFlag = True
        #     self.posFlag = True
        # elif button.data[1] == 1:
        #     recoveryj = get_current_posj()
        #     recoveryPosj = posj(recoveryj[0], recoveryj[1], recoveryj[2], recoveryj[3], recoveryj[4], 0)
        #     movej(recoveryPosj, 20, 20)
        #     print('recovery')
        #     self.permissionFlag = True
        #     self.posFlag = True

# Initialize spacemouse object
connexion3d = spacemouse()

def listener():
    # ROS listener function
    print("listener inside")
    rospy.init_node('spaceMouse')
    rospy.Subscriber("spacemouseValues", String, connexion3d.callback)
    rospy.Subscriber("pos", Float64MultiArray, connexion3d.errorLog)

    rospy.spin()

def main():
    # Main function to start ROS listener process
    process1 = Process(target=listener)
    process1.start()


