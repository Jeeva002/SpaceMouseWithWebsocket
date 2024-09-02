#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy  # Import the rospy module for ROS
from std_msgs.msg import Float64MultiArray  # Import Float64MultiArray message type
import sys, os
sys.dont_write_bytecode = True
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../common/imp")))  # Add custom import path

# Robot ID and Model constants
ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509"

# Import necessary modules from custom libraries
from multiprocessing import Process
import datetime
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *
from DRFC import *
from dsr_msgs.msg import *
from dsr_msgs.srv import *
from GenerateLog import logFile
from scipy.spatial.transform import Rotation as R
import numpy as np

class spacemouse():
    def __init__(self):
        logFile.writeDebug('space mouse robot control program initiated')
        self.pos = [0, 0, 0, 0, 0, 0]  # Initialize position and rotation variables
        self.rx = 0
        self.ry = 0
        self.rz = 0
        self.only_once_flag = False
        self.posFlag = True
        self.data_to_send = Float64MultiArray()  # Initialize data to send via ROS
        self.pub = rospy.Publisher('pos', Float64MultiArray, queue_size=100)  # ROS Publisher for position data
        self.permissionFlag = True  # Flag to control movement permission
        self.roll = 0
        self.yaw = 0
        self.pitch = 0
        self.previousRx = 0
        self.previousRy = 0
        self.previousRz = 0

    def rotationCalculation(self, eulerAngle, relativeValue):
        """
        Function to calculate and apply incremental rotations based on relative values.

        Args:
        - eulerAngle: Current Euler angles (ZYZ convention)
        - relativeValue: Relative rotation values to apply (deltaRx, deltaRy, deltaRz)

        Returns:
        - currentEulerAngles: Current Euler angles (in radians)
        - modified_euler_angles: Modified Euler angles after applying relative rotations
        """
        currentEulerAngles = [eulerAngle[0] * 0.017444444, eulerAngle[1] * 0.017444444, eulerAngle[2] * 0.017444444]  # Convert to radians
        r = R.from_euler('zyz', currentEulerAngles)  # Create rotation object
        currentRotationMatrix = R.from_euler('zyz', currentEulerAngles).as_matrix()  # Convert to rotation matrix
        
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

        # Combine rotation matrices
        rx_ry_rz_mat = (rxRotationMatrix.dot(ryRotationMatrix)).dot(rzRotationMatrix)
        modified_rotation_matrix = rx_ry_rz_mat.dot(currentRotationMatrix)  # Apply relative rotation
        
        # Convert modified rotation matrix back to Euler angles
        modified_euler_angles = R.from_matrix(modified_rotation_matrix).as_euler('zyz', degrees=False)

        return currentEulerAngles, modified_euler_angles

    def callback(self, data):
        """
        Callback function for receiving data from the spacemouse.

        Args:
        - data: Float64MultiArray message containing spacemouse data
        """
        global count
        logFile.writeInfo('spacemouse data published and received')
        try:
            # Check if data is within permissible range and permission flag is set
            if (data.data[0] != 0 or data.data[1] != 0 or data.data[2] != 0) and self.permissionFlag == True:
                if self.posFlag == True:
                    self.rob_pos = get_current_posx()  # Get current robot position
                    self.posFlag = False  # Set position flag

                # Convert spacemouse data to robot position increments
                if 0.2 < data.data[0] < 1.9:
                    self.x = data.data[0] * 30
                    self.pos[0] = self.x
                if -0.2 > data.data[0] > -1.9:
                    self.x = data.data[0] * 30
                    self.pos[0] = self.x
                if -0.2 < data.data[0] < 0.2:
                    self.pos[0] = 0

                if 0.2 < data.data[1] < 1.9:
                    self.y = -data.data[1] * 30
                    self.pos[1] = self.y
                if -0.2 > data.data[1] > -1.9:
                    self.y = -data.data[1] * 30
                    self.pos[1] = self.y
                if -0.2 < data.data[1] < 0.2:
                    self.pos[1] = 0

                if 0.5 < data.data[2] < 1.9:
                    self.z = -data.data[2] * 30
                    self.pos[2] = self.z
                if -0.5 > data.data[2] > -1.9:
                    self.z = -data.data[2] * 30
                    self.pos[2] = self.z
                if -0.5 < data.data[2] < 0.5:
                    self.pos[2] = 0

                if 0.7 < data.data[4] < 1.9:
                    self.rx = -data.data[4] * 12
                    self.pos[3] = self.rx
                if -0.7 > data.data[4] > -1.9:
                    self.rx = -data.data[4] * 12
                    self.pos[3] = self.rx
                if -0.7 < data.data[4] < 0.7:
                    self.pos[3] = self.rx

                if 0.7 < data.data[3] < 1.9:
                    self.ry = data.data[3] * 12
                    self.pos[4] = self.ry
                if -0.7 > data.data[3] > -1.9:
                    self.ry = data.data[3] * 12
                    self.pos[4] = self.ry
                if -0.7 < data.data[3] < 0.7:
                    self.pos[4] = self.ry

                if 0.7 < data.data[5] < 1.9:
                    self.rz = -data.data[5] * 12
                    self.pos[5] = self.rz
                if -0.7 > data.data[5] > -1.9:
                    self.rz = -data.data[5] * 12
                    self.pos[5] = self.rz
                if -0.7 < data.data[5] < 0.7:
                    self.pos[5] = self.rz

                checkPosJ = get_current_posj()  # Check current joint positions
                self.x = 0
                self.y = 0
                self.z = 0

                # Check if joint 6 is within permissible range
                if -340 < checkPosJ[5] < 340:
                    self.only_once_flag = True
                    logFile.writeInfo('pos data published to safety verification function')

                    incrementValue = [self.rx, self.ry, self.rz]  # Incremental rotation values
                    currentEulerAngle, computedEuler = self.rotationCalculation([self.rob_pos[0][3], self.rob_pos[0][4], self.rob_pos[0][5]], incrementValue)
                    
                    # Get current and desired rotation matrices
                    currentRotationMatrix = R.from_euler('zyz', currentEulerAngle).as_matrix()
                    desiredRotationMatrix = R.from_euler('zyz', computedEuler).as_matrix()
                    relativeRotationMatrix = np.dot(currentRotationMatrix, desiredRotationMatrix.T)

                    # Convert relative rotation matrix back to Euler angles
                    relativeEuler = R.from_matrix(relativeRotationMatrix).as_euler('zyz')
                    relativeEuler[0] = relativeEuler[0] * 57.324840764
                    relativeEuler[1] = relativeEuler[1] * 57.324840764
                    relativeEuler[2] = relativeEuler[2] * 57.324840764

                    set_velx(40, 20)  # Set velocity and acceleration
                    set_accx(40, 20)
                    print("POS", self.pos[0], self.pos[1], self.pos[2], relativeEuler[0], relativeEuler[1], relativeEuler[2])
                    
                    # Move to desired position using amovel command
                    amovel(posx(self.pos[0], self.pos[1], self.pos[2], relativeEuler[0], relativeEuler[1], relativeEuler[2]), ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    self.rx = 0
                    self.ry = 0
                    self.rz = 0
                    self.x = 0
                    self.y = 0
                    self.z = 0
                else:
                    print("joint6")
                    jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)  # Stop movement
                    self.permissionFlag = False  # Disable movement permission

            # Check if spacemouse data is near zero and the flag is set to stop movement
            elif -0.2 < data.data[0] < 0.2 and -0.2 < data.data[1] < 0.2 and -0.2 < data.data[2] < 0.2 and self.only_once_flag == True:
                print("else")
                self.only_once_flag = False
                jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)  # Stop movement
                logFile.writeInfo('robot stopped moving')

        except Exception as e:
            # Exception handling
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(e, fname, exc_tb.tb_lineno)

    def errorLog(self, data):
        """
        Function to handle error logging and recovery based on received data.

        Args:
        - data: Float64MultiArray message containing data related to errors
        """
        state = get_robot_state()  # Get current robot state
        MY_flag = False

        # Check if robot is in error state or needs recovery
        if state == 10 or state == 3 or state == 8:
            logFile.writeError('servo turned off')
            MY_flag = True
            
            # Perform error recovery actions
            if MY_flag == True:
                file1 = open("/home/ubuntu/jv_ws/src/doosan-robot/dsr_example/py/scripts/main/safeoffErrorEvaluation.txt", "a")
                error = get_last_alarm()  # Get last alarm/error
                pos = get_current_posx()  # Get current position
                logFile.writeError(error)
                file1.write(f'\n {datetime.datetime.now()} Targetpos:{data.data},vel:40,acc=40 \n {datetime.datetime.now()} currentpos:{pos[0]}')
                file1.close()
                set_robot_control(CONTROL_RESET_SAFET_OFF)  # Reset safety off control
                logFile.writeInfo('servo turning ON')
                time.sleep(2)
                MY_flag = False

    def homePosition(self, button):
        """
        Function to handle homing and recovery based on button press data.

        Args:
        - button: Float64MultiArray message containing button press data
        """
        homePos = posj(0, 0, 90, 0, 90, 0)  # Define home position

        # Check button press data and perform corresponding actions
        if button.data[0] == 1:
            self.permissionFlag = False
            jog_multi([0, 0, 0, 0, 0, 0], MOVE_REFERENCE_BASE, 0)  # Stop movement
            movej(homePos, 20, 20)  # Move to home position
            self.permissionFlag = True
            self.posFlag = True
        elif button.data[1] == 1:
            recoveryj = get_current_posj()  # Get current joint positions
            recoveryPosj = posj(recoveryj[0], recoveryj[1], recoveryj[2], recoveryj[3], recoveryj[4], 0)  # Define recovery position
            movej(recoveryPosj, 20, 20)  # Move to recovery position
            print('recovery')
            self.permissionFlag = True
            self.posFlag = True

connexion3d = spacemouse()  # Initialize spacemouse control object

def listener():
    """
    Function to initialize ROS node and subscribe to relevant topics.
    """
    rospy.init_node('sapceMouse', anonymous=True)  # Initialize ROS node
    rospy.Subscriber("spacemouse", Float64MultiArray, connexion3d.callback)  # Subscribe to spacemouse data topic
    rospy.Subscriber("pos", Float64MultiArray, connexion3d.errorLog)  # Subscribe to position data topic
    rospy.Subscriber("spacemouseButton", Float64MultiArray, connexion3d.homePosition)  # Subscribe to button press topic
    rospy.spin()  # Keep the node running

def main():
    """
    Main function to start the listener process.
    """
    process1 = Process(target=listener)  # Create a new process for the listener function
    process1.start()  # Start the process

if __name__ == "__main__":
    main()  # Run the main function if script is executed directly
