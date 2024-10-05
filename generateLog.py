#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import rospy
import logging
import datetime
class logManagement():
     def __init__(self):
            # Initialize ROS node
        #    rospy.init_node('my_node')

            # Set up Python logging
            self.logger = logging.getLogger('my_node_logger')
            self.logger.setLevel(logging.DEBUG)

            # Create a handler for writing log messages to a file
            file_handler = logging.FileHandler('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/test.log')  # Change the filename as needed
            file_handler.setLevel(logging.DEBUG)

            # Create a formatter
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            file_handler.setFormatter(formatter)

            # Add the handler to the logger
            self.logger.addHandler(file_handler)

     
     def writeDebug(self,msg):
           self.logger.debug(f'{datetime.datetime.now()} - {msg}')
     def writeInfo(self,msg):  
            self.logger.info(f'{datetime.datetime.now()} - {msg}')
     def writeWarning(self,msg):
           self.logger.warning(f'{datetime.datetime.now()} - {msg}')
     def writeError(self,msg):
           self.logger.error(f'{datetime.datetime.now()} - {msg}')
           
logFile=logManagement()







