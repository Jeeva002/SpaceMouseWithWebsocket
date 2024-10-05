#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import datetime

class logHandling:
    def __init__(self):
        self.file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  

    def writeDebug(self,msg):
        file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  
        file.write(f'\n {datetime.datetime.now()} Debug:{msg}')
        file.close()

    def writeInfo(self,msg):
        file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  
        file.write(f'\n {datetime.datetime.now()} Info:{msg}')
        file.close()

    def writeError(self,msg):
         file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  
         file.write(f'\n {datetime.datetime.now()} Error:{msg}')
         file.close()

    def writeWarning(self,msg):
         file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  
         file.write(f'\n {datetime.datetime.now()} Warning:{msg}')
         file.close()

    def writeCritical(self,msg):
         file=open('/home/amritha/teleop_ws/src/Telerobot_control/doosan-robot/dsr_example/py/scripts/SpaceMouseWithWebsocket/logMonitor.txt', "a"  )  
         file.write(f'\n {datetime.datetime.now()} Critical:{msg}')
         file.close()

logFile=logHandling()

