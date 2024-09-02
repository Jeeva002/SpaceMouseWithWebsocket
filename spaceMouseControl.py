#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy  # Import the rospy module for ROS
from spaceMouse6AxisControl import main  # Import the main function from spaceMouse6AxisControl module

def launchSpaceMouse():
    """
    Function to launch the main function from spaceMouse6AxisControl module.
    """
    main()  # Call the main function from spaceMouse6AxisControl module

if __name__ == "__main__":


    launchSpaceMouse()  # Call the function to launch the space mouse control

    # Wait until rospy shuts down (this typically won't happen unless rospy.shutdown() is called)
    while not rospy.is_shutdown():
        pass  # Do nothing, just wait
