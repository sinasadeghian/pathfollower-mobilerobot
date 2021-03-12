#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
"""

import rospy
from std_msgs.msg import Float32 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)
from mobrob_util.msg import ME439WheelSpeeds

def listener(): 
    rospy.init_node('motor_node')
    
    # Subscribe to the "wheel_speed_desired_left" topic
    subL = rospy.Subscriber('/wheel_speed_desired_left', Float32, set_wheel_speed_left) 
    
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    # Subscribe to the "wheel_speed_desired_right" topic
    subR = rospy.Subscriber('/wheel_speed_desired_right', Float32, set_wheel_speed_right) 
    #### END CODE ####
   
    rospy.spin()    # keep the node from exiting


def set_wheel_speed_left(msg_inL): 
    wheel_speed_desired_left = int(msg_inL.data)
    motors.motor1.setSpeed(wheel_speed_desired_left)
#    rospy.loginfo("left wheel set to {0}".format(wheel_speed_desired_left))

def set_wheel_speed_right(msg_inR): 
    wheel_speed_desired_right = int(msg_inR.data)
    motors.motor2.setSpeed(wheel_speed_desired_right)
#    rospy.loginfo("right wheel set to {0}".format(wheel_speed_desired_right))


# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except : 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    pass
