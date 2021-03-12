#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
title: Motor Command Node - ME439 Intro to robotics, wisc.edu
"""

import rospy
from std_msgs.msg import Float32
import PID_controler as pid


def talker_for_wheel_speeds():
    
    rospy.init_node('MotorCommandNode',anonymous=False)
    
    pub_wheel_speed_desired_left = rospy.Publisher('wheel_speed_desired_left',Float32,queue_size=1)
    pub_wheel_speed_desired_right = rospy.Publisher('wheel_speed_desired_right',Float32,queue_size=1)
    
    wheel_speed_desired_left_msg = Float32()
    max_motor_command = 400.0
    K_p = 50
    K_d = 0.1
    init_speed = 200
    while True: 
        error = input('error ')
        
        PID  = pid.robot_PID(error,K_p,K_d)
        PID.PIDcontrol()
        
        print(PID.PID_val)
        wheel_speed_desired_left = init_speed + PID.PID_val
        if (wheel_speed_desired_left > max_motor_command and (no_line == 0)):
           wheel_speed_desired_left = max_motor_command
        
        wheel_speed_desired_left_msg = Float32()
        wheel_speed_desired_left_msg.data = wheel_speed_desired_left
        pub_wheel_speed_desired_left.publish(wheel_speed_desired_left_msg)
        
        wheel_speed_desired_right = init_speed - PID.PID_val 

        if(wheel_speed_desired_right > max_motor_command and (no_line == 0)):
            wheel_speed_desired_right = max_motor_command
           
        print('left = '+str(wheel_speed_desired_left)+' right= '+str(wheel_speed_desired_right))    
        wheel_speed_desired_right_msg = Float32()
        wheel_speed_desired_right_msg.data = wheel_speed_desired_right
        pub_wheel_speed_desired_right.publish(wheel_speed_desired_right_msg)
        
        
        
# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_wheel_speeds()
    except rospy.ROSInterruptException: 
        pass
