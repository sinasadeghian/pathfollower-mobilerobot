#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32
from lidar import *
# Import "serial" to get data from the AlaMode
import serial   
import traceback 
# Import the message types we will need
from mobrob_util.msg import ME439SensorsRaw
import PID_controler as robot_pid
import time

class Point():
    def __init__(self, x, z, r):
        self.x = x
        self.z = z
        self.r = r

class Rosbot():
    def __init__(self):
        
        self.rosbot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.left_pub = rospy.Publisher('/wheel_speed_desired_left',Float32,queue_size=1)
        self.right_pub = rospy.Publisher('/wheel_speed_desired_right',Float32,queue_size=1)
        self.all_sensors = rospy.Subscriber('/sensors_raw',ME439SensorsRaw,self.light_dark_callback)
        
        # Line following Params
        self.init_speed = rospy.get_param('init_speed')
        self.max_motor_command = rospy.get_param('max_motor_command')
        self.line_K_p = rospy.get_param('K_p')
        self.line_K_d = rospy.get_param('K_d')
        # Initialize sensor readings
        self.R_s = 0.0
        self.M_s = 0.0
        self.L_s = 0.0
        

        self.ctrl_c = False
        self.rate = rospy.Rate(10) #10 hz

        # Lidar Params
        self.base_speed = 100.0
        self.targetAngle = np.pi/2
        self.targetDistance = 1.3
        self.pid_angle = PID(P=50.0, D=0.0)
        self.pid_angle.setPoint(self.targetAngle)
        self.pid_distance = PID(P=50.0, I=0.0, D=0.0)
        self.pid_distance.setPoint(self.targetDistance)
        self.wheelWidth = rospy.get_param('wheel_width_actual')
        self.wheelDiameter = rospy.get_param('wheel_diameter_actual')
        self.points = []
        
        rospy.on_shutdown(self.shutdownhook)
    
    def setSpeed(self, leftSpeed, rightSpeed):
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = leftSpeed
        right_msg.data = rightSpeed
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def aroundObject(self, filteredPoints, speedSetter):
        # Calculate the angular correction
        
        # We assume the 5 closest points will be near eachother, and find the average angle they are from
        # the robot. We are going to following count-clockwise, so we want to turn until they are at ~90
        # to the left.
        closestFive = closestPoint(filteredPoints, n=5)
        angles = np.array([np.arctan2(p.x, p.z) for p in closestFive])
        avgAngle = np.average(angles) + np.pi/2
        avgDistance = np.average(np.array([p.r for p in filteredPoints]))
        omega_angle = self.pid_angle.update(avgAngle)
        omega_distance = self.pid_distance.update(avgDistance)
        #omega_distance = limit(omega_distance, 4.0)
        #print("Dist: {}, Omega: {}   Angle: {}, Omega: {}".format(avgDistance, omega_distance, np.rad2deg(avgAngle), omega_angle))
        #rospy.loginfo("Dist: {},Angle: {} ".format(avgDistance ,np.rad2deg(avgAngle) ))
        # print(omega_angle)
        # print(avgAngle)
        omega = (omega_angle + omega_distance) / 2
        #print("X: {}, Y: {}, R: {}".format(closestFive[0].x, closestFive[0].y, closestFive[0].r))
        deltaAngle = self.targetAngle - avgAngle # negative means turn right
        # deltaDistance = self.targetDistance - avgDistance # negative mean too close
        # delta = deltaAngle - 30*deltaDistance

        vl = self.base_speed - (60 * omega * self.wheelWidth)
        vr = self.base_speed + (60 * omega * self.wheelWidth)
        
        if(np.abs(deltaAngle) > np.pi/5):
                shift = min(vl, vr)
                vl -= shift
                vr -= shift
        
        #rospy.loginfo("Dist: {}, Angle: {}, vl: {}, vr: {}".format(avgDistance, np.rad2deg(avgAngle), vl, vr))

        speedSetter(vl, vr)
        
    def scan_callback(self,msg):
        
        ###############################################################
        #### NOTE : The RP Lidar is able to read valus > 0.135(m) #####
        ###############################################################
        # len.msg.ranges = 720
        self.points = []
        angular_res = 2*np.pi/len(msg.ranges)
        for i in range(0, len(msg.ranges)):
            r = msg.ranges[i]
            x = r*np.cos(i*angular_res) # positive left
            y = r*np.sin(i*angular_res) # positive forward
            self.points.append(Point(x, y, r))


    def main_control_loop(self):
        
        line_following = True
        while not self.ctrl_c:
            filtered = filterHorizon(self.points)
            distance_ahead = 2
            points = self.points
            if(len(points) > 30):
                print(len(points))
                forward_points = [points[i] for i in range(len(points)/2 - 3, len(points)/2 +3)]
                forward_points = filterHorizon(forward_points)
                distance_ahead = np.average(np.array([p.r for p in forward_points]))
                #dist = self.points[len(self.points) / 2].r
            #print("Line Following: {},    Distance: {}".format(line_following, distance_ahead))
            # Line following
            if(line_following):
                # Test if obstacle is ahead
                if(len(self.points) > 1 and distance_ahead < 0.30):
                    line_following = False
                    continue
                self.line_following_controller()
            else:
                # Return control to the line follower
                if(self.R_s < 100 and self.M_s < 100 and self.L_s < 100):
                    line_following = True
                    
                    # Turn 90 degrees left
                    self.setSpeed(0, 100)
                    time.sleep(2.0)
                    self.setSpeed(0, 0)
                if(len(filtered) > 1):
                    self.aroundObject(filtered, self.setSpeed)

    def light_dark_callback(self, msg_in):
        # unpack the message
        self.R_s = msg_in.a0
        self.M_s = msg_in.a1
        self.L_s = msg_in.a2

    def line_following_controller(self):
            
        R_s = self.R_s
        M_s = self.M_s
        L_s = self.L_s
        error = 0
        no_line = 0
        # Errors:
        if(R_s > 100. and M_s > 100. and L_s < 100.):
                error = 2
        elif(R_s > 100. and M_s < 100. and L_s < 100.):
                error = 1
        elif(R_s > 100. and M_s < 100. and L_s > 100.):
                error = 0
        elif(R_s < 100. and M_s < 100. and L_s > 100.):
                error = -1
        elif(R_s < 100. and M_s > 100. and L_s > 100.):
                error = -2
        elif((R_s > 100. and M_s > 100. and L_s > 100.) or (R_s < 100. and M_s < 100. and L_s < 100.)  ): # no line
                # Lidar Case
                no_line = 1
        #PID
        PID  = robot_pid.robot_PID(error,self.line_K_p,self.line_K_d)
        PID.PIDcontrol()
        #print(str(error)+'  '+str(R_s)+' '+str(M_s)+' '+str(L_s)) 
        #determine the left wheel speed
        if(no_line == 1):
                wheel_speed_desired_left = 0.0
                wheel_speed_desired_right = 0.0
        elif(no_line == 0):
                wheel_speed_desired_left = self.init_speed - PID.PID_val
                wheel_speed_desired_right = self.init_speed + PID.PID_val # determine the right wheel speed
        if (wheel_speed_desired_left > self.max_motor_command):
            wheel_speed_desired_left = self.max_motor_command
        if (wheel_speed_desired_right > self.max_motor_command):
            wheel_speed_desired_right = self.max_motor_command

        # pack and publish
        #wheel_speed_desired_left_msg = Float32()
        #wheel_speed_desired_left_msg.data = wheel_speed_desired_left
        #pub_wheel_speed_desired_left.publish(wheel_speed_desired_left_msg)
        #rospy.loginfo(wheel_speed_desired_left_msg)

        # pack and publish
        #wheel_speed_desired_right_msg = Float32()
        #wheel_speed_desired_right_msg.data = wheel_speed_desired_right
        #pub_wheel_speed_desired_right.publish(wheel_speed_desired_right_msg)
        #rospy.loginfo(wheel_speed_desired_right_msg)
        
        self.setSpeed(wheel_speed_desired_left, wheel_speed_desired_right)

    def shutdownhook(self):
        self.ctrl_c = True
    


        
if __name__ == '__main__':
    rospy.init_node('rosbot',anonymous=False) 
    rosbot = Rosbot() 
    try:
        rosbot.main_control_loop()
        
    except rospy.ROSInterruptException:
        traceback.print_exc()
        pass
    
