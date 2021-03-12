#!/usr/bin/env python

import rospy
# Import "serial" to get data from the AlaMode
import serial   
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32
from mobrob_util.msg import ME439SensorsRaw


# Publish sensors data at the rate it comes in
# Here we publish: 
#  Analogs (levels), 
#  Ultrasound (microseconds), 
#  and Encoders (counts) 
# all in ONE TOPIC 
def sensors_reader(): 
    # Launch a node called "sensors_node"
    rospy.init_node('sensors_node', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog! 
    pub_sensors_raw = rospy.Publisher('/sensors_raw', ME439SensorsRaw, queue_size=1)
    
    # Declare the message that will go on the topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We put data in it using the .data field of the message.
    msg_sensors_raw = ME439SensorsRaw()
    
    
# Data comes in on the Serial port. Set that up and start it. 

    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyS0')  #serial port to alamode
    ser.baudrate = 115200 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. 

    ser.flushInput()
    ser.readline()
    # Initialize variables
    tstart = rospy.get_time()


    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    cnt_line = 0
    while not rospy.is_shutdown():
        newa0 = 0
        newa1 = 0
        newa2 = 0
        newa3 = 0
        newa4 = 0
        newa5 = 0
        newu0 = 0
        newu1 = 0
        newu2 = 0
        newe0 = 0
        newe1 = 0
        new_data_packet = 0
        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            line = line.split(":")
            # Element 0 of "line" will be a string that says what the data are: 
            data_type = line[0]
            # Element 1 of "line" will be the value, an Integer
            data_value = int(line[1])
#            print(data_type)
#            print(line)
            if data_type == 'A0':
                msg_sensors_raw.a0 = data_value    # Analog reading 
#                pub_A0.publish(msg_A0)
            elif data_type == 'A1':
                msg_sensors_raw.a1 = data_value    # Analog reading 
#                pub_A1.publish(msg_A1)
            elif data_type == 'A2':
                msg_sensors_raw.a2 = data_value    # Analog reading 
#                pub_A2.publish(msg_A2)
            elif data_type == 'A3':
                msg_sensors_raw.a3 = data_value    # Analog reading 
#                pub_A3.publish(msg_A3)
            elif data_type == 'A4':
                msg_sensors_raw.a4 = data_value    # Analog reading 
#                pub_A4.publish(msg_A4)
            elif data_type == 'A5':
                msg_sensors_raw.a5 = data_value    # Analog reading 
#                pub_A5.publish(msg_A5)
            elif data_type == 'U0':
                msg_sensors_raw.u0 = data_value    # Ultrasonic Sensor reading 
#                pub_U0.publish(msg_U0)
            elif data_type == 'U1':
                msg_sensors_raw.u1 = data_value    # Analog reading 
#                pub_U1.publish(msg_U1)
            elif data_type == 'U2':
                msg_sensors_raw.u2 = data_value    # Analog reading 
#                pub_U2.publish(msg_U2)
            elif data_type == 'E0':    #only use it as an encoder0 reading if it is one. 
                msg_sensors_raw.e0 = data_value    # Here is the actual encoder reading. 
#                pub_E0.publish(msg_E0)
            elif data_type == 'E1':    #only use it as an encoder1 reading if it is one. 
                msg_sensors_raw.e1 = data_value    # Here is the actual encoder reading. 
                newe1 = 1
#                pub_E1.publish(msg_E1)
            
            if newe1 :
                newe1 = 0
                pub_sensors_raw.publish(msg_sensors_raw)

        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
