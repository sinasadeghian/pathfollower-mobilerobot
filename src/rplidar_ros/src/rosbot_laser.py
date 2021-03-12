import rospy

from sensor_msgs.msg import LaserScan


class Rosbot():
    def __init__(self):
        
        self.rosbot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.l = 0.0 # left
        self.b = 0.0 # back
        self.r = 0.0 # right
        self.ctrl_c = False
        
        self.rate = rospy.Rate(10) #10 hz
        
        rospy.on_shutdown(self.shutdownhook)
        
    def scan_callback(self,msg):
        
        self.r = msg.ranges[len(msg.ranges)/4*3]
        self.b = msg.ranges[len(msg.ranges)/4*2]
        self.l = msg.ranges[len(msg.ranges)/4]

        
    def read_laser(self):
        
        while not self.ctrl_c:
            
            if self.l > 5:
                self.l = 5
                
            if self.b >5:
                self.b = 5
                
            if self.r > 5:
                self.r =5
            
            print ("b = " + str(self.b))# + " r = " + str(self.r) + " l = " + str(self.l))

    def shutdownhook(self):
        self.ctrl_c = True
    


        
if __name__ == '__main__':
    rospy.init_node('rosbot_test',anonymous=True) 
    rosbot = Rosbot() 
    try:
        rosbot.read_laser()
        
    except rospy.ROSInterruptException:
        traceback.print_exc()
        pass
    