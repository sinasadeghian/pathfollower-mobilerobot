class robot_PID():
        
    def __init__(self, error,K_p,K_d):
        self.K_p = K_p
        self.K_d = K_d
        self.error = error
        self.prev_error = 0
        self.PID_val = 0
        
    def PIDcontrol(self):
        P = self.error
        D = self.error - self.prev_error
        self.PID_val = self.K_p * P + self.K_d * D 
        self.prev_error = self.error
