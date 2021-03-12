import numpy as np

def sortKey(point):
    return point.r

def filterHorizon(points):
    filtered = []
    for point in points:
        r = np.sqrt(point.x*point.x + point.z*point.z)
        if(r < 2.0):
            point.r = r
            filtered.append(point)
    return filtered

def closestPoint(points, n=1):
    sortedPoints = sorted(points, key=sortKey)
    if(n == 1):
        return sortedPoints[0]
    else:
        return sortedPoints[:n]

def limit(val, lim):
    if(val < 0):
        if val < -1*lim:
            return -1*lim
        else:
            return val
    else:
        if val>lim:
            return lim
        else:
            return val

class PID():
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0