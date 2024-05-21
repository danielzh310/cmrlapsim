import numpy as np

class _Cell():
	def getVoltage(self, power):
		# calculate loaded voltage
		unloaded = self.getUnloadedVoltage()
		return 1/2*(unloaded + np.sqrt(unloaded**2-4*power*self.getDCIR()))

	def getHeat(self, power):
		current = power/self.getVoltage(power)
		return current**2 * self.getDCIR()

	def discharge(self, power, dt):
		self.SoC -= (power + self.getHeat(power))*dt/self.capacity

	def charge(self, power, dt):
		self.SoC += (power - self.getHeat(power))*dt/self.capacity

	def joulesToAHr(self, joules):
		return joules/self.nominal/3600

	def aHrToJoules(self, aHr):
		return aHr*self.nominal*3600 

class cellVTC6(_Cell):
	def __init__(self, SoC=1.0, cycCount=10):
		assert(0 <= SoC <= 1)
		self.SoC = SoC
		self.cycCount = cycCount
		self.nominal = 3.6
		# If 0 to 2 cycles have been done, capacity is 2.6858 Ah
		# If 3 to 7 cycles have been done, capacity is 2.6257 Ah
		# If 7 or more cycles have been done, capacity is 2.5828 Ah
		self.capacity = self.aHrToJoules(2.6858 if cycCount <= 2 else 2.6257 if cycCount <= 7 else 2.5828)
		self.maxVoltage = 4.2
		self.minVoltage = 2.8


		# Create lookup table
		# If 0 to 2 cycles have been done, use cycle 0 lookup table
		# If 3 to 7 cycles have been done, use cycle 5 lookup table
		# If 7 or more cycles have been done, use cycle 10 lookup table
		lookupFile = 'Data/vtc6_cap_cyc0.csv' if cycCount <= 2 else 'Data/vtc6_cap_cyc5.csv' if cycCount <= 7 else 'Data/vtc6_cap_cyc10.csv'
		# Get lookup values
		data = np.genfromtxt(lookupFile, delimiter=',')
		mask = (np.nan_to_num(data) != 0).any(axis=1)
		data = data[mask]
		self.data = data[5:,:]
		self.timeArr = self.data[:,0]
		self.voltageArr = self.data[:,1] + 9 * self.getDCIR()

	def getDCIR(self):
		# If 0 to 2 cycles have been done, DCIR is 12.99082 mOhm
		# If 3 to 7 cycles have been done, DCIR is 15.78273 mOhm
		# If 7 or more cycles have been done, DCIR is 15.78273 mOhm TODO: Update value after cyc10 DCIR data available
		return 0.01299082 if self.cycCount <= 2 else 0.01578273 if self.cycCount <= 7 else 0.01578273
	
	def getUnloadedVoltage(self):
		# Uses lookup table
		
		# Scale SoC to time for a CC load
		time = (1 - self.SoC) * (self.timeArr[-1] - self.timeArr[0]) + self.timeArr[0]
		index = max(np.searchsorted(self.timeArr, time)-1,0)

		# Capacity test done at 9A CC
		return self.voltageArr[index]


class cell30Q(_Cell):
	def __init__(self, SoC=1.0):
		assert(0 <= SoC <= 1)
		self.SoC = SoC
		self.nominal = 3.6
		self.capacity = self.aHrToJoules(3.0)
		self.maxVoltage = 4.2
		self.minVoltage = 3.0

	def getDCIR(self):
		# TODO: Add temperature and SoC dependence
		return 0.013

	def getUnloadedVoltage(self):
		# calculate unloaded voltage from SoC
		v = 4.167 - (1-self.SoC)*0.35*self.joulesToAHr(self.capacity)
		if self.SoC < 0.1:
			v -= (0.1-self.SoC)*0.5
		return v

class pouchCell52(_Cell):
    def __init__(self, SoC=1.0):
        assert(0 <= SoC <= 1)
        self.SoC = SoC
        self.nominal = 3.7
        self.capacity = self.aHrToJoules(7.0)
        self.maxVoltage = 4.2
        self.minVoltage = 3.0
    def getDCIR(self):
        # TODO: Add temperature and SoC dependence
        return 0.0013
    def getUnloadedVoltage(self):
        # calculate unloaded voltage from SoC
        v = 4.167 - (1-self.SoC)*0.35*self.joulesToAHr(self.capacity)
        if self.SoC < 0.1:
            v -= (0.1-self.SoC)*0.5
        return v

