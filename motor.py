import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from proto.vehicle_config_pb2 import MotorType

class _Motor():
	def getHeat(self, rpm, torque):
		radPerSec = self.rpmToRadPerSec(rpm)
		mechPower = radPerSec*torque
		eff = self.getEfficiency(speed, torque)
		return mechPower*eff/(1-eff)

	@staticmethod
	def rpmToRadPerSec(rpm):
		return np.pi/30*rpm

	@staticmethod
	def radPerSecToRpm(radPerSec):
		return radPerSec*30/np.pi

class emraxMotor(_Motor):
	# Emrax 228 MV
	def __init__(self):
		pass

	def getMaxTorque(self, rpm, voltage):
		motorTorque = 240
		baseSpeed = 5170 * voltage/470
		topSpeed = 12000 * voltage/470  # TODO: 470 magic number - is this an old AC Max???
		if rpm > baseSpeed:
			motorTorque *= 1-(rpm-baseSpeed)/(topSpeed-baseSpeed)

		return motorTorque

	def getHeat(self, rpm, torque):
		eff = self.getEfficiency(rpm, torque)

	def getEfficiency(self, rpm, torque):
		# TODO: Use efficiency map
		return 0.92

class AmkMotor(_Motor):
	def __init__(self, speedLimit=False, effData='Data/amkefficiencymap.csv'):
		self.speedLimit = speedLimit
		self.efficiencyMap = self.importEfficiencyMap(effData)
		self.kT = 0.26
		self.ke = 18.8/1000*60/2/np.pi 
		self.R = 0.135
		self.maxTorque = 21
		self.r = 5 # efficiency polynomial order
		self.coeffsList = self.fitEfficiencyMap()

	def getMaxTorque(self, rpm, voltage): 
		motorTorque = 21
		baseSpeed = 16000 * abs(voltage)/600
		topSpeed = 24000 * abs(voltage)/600
		if rpm > baseSpeed:
			motorTorque *= 1-(rpm-baseSpeed)/(topSpeed-baseSpeed)

		if rpm > 20000 and self.speedLimit:
			return 0.1 # -10 # brake, b/c coasting ineffective if ratios differ
			#raise(Exception('Motor max speed exceeded'))

		return motorTorque*voltage/abs(voltage)

	def getEfficiency(self, rpm, torque):
		# Linearly interpolate efficiency

		if torque < 0:
			return 1
		rpm = min(abs(rpm),20000)
		torque = abs(torque)
		rpms = [0, 500] + [i*1000 for i in range(1,20)] + [20001]
		torques = [1.3, 2.7, 5.4, 7.9, 10.4, 12.5, 14.4, 16.0, 17.4, 18.5, 19.6, 21.01]

		try:
			lo = [rpms[i] <= rpm < rpms[i+1] for i in range(len(rpms)-1)].index(True)
			loSpeed, hiSpeed = rpms[lo:lo+2]

			lo = [torques[i] <= torque < torques[i+1] for i in range(len(torques)-1)].index(True)
			loTorque, hiTorque = torques[lo:lo+2]
		except:
			raise(Exception('Invalid Speed/Torque Requested'))

		speedWeight = 1-(rpm-loSpeed)/(hiSpeed-loSpeed)
		torqueWeight = 1-(torque-loTorque)/(hiTorque-loTorque)


		a = self.efficiencyMap.loc[loTorque,str(loSpeed)]
		b = self.efficiencyMap.loc[loTorque,str(hiSpeed)]
		c = self.efficiencyMap.loc[hiTorque,str(loSpeed)]
		d = self.efficiencyMap.loc[hiTorque,str(hiSpeed)]

		eff1 = a*speedWeight + b*(1-speedWeight)
		eff2 = c*speedWeight + d*(1-speedWeight)

		return (eff1*torqueWeight + eff2*(1-torqueWeight))/100

	def importEfficiencyMap(self, file):
		df = pd.read_csv(file)
		df.set_index('torque', inplace=True)
		
		return df

	def fitEfficiencyMap(self):
		rpms = ['0', '500'] + [str(i*1000) for i in range(1,20)] + ['20001']
		torques = np.array(self.efficiencyMap.index)

		coeffsList = []

		for rpm in rpms:
			effs = self.efficiencyMap.loc[:,rpm]
			coeffs = np.polyfit(torques, effs, self.r)
			coeffsList.append(coeffs)

		return coeffsList

	def getEffFunction(self, rpm):
		rpms = [0, 500] + [i*1000 for i in range(1,20)] + [20001]

		rpm = min(rpm, 20000)

		try:
			# TODO: Use np.searchsorted()
			lo = [rpms[i] <= rpm < rpms[i+1] for i in range(len(rpms)-1)].index(True)
			loSpeed, hiSpeed = rpms[lo:lo+2]
		except:
			print(rpm)
			raise(Exception('Invalid Speed/Torque Requested'))

		speedWeight = 1-(rpm-loSpeed)/(hiSpeed-loSpeed)
		coeffsLo, coeffsHi = self.coeffsList[lo:lo+2]
		coeffs = [(speedWeight*coeffsLo[i] + (1-speedWeight)*coeffsHi[i])/100 for i in range(len(coeffsLo))]
		
		poly = lambda x: np.sum(coeffs[i]*x**(self.r-i) for i in range(self.r+1))
		return poly

	def getRegenEffFunction(self, rpm):
		effFct = self.getEffFunction(rpm)

		w = self.rpmToRadPerSec(rpm)

		return lambda x: effFct(x) #* (1 - x*self.R/self.ke/self.kT/w)

motorRegistry = {
	MotorType.AMK: AmkMotor
}