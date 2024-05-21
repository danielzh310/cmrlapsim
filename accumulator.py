import numpy as np

from proto.vehicle_config_pb2 import AccumulatorConfig, CellType
from cell import cellVTC6, pouchCell52

cellRegistry = {
	CellType.VTC6: cellVTC6,
	CellType.PouchCell52: pouchCell52
}

class Accumulator():
	def __init__(self, config: AccumulatorConfig):

		assert config.cellType in cellRegistry.keys()
		self.cell = cellRegistry[config.cellType](SoC=config.stateOfCharge)
		self.s = config.nSeries
		self.p = config.nParallel
		self.n = self.s*self.p
		self.SoC = self.cell.SoC
		self.maxVoltage = 600 # rules :'(

		self.heatLoss = 0
		self.voltage = self.getVoltage(0)
		self.power = 0
		self.current = self.power/self.voltage

		self.recovered = 0
		self.discharged = 0

		self.prevVoltage = self.getVoltage(0)

	def getVoltage(self, power):
		return self.s*self.cell.getVoltage(power/self.n)

	def getHeat(self, power):
		return self.n*self.cell.getHeat(power/self.n)

	def getMaxRegenPower(self):
		maxVoltage = min(self.s*self.cell.maxVoltage, self.maxVoltage)
		powers = np.arange(0,-200e3,-1e2) # Join the battle against premature code optimization

		voltages = self.getVoltage(powers)

		return -1*powers[np.argmax(voltages > maxVoltage)]

	def getMaxDischargePower(self):
		minVoltage = self.s*self.cell.minVoltage
		powers = np.arange(0, 200e3, 1e2)

		voltages = self.getVoltage(powers)

		maxPower = powers[np.argmax(np.nan_to_num(voltages) < minVoltage)-1]
		# print('')
		# print(powers)
		# print(voltages)
		# print(maxPower)
		# print(voltages < minVoltage)
		# print(np.argmax(voltages < minVoltage))
		return maxPower

	def discharge(self, power, dt):
		self.discharged += power*dt
		self.heatLoss += self.getHeat(power)*dt

		self.power = power
		self.prevVoltage = self.voltage
		self.voltage = self.getVoltage(self.power)
		self.cell.discharge(power/self.n, dt)
		self.SoC = self.cell.SoC
		self.current = self.power/self.voltage
	

	def charge(self, power, dt):
		self.cell.charge(power/self.n, dt)

		self.recovered += (self.cell.SoC - self.SoC) * self.n * self.cell.capacity
		self.SoC = self.cell.SoC

		self.heatLoss += self.getHeat(power)*dt
		self.power = -power
		self.voltage = self.getVoltage(-power)
		self.current = self.power/self.voltage

	def getEnergy(self):
		return self.n*self.SoC*self.cell.capacity
