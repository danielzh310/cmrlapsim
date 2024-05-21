import copy
import numpy as np

from proto.vehicle_config_pb2 import PowertrainConfig, MotorType

from accumulator import Accumulator
from motor import motorRegistry
from tire import Tire

class Powertrain():
	def __init__(self, config: PowertrainConfig):
		self.accumulator = Accumulator(config.accumulatorConfig)

		assert config.motorType in motorRegistry.keys()
		self.frontMotor = motorRegistry[config.motorType]()
		self.rearMotor = motorRegistry[config.motorType]()
		self.frontRatio = config.frontRatio
		self.rearRatio = config.rearRatio

		self.tire = Tire(config.tireConfig)
		self.inverterEff = config.inverterEff
		self.brakeBias = 2

		self.power = 0

		# Instantaneous heat loads, W
		self.motorHeatLoad = 0
		self.inverterHeatLoad = 0
		self.brakeHeatLoad = 0

		# Cumulative energy losses, J
		self.motorHeatLoss = 0
		self.inverterHeatLoss = 0
		self.brakeHeatLoss = 0

	def calculateTractionLimits(self, frontNormal, rearNormal):
		maxLongFront = self.tire.getMaxLongForce(frontNormal)
		maxLongRear = self.tire.getMaxLongForce(rearNormal)

		# should account for tire inertia too, but that's a little tricky since
		# the vehicle acceleration is unknown at this point
		# Probably should just use the prev value to add rot inertia contribution?

		maxFront = maxLongFront*self.tire.radius/self.frontRatio
		maxRear = maxLongRear*self.tire.radius/self.rearRatio

		return maxFront, maxRear

	def determineDrsState(self, v, maxPower, frontNormal_noDRS, rearNormal_noDRS):

		# Taking mean eliminates some numerical instability from bouncing in and out of field weakening
		voltage = (self.accumulator.voltage + self.accumulator.prevVoltage)/2

		frontRpm = self.frontMotor.radPerSecToRpm(v/self.tire.radius*self.frontRatio)
		rearRpm = self.rearMotor.radPerSecToRpm(v/self.tire.radius*self.rearRatio)

		#calculate max traction
		frontMax, rearMax = self.calculateTractionLimits(frontNormal_noDRS, rearNormal_noDRS)

		front_max_torque = self.frontMotor.getMaxTorque(frontRpm, voltage)
		rear_max_torque = self.rearMotor.getMaxTorque(rearRpm, voltage)

		if (front_max_torque > frontMax or rear_max_torque > rearMax):
			# disable drs since more traction is needed
			return False
		# enable drs since extra traction isn't needed
		return True

	def accelerate(self, v, maxPower, frontNormal, rearNormal, dt):

		maxPower = min(maxPower, self.accumulator.getMaxDischargePower())
		# print("Min Power: ", maxPower, " ")
		# maxPower = min()

		# Taking mean eliminates some numerical instability from bouncing in and out of field weakening
		voltage = (self.accumulator.voltage + self.accumulator.prevVoltage)/2

		frontRpm = self.frontMotor.radPerSecToRpm(v/self.tire.radius*self.frontRatio)
		rearRpm = self.rearMotor.radPerSecToRpm(v/self.tire.radius*self.rearRatio)

		frontMax, rearMax = self.calculateTractionLimits(frontNormal, rearNormal)

		frontMax = min(frontMax, self.frontMotor.getMaxTorque(frontRpm, voltage))
		rearMax = min(rearMax, self.rearMotor.getMaxTorque(rearRpm, voltage))

		frontEffFct = self.frontMotor.getEffFunction(frontRpm)
		rearEffFct = self.rearMotor.getEffFunction(rearRpm)

		wFront = self.frontMotor.rpmToRadPerSec(frontRpm)
		wRear = self.rearMotor.rpmToRadPerSec(rearRpm)

		powerTol = 100 # Watts

		hi = 1.0
		lo = 0.0
		tracCapFrac = (lo + hi)/2 # Fraction of tractive capacity

		count = 0
		while True:

			frontTorque = tracCapFrac * frontMax
			rearTorque = tracCapFrac * rearMax

			frontPower = frontTorque*wFront/frontEffFct(frontTorque)/self.inverterEff
			rearPower = rearTorque*wRear/rearEffFct(rearTorque)/self.inverterEff

			power = 2*(frontPower + rearPower)

			if (0 < maxPower-power < powerTol) or abs(lo-hi) < 0.001:
				break
			elif power > maxPower:
				hi = tracCapFrac
			else:
				lo = tracCapFrac
			tracCapFrac = (lo + hi)/2 

			count += 1 
			assert count < 1000 # timeout
		
		frontEff = frontEffFct(frontTorque)
		rearEff = rearEffFct(rearTorque)

		self.power = 2*(frontPower + rearPower)  # Electrical power out of pack

		self.accumulator.discharge(self.power, dt)
		self.inverterHeatLoad = self.power * (1-self.inverterEff)
		self.motorHeatLoad = 2*(frontTorque*wFront*(1-frontEff)/frontEff + rearTorque*wRear*(1-rearEff)/rearEff)

		mechPower = 2*(frontTorque*wFront + rearTorque*wRear)
		assert(np.isclose(mechPower + self.inverterHeatLoad + self.motorHeatLoad, self.power))

		self.inverterHeatLoss += self.inverterHeatLoad * dt
		self.motorHeatLoss += self.motorHeatLoad * dt

		return frontTorque*self.frontRatio, rearTorque*self.rearRatio, frontTorque, rearTorque

	def brake(self, v, minRegen, maxPower, frontNormal, rearNormal, dt, regen=True):
		# 0 <= minRegen <= 1 is min fraction of tractive capacity that regen has
		# to achieve to be used exclusively. Otherwise, supplemented with mech brakes
		
		frontMax, rearMax = self.calculateTractionLimits(frontNormal, rearNormal)

		wFront = v/self.tire.radius*self.frontRatio
		wRear = v/self.tire.radius*self.rearRatio

		if regen:

			maxPower = min(maxPower, self.accumulator.getMaxRegenPower())

			voltage = self.accumulator.getVoltage(min(self.power,0)) # Min accounts for transition from accel to braking

			frontRpm = self.frontMotor.radPerSecToRpm(wFront)
			rearRpm = self.rearMotor.radPerSecToRpm(wRear)

			frontMotorMax = min(frontMax, self.frontMotor.maxTorque)
			rearMotorMax = min(rearMax, self.rearMotor.maxTorque)

			frontEffFct = self.frontMotor.getRegenEffFunction(frontRpm)
			rearEffFct = self.rearMotor.getRegenEffFunction(rearRpm)

			powerTol = 100 # Watts

			hi = 1.0
			lo = 0.0
			tracCapFrac = (lo + hi)/2 # Fraction of tractive capacity

			count = 0
			while True:

				frontTorque = tracCapFrac * frontMotorMax
				rearTorque = tracCapFrac * rearMotorMax

				frontPower = frontTorque*wFront*frontEffFct(frontTorque)*self.inverterEff
				rearPower = rearTorque*wRear*rearEffFct(rearTorque)*self.inverterEff

				power = 2*(frontPower + rearPower)

				if (0 < maxPower-power < powerTol) or abs(lo-hi) < 0.001:
					break
				elif power > maxPower:
					hi = tracCapFrac
				else:
					lo = tracCapFrac
				tracCapFrac = (lo + hi)/2 

				count += 1 
				assert count < 1000 # timeout
				assert(tracCapFrac <= 1.0)

			frontEff = frontEffFct(frontTorque)
			rearEff = rearEffFct(rearTorque)

			self.power = 2 * (frontPower + rearPower) # Electrical power into pack

			inverterHeatLoad = self.power*(1 - self.inverterEff)/self.inverterEff
			motorHeatLoad = 2*(frontTorque*wFront*(1-frontEff) + rearTorque*wRear*(1-rearEff))

			self.inverterHeatLoss += inverterHeatLoad * dt
			self.motorHeatLoss += motorHeatLoad * dt
			self.accumulator.charge(self.power, dt)

			mechPower = 2*(frontTorque*wFront + rearTorque*wRear)
			assert(np.isclose(mechPower - inverterHeatLoad - motorHeatLoad, self.power))

		else:
			frontTorque = 0
			rearTorque = 0

		frontMech = rearMech = 0

		if frontTorque + rearTorque < (frontMax + rearMax)*minRegen:
			# Supplement with mech brakes
			frontMech = frontMax - frontTorque
			rearMech = rearMax - rearTorque

			# Implement fixed mechanical brake bias
			frontMech = min(frontMech, rearMech*self.brakeBias)
			rearMech = min(rearMech, frontMech/self.brakeBias)

			self.brakeHeatLoad = 2*(frontMech*wFront + rearMech*wRear)
			self.brakeHeatLoss += self.brakeHeatLoad*dt


		return -(frontTorque+frontMech)*self.frontRatio, -(rearTorque+rearMech)*self.rearRatio, -frontTorque, -rearTorque
