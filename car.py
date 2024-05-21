from copy import deepcopy
import numpy as np
from numpy import array

from google.protobuf import text_format
from proto.state_pb2 import ControlMode, VehicleState, VehicleStateLog
from proto.vehicle_config_pb2 import VehicleConfig
from proto.policy_config_pb2 import DrsPolicy

from cornering import getLatAccel
from tire import Tire
from powertrain import Powertrain
from utils import getEffectiveLinearMass
from numpy import arange

G = 9.8
g = 9.8
LAT_THRESHOLD = 2
RHO = 1.225 #kg / m^3 == air density constant

class Car():

	def __init__(self, config):

		self.state = VehicleState()
		self.log = VehicleStateLog()

		self.powertrain = Powertrain(config.powertrainConfig)
		self.tire = self.powertrain.tire

		self.m = config.massProperties.vehicleMass + config.massProperties.driverMass
		self.wheelBase = config.suspensionConfig.wheelBase
		self.trackWidth = config.suspensionConfig.trackWidth
		self.cgHeight = config.massProperties.cgHeight

		self.aeroConfig = config.aeroConfig
		self.aeroConfig_drs = config.aeroConfig_drs
		self.massProperties = config.massProperties
		self.suspensionConfig = config.suspensionConfig

		self.document(ControlMode.COASTING)


	# Custom copy constructor to avoid expensive log duplication
	# Copy is "semi-deep"
	def __copy__(self):
		result = Car.__new__(Car)
		memo = {id(self): result}
		for k, v in self.__dict__.items():
			if isinstance(v, VehicleStateLog):
				setattr(result, k, VehicleStateLog())
			else:
				setattr(result, k, deepcopy(v, memo))
		return result

	# calculateDrs and determineDrs functions were here. See PR #12 ("Cleaned up unused DRS functions")

	# sets drs state
	def setDrsState(self, state):
		self.state.drs = state 

	def accelerate(self, maxPower, dt, drsPolicy):
		##self.calculateDrs(maxPower)
		# self.setDrsState(self.determineDrs(self, self.accel, 0))

		if (drsPolicy == DrsPolicy.OPEN or drsPolicy == DrsPolicy.DYNAMIC_1 or drsPolicy == DrsPolicy.DYNAMIC_2):
			self.setDrsState(True)
		else:
			self.setDrsState(False)

		drag = self.state.dragForce = self.getDrag(self.state.velocity)
		downForce = self.state.downForce = self.getDownForce(self.state.velocity)

		loadTransfer = (self.state.accel*self.m*self.cgHeight)/self.wheelBase

		r_m = self.massProperties.rearMassFraction
		r_a = self.aeroConfig.rearAeroFraction
		if self.state.drs == True:
			r_a = self.aeroConfig_drs.rearAeroFraction
		frontTireLoad = (self.m * g /2 * (1-r_m)) - loadTransfer/2 + downForce * (1-r_a)/2
		rearTireLoad = (self.m * g / 2 * r_m) + (loadTransfer / 2) + (downForce * r_a/2)

		frontTorque, rearTorque, frontMotorTorque, rearMotorTorque = self.powertrain.accelerate(self.state.velocity, maxPower, frontTireLoad, rearTireLoad, dt)
		assert(abs(frontMotorTorque) < 21.5 and abs(rearMotorTorque) < 21.5)
		self.state.frontMotorTorque = frontMotorTorque
		self.state.rearMotorTorque = rearMotorTorque

		force = 2*(frontTorque + rearTorque)/self.powertrain.tire.radius - drag
		self.state.accel = force / getEffectiveLinearMass(self)

		self.state.position += self.state.velocity*dt + 1/2*self.state.accel*dt**2
		self.state.velocity += self.state.accel*dt

		self.state.timestamp += dt

		self.document(ControlMode.COASTING if maxPower < 1e3 else ControlMode.ACCELERATING)

	def brake(self, maxPower, dt, drsPolicy):

		#self.setDrsState(self.determineDrs(self, self.accel, 0))

		if (drsPolicy == DrsPolicy.OPEN or drsPolicy == DrsPolicy.DYNAMIC_1):
			self.setDrsState(True)
		else:
			self.setDrsState(False)

		drag = self.state.dragForce = self.getDrag(self.state.velocity)
		downForce = self.state.downForce = self.getDownForce(self.state.velocity)

		loadTransfer = (self.state.accel*self.m*self.cgHeight)/self.wheelBase

		r_m = self.massProperties.rearMassFraction
		r_a = self.aeroConfig.rearAeroFraction
		if self.state.drs == True:
			r_a = self.aeroConfig_drs.rearAeroFraction
		frontTireLoad = (self.m * g /2 * (1-r_m)) - loadTransfer/2 + downForce * (1-r_a)/2
		rearTireLoad = (self.m * g / 2 * r_m) + (loadTransfer / 2) + (downForce * r_a/2)

		minRegenFrac = 1
		frontTorque, rearTorque, self.state.frontMotorTorque, self.state.rearMotorTorque = self.powertrain.brake(self.state.velocity, minRegenFrac, maxPower, frontTireLoad, rearTireLoad, dt)

		force = 2*(frontTorque + rearTorque)/self.powertrain.tire.radius - drag
		self.state.accel = force / getEffectiveLinearMass(self)

		self.state.position += self.state.velocity*dt + 1/2*self.state.accel*dt**2
		self.state.velocity += self.state.accel*dt

		self.state.timestamp  += dt

		self.document(ControlMode.BRAKING)

	#find highest lat within threshold of radius, otherwise use closest
	def getCornerSpeed(self, radius, lats, rads):
		latsCopy = np.array(lats, copy=True)
		distance = np.abs(np.array(rads) - radius)
		finalLats = np.where(distance<LAT_THRESHOLD, latsCopy, 0)
		speeds = np.sqrt(np.abs(finalLats * radius))
		speedsAll = np.sqrt(np.abs(latsCopy * radius))
		v = np.max(speeds)
		lat = np.abs(np.max(finalLats))
		if (v < 1):
			# From Nico himself:
			#   velocity shouldn't be below 1 for whatever reason, so try this heuristic, if it fixes it great,
			#   if it doesn't, try the next thing
			v = speedsAll[np.argmin(distance)]
			# print("nearest")
			if (v < 1):
				# print("heuristic")
				v = np.sqrt(1.2 * g * radius)
		return v, lat
		

	def corner(self, radius, angle, dt, lats, rads, drs_lats, drs_rad, drsPolicy):
		v, lat = self.getCornerSpeed(radius, lats, rads)
		v_drs, lat_drs = self.getCornerSpeed(radius, drs_lats, drs_rad)

		if (drsPolicy == DrsPolicy.DYNAMIC_1 or drsPolicy == DrsPolicy.DYNAMIC_2):
			if (v_drs > v): 
				self.setDrsState(True)
				v = v_drs
			else: 
				self.setDrsState(False)
		elif (drsPolicy == DrsPolicy.OPEN):
			self.setDrsState(True)
		else:
			self.setDrsState(False)

		#if track starts with a corner, assume instant acceleration to corner speed
		if (self.state.velocity < 1):
			self.state.velocity = v
		
		self.state.velocity = min(self.state.velocity, v)
		self.state.accel = 0.0
		t = radius*angle/self.state.velocity
		self.state.dragForce = self.getDrag(self.state.velocity)
		self.state.downForce = self.getDownForce(self.state.velocity)

		# From 19e 2019-05-08 multi-radius skidpad testing
		# Alternative would be to use aerodynamic and tire drag. Tire drag 
		# would require some work.
		# Could precompute, but it's a few ms
		experimentalRadii = [15.125, 12.125, 9.125]
		experimentalPowers = [6.21, 5.95, 5.21]
		r = 1 # polynomial order
		coeffs = np.polyfit(experimentalRadii, experimentalPowers, r)
		powerFct = lambda x: np.sum(coeffs[i]*x**(r-i) for i in range(r+1))

		power = powerFct(radius)*1000
		w = self.state.velocity/self.powertrain.tire.radius
		self.state.frontMotorTorque = power/w/4/self.powertrain.frontRatio
		self.state.rearMotorTorque = power/w/4/self.powertrain.rearRatio

		for i in range(int(t//dt)):
			self.state.timestamp  += dt
			self.state.position += self.state.velocity*dt
			self.powertrain.accumulator.discharge(power, dt)
			self.document(ControlMode.CORNERING)

	def getDownForce(self, v):
		if self.state.drs == True:
			return RHO * self.aeroConfig_drs.AC_l * v**2 / 2
		return RHO * self.aeroConfig.AC_l * v**2 / 2

	def getDrag(self, v):
		if self.state.drs == True:
			return RHO * self.aeroConfig_drs.AC_d * v**2 / 2
		return RHO * self.aeroConfig.AC_d * v**2 / 2

	def getDownForce_drs(self, v, drs):
		if drs == True:
			return RHO * self.aeroConfig_drs.AC_l * v**2 / 2
		return RHO * self.aeroConfig.AC_l * v**2 / 2

	def getDrag_drs(self, v, drs):
		if drs == True:
			return RHO * self.aeroConfig_drs.AC_d * v**2 / 2
		return RHO * self.aeroConfig.AC_d * v**2 / 2

	def getDragHeight_drs(self, drs):
		if drs == True:
			return self.aeroConfig_drs.dragHeight
		return self.aeroConfig.dragHeight
	
	def getRearAeroFraction_drs(self, drs):
		if drs == True:
			return self.aeroConfig_drs.rearAeroFraction
		return self.aeroConfig.rearAeroFraction

	def getFz(self, a_x, a_y, v, drs):
		# TODO: More sophisticated treatment of aero

		r = self.massProperties.rearMassFraction
		f = 1 - r
		Fz = self.m*g/2 * array([f, f, r, r])

		r = self.getRearAeroFraction_drs(drs)
		f = 1 - r
		Fz += self.getDownForce_drs(v, drs)/2 * array([f, f, r, r])

		longTransfer = self.cgHeight*self.m*a_x / self.wheelBase
		Fz += longTransfer/2 * array([-1, -1, 1, 1])

		latTransfer = self.cgHeight*self.m*a_y  /self.trackWidth
		r = self.suspensionConfig.rearRollStiffnessFraction
		f = 1 - r
		Fz += latTransfer * array([-f, f, r, -r])

		dragTransfer = self.getDragHeight_drs(drs)*self.getDrag_drs(v, drs)/self.wheelBase
		Fz += dragTransfer/2 * array([-1, -1, 1, 1])

		return Fz

	def getCambers(self, a_x, a_y):
		return np.zeros(4)

	# Return tire angles relative to x direction
	def getTireAngles(self, swangle):
		# TODO: Make somewhat real

		return swangle * array([1,1,0,0])

	def setSegmentNumber(self, number):
		self.state.segmentNumber = number
	
	def setSegmentType(self, type):
		self.state.segmentType = type

	def document(self, state):
		self.state.current = self.powertrain.accumulator.current
		self.state.voltage = self.powertrain.accumulator.voltage
		self.state.power = self.powertrain.accumulator.power
		self.state.stateOfCharge = self.powertrain.accumulator.SoC
		self.state.motorHeatTotal = self.powertrain.motorHeatLoad
		self.state.inverterHeat = self.powertrain.inverterHeatLoad
		self.state.controlMode = state

		self.log.vehicleState.append(self.state)

	def saveLog(self, filename):
		# print(self.log)
		with open(filename, "w") as f:
			f.write(text_format.MessageToString(self.log))

	def saveLogAsCSV(self, filename):
		state_types = VehicleState.DESCRIPTOR.fields_by_name.keys()
		with open(filename, "w") as f:
			line = ",".join(state_types) + "\n"
			print(line)
			f.write(line)
			for state in self.log.vehicleState:
				line = ""
				for state_type in  state_types:
					line = line + str(getattr(state, state_type)) + ","
				line = line + '\n'
				f.write(line)

	@staticmethod
	def rpmToRadPerSec(rpm):
		return np.pi/30*rpm

	

