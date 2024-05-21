import copy
from proto.policy_config_pb2 import DrsPolicy


class Policy():
	def __init__(self, config):
		self.accelPower = config.maxDischargePower
		self.regenPower = config.maxRegenPower
		self.dt = config.dt
		self.drsPolicy = config.drsPolicy

		self.velocities = []
		self.accels = []
		self.positions = []
		self.currents = []
		self.voltages = []
		self.powers = []
		self.SoCs = []
		self.states = []
		self.frontMotorTorques = [0.0]
		self.rearMotorTorques = [0.0]
		self.motorHeats = []
		self.inverterHeats = []

	def accel(self, car):
		car.accelerate(self.accelPower, self.dt, self.drsPolicy)

	def brake(self, car):
		car.brake(self.regenPower, self.dt, self.drsPolicy)

	def coast(self, car):
		car.accelerate(1, self.dt, self.drsPolicy)

class Controller():
	def __init__(self, car, policy):
		self.car = car
		self.policy = policy

	def straight(self, distance, nextCornerRadius, lats, rads, drs_lats, drs_rad):
		target = self.car.state.position + distance
		
		v_f, lat = self.car.getCornerSpeed(nextCornerRadius, lats, rads)
		v_f_drs, lat_drs = self.car.getCornerSpeed(nextCornerRadius, drs_lats, drs_rad)

		
		if (self.policy.drsPolicy == DrsPolicy.DYNAMIC_1 or self.policy.drsPolicy == DrsPolicy.DYNAMIC_2):
			v_f = max(v_f, v_f_drs)		
		elif (self.policy.drsPolicy == DrsPolicy.OPEN):
			v_f = v_f_drs
		
		#print('Corner speed: {}'.format(v_f))
		brakePoint = self.findBrakePoint(v_f, distance)

		while self.car.state.position < brakePoint:
			self.policy.accel(self.car)
		while self.car.state.position < target:
			if self.car.state.velocity > v_f:
				self.policy.brake(self.car)
			else: # Due to brakepoint search tolerance
				self.policy.coast(self.car)

	def corner(self, cornerRadius, cornerAngle, lats, rads, drs_lats, drs_rad):
		self.car.corner(cornerRadius, cornerAngle, self.policy.dt, lats, rads, drs_lats, drs_rad, self.policy.drsPolicy)

	def getBrakePointError(self, car, policy, brakePoint, target, v_f):
		assert(brakePoint < target)

		while car.state.position < brakePoint:
			policy.accel(car)
		while car.state.velocity > v_f:
			policy.brake(car)
			assert(car.state.velocity > 0)

		return target - car.state.position


	def findBrakePoint(self, v_f, distance):
		v_i = self.car.state.velocity

		target = self.car.state.position + distance
		
		car2 = copy.deepcopy(self.car)

		# Check if there exists a solution at all
		if v_i < v_f:
			while car2.state.position < target:
				self.policy.accel(car2)
            # Car can't reach corner speed (or it's the final straight)
			if car2.state.velocity < v_f: 
				return target

		else:
			while (car2.state.position < target) and (car2.state.velocity > v_f):
				self.policy.brake(car2)
			if car2.state.velocity > v_f: # R I P
				print('Car entered straight too fast')
				return self.car.state.position

		lo = self.car.state.position
		hi = distance + self.car.state.position
		brakePoint = (lo + hi)/2

		error = 1e9
		count = 0
		# print("Find break point \n")
		while True:
			car2 = copy.copy(self.car)
			error = self.getBrakePointError(car2, self.policy, brakePoint, target, v_f)
			# print(error)
			if 0 < error < 2.0:
				break
			elif error > 0:
				lo = brakePoint
				brakePoint = (lo + hi)/2
			else:
				hi = brakePoint
				brakePoint = (lo + hi)/2

			count += 1
			# assert(count < 100) # timeout
			if (count > 100):
				print("Get Brake Point Error: ", error)
				break
		return brakePoint