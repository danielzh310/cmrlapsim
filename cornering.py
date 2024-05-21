import matplotlib.pyplot as plt
import numpy as np
from numpy import (arange, arccos, arcsin, arctan, array, cos, exp, ones, sign,
                   sin, sqrt, std, tan, zeros)


def getLatAccel(car, swangle, v, yaw, drs, plot=False):
	tireAngles = car.getTireAngles(swangle)
	
	yaws = []
	a_xs = []
	a_ys = []
	yawMoments = []

	a_x = a_y = w = 0
	yawMoment = 0
	prevFz = car.getFz(a_x, a_y, v, drs)
	maxFzDiff = 100*ones(4)
	a_lat = .001 # No ZeroDivisionError
	r = 0
	yawdot = 0
	for i in range(int(1e4)):
		yaws.append(yaw)
		a_xs.append(a_x)
		a_ys.append(a_y)
		yawMoments.append(yawMoment)

		xmask = array([1, -1, 1, -1])
		ymask = array([-1, -1, 1, 1])

		y = car.wheelBase/2
		x = car.trackWidth/2
		vx = v*cos(yaw)*np.ones(4) + x*w*xmask
		vy = v*sin(yaw)*np.ones(4) + y*w*ymask

		if (not np.all(vx > 0)):
			###/print('failed on vx > 0')
			return None
		#assert(np.all(vx > 0))


		# print(w, r, np.degrees(arctan(vy/vx)))
		slipAngles = tireAngles + arctan(vy/vx)

		Fz = car.getFz(a_x, a_y, v, drs)
		Fz = np.clip(Fz, prevFz-maxFzDiff, prevFz+maxFzDiff)
		cambers = car.getCambers(a_x, a_y)

		pars = (Fz, slipAngles, 0, cambers, 12)
		Fx = car.tire.getFx(*pars)*0
		Fy = car.tire.getFy(*pars)
		Mz = car.tire.getMz(*pars)*0

		F_carx = Fx*cos(tireAngles) + Fy*sin(tireAngles)
		F_cary = Fx*sin(tireAngles) + Fy*cos(tireAngles) 

		r = car.massProperties.rearMassFraction
		f = 1-r
		momentArmsFracs = array([f, f, r, r])
		yawMoment = sum(Mz + xmask*F_carx*car.trackWidth/2 + \
			            ymask*F_cary*car.wheelBase*momentArmsFracs)
		#print(yawMoment)

		b = 1e4
		dt = 0.003
		yawdot += (yawMoment/car.massProperties.yawInertia - sign(yawdot)*yawdot**2*b) * dt
		yaw += yawdot * dt #sign(dyaw)*dyaw**2.0

		# if i > 5e1:
		# 	# plt.plot(a_ys)
		# 	# plt.show()
		# 	break

		a_x = 0 #sum(F_carx)/car.m
		a_y = sum(F_cary)/car.m

		prevALat = a_lat
		maxDiff = 0.03
		a_lat = sum(F_cary*cos(yaw) + F_carx*sin(yaw))/car.m
		a_lat = np.clip(a_lat, prevALat - maxDiff, prevALat + maxDiff)
		r = v**2/a_lat
		w = sqrt(a_lat/r)

		prevFz = Fz
		#print(sum(F_cary))

		if (std(yaws[-300:]) < 1e-3 and i > 300):
			###/print('Yaw Moment = {} Nm'.format(yawMoment))
			###/print('Converged in {} iterations'.format(i))
			###/print('Lat accel {} '.format(a_lat))
			break
	else:
		###/print('failed')
		###/print(yawMoments[-10:])
		#if np.std(yaws[-500:]) < 0.25 and False:
		#	pass
		#else:
		#	print(swangle, v, yaw)
		#	plt.plot(np.degrees(yaws))
		#	plt.show()
		return None
			

	if plot:

		print(np.degrees(slipAngles))

		print('here')
		print(len(yaws))

		plt.plot(np.degrees(yaws))
		plt.show()

		print('also here')

		x = car.wheelBase/2 * array([1, 1, -1, -1])
		y = car.trackWidth/2 * array([1, -1, -1, 1])

		print('Max Tire Load = {} N'.format(round(max(Fz))))

		plt.xlim(-2, 2)
		plt.ylim(-2, 2)

		for i in range(4):
			plt.arrow(x[i], y[i], F_carx[i]/2000, F_cary[i]/2000, width=.01)


		vx = v/abs(v)*cos(yaw)
		vy = v/abs(v)*sin(yaw)

		plt.arrow(0, 0, vx, vy, width=.02)

		plt.show()
	return(yaw, a_lat, slipAngles, yawMoment, Fz, v)
