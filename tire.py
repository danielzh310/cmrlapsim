import matplotlib.pyplot as plt
import numpy as np
from numpy import arange, arccos, arcsin, arctan, cos, exp, sign, sin, tan

from proto.vehicle_config_pb2 import TireConfig, TireType

# Note: All referenced equation & page numbers refer to Ch. 4 of Tire and Vehicle Dynamics, by Hans Pacejka

class Tire():

	def __init__(self, config: TireConfig):
		# TODO: Add handling for multiple tire types
		assert(config.tireType is TireType.R25B_18x7_5)

		self.longKnockdown = config.longKnockdown
		self.latKnockdown = config.latKnockdown
		self.radius = 0.0254 * 9
		self.circum = 2*np.pi*self.radius

		# Precompute maximum longitudinal forces for straight line driving
		self.normalLoads = arange(1, 3000, 4)
		SR = np.arange(0, 0.2, 0.005)
		self.Fxs = []
		for load in self.normalLoads:
			Fx = self.getFx(load, SA=0, SR=SR, camber=0, pressure=12)
			self.Fxs.append(np.max(Fx))

	def getRpm(self, velocity):
		return velocity/self.circum*60

	def getMaxLongForce(self, normalLoad):

		idx = np.searchsorted(self.normalLoads, normalLoad)-1
		return self.Fxs[idx]

	def getFx(self, normalLoad, SA=0, SR=0, camber=0, pressure=12):
		
		# Linear model of longitudinal force - leaving this here for now so I don't break everything
		#return (2.54 - (normalLoad-200*4.45)*.17/50/4.45) * normalLoad * self.knockdown

		Fz = normalLoad # Newtons

		gamma = np.radians(camber) # convert to radians
		gamma_star = sin(gamma) # 4.E4

		alpha = SA # radians
		alpha_star = tan(alpha) # E.E3

		kappa = SR # 4.E5 longitudinal slip ratio
		
		# User scaling factors

		# Pure slip
		lambda_Fzo = 1 # nominal (rated) load
		lambda_mux = self.longKnockdown # peak friction coefficient
		lambda_mV = 1 # with slip speed Vs decaying friction
		lambda_Kxk = 1 # brake slip stiffness
		lambda_Cx = 1 # shape factor
		lambda_Ex = 1 # curvature factor
		lambda_Hx = 0 # horizontal shift
		lambda_Vx = 0 # vertical shiftt
		lambda_Kyg = 1 # camber force stiffness
		lambda_Kzg = 1 # camber torque stiffness
		lambda_t = 1 # pneumatic trail (effecting aligning torque stiffness)
		lambda_Mr = 1 # residual torque

		# Combined slip
		lambda_xa = 1 # a influence on Fx(k)
		lambda_s = 1 # Mz moment arm of Fx

		# Other
		lambda_Cz = 1 # radial tire stiffness
		lambda_Mx = 1 # overturning couple stiffness
		lambda_VMx = 1 # overturning couple vertical shift
		lambda_My = 1 # rolling resistance moment

		Vs = abs(kappa * 20) # m/s  slip velocity - shouldn't matter as long as lambda_muV = 0
		Vo = 25 * 0.44704 # m/s  
		A_mu = 10 # 4.E8
		lambda_mux_star = lambda_mux/(1 + lambda_mV*Vs/Vo) # 4.E7
		lambda_mux_prime = A_mu*lambda_mux_star/(1 + (A_mu - 1)*lambda_mux) # 4.E8

		# Coefficients
		p_Cx1 = 1.255121165
		p_Dx1 = 2.994347749
		p_Dx2 = -0.388766188
		p_Dx3 = 0.006705784
		p_Ex1 = 1.43146928
		p_Ex2 = -17.80731096
		p_Ex3 = 1.34E+02
		p_Ex4 = 0.028057235
		p_Kx1 = 48.81651581
		p_Kx2 = -0.980949263
		p_Kx3 = 0.050522213
		p_px1 = -0.855704992
		p_px2 = -0.824970812
		p_px3 = -0.2008483
		p_px4 = 0.908398067
		r_Bx1 = 5.733213812
		r_Bx2 = 8.28262792
		r_Bx3 = 0.262200621
		r_Cx1 = 1.314818222
		r_Ex1 = 0.361634286
		r_Ex2 = -0.024145214
		r_Hx1 = -0.070279556

		# undetermined
		p_Hx1 = -1
		p_Hx2 = -1
		p_Vx1 = -1
		p_Vx2 = -1

		zeta_1 = 1

		eps_x = 0.001

		# Page 177:
		Fzo = 250 * 4.448 # nominal rated wheel load (N)
		Fzo_prime = lambda_Fzo * Fzo # for tire with different nominal rated load
		dfz = (Fz - Fzo_prime)/Fzo_prime

		p_i = pressure * 6.89476 # tire pressure (kPa)
		p_io = 12 * 6.89476 # nominal inflation pressure (kPa)
		dpi = (p_i - p_io)/p_io # normalized inflation pressure

		# Pure slip
		S_Hx = (p_Hx1 + p_Hx2*dfz)*lambda_Hx # 4.E15
		S_Vx = Fz * (p_Vx1 + p_Vx2*dfz)*lambda_Vx * lambda_mux_prime*zeta_1 # 4.E18
		K_xk = Fz*(p_Kx1 + p_Kx2*dfz)*exp(p_Kx3*dfz)*(1 + p_px1*dpi + p_px2*dpi**2) # 4.E15
		kappa_x = kappa + S_Hx # 4.E10
		Ex = (p_Ex1 + p_Ex2*dfz + p_Ex3*dfz**2)*(1 - p_Ex4*sign(kappa_x))*lambda_mux_star # 4.E14
		Ex = np.clip(Ex, None, 1)
		mu_x = (p_Dx1 + p_Dx2*dfz)*(1+p_px3*dpi + p_px4*dpi**2)*(1 - p_Dx3*gamma**2)*lambda_mux_star # 4.E13
		Dx = np.clip(mu_x * Fz * zeta_1, 0, None) # 4.E12
		Cx = np.clip(p_Cx1 * lambda_Cx, 0, None) # 4.E11
		Bx = K_xk/(Cx*Dx + eps_x) # 4.E16
		Fx0 = Dx * sin(Cx * arctan(Bx*kappa_x - Ex*(Bx*kappa_x - arctan(Bx*kappa_x)))) + S_Vx # 4.E9

		# Combined slip

		B_xa = (r_Bx1 + r_Bx3*gamma_star**2)*cos(arctan(r_Bx2*kappa))*lambda_xa # 4.E54
		B_xa = np.clip(B_xa, 0, None)
		C_xa = r_Cx1 # 4.E55
		E_xa = np.clip(r_Ex1 + r_Ex2*dfz, None, 1) # 4.E56
		S_Hxa = r_Hx1 # 4.E57
		alpha_S = alpha_star + S_Hxa # 4.E53
		G_xao = cos(C_xa*arctan(B_xa*S_Hxa - E_xa*(B_xa*S_Hxa - arctan(B_xa*S_Hxa)))) # 4.E52
		G_xa = cos(C_xa*arctan(B_xa*alpha_S - E_xa*(B_xa*alpha_S - arctan(B_xa*alpha_S))))/G_xao # 4.E51
		G_xa = np.clip(G_xa, 0, None)
		Fx = G_xa * Fx0 # 4.E50

		return Fx


	def getFy(self, normalLoad, SA, SR, camber, pressure, giveByCy=0, decay=False):
		if giveByCy:
			return self._getFy(normalLoad, SA, SR, camber, pressure, giveByCy=giveByCy, decay=decay)
		
		else:
			SA2 = -1*SA
			Fy1 = self._getFy(normalLoad, SA, SR, camber, pressure, decay=decay)
			Fy2 = self._getFy(normalLoad, SA2, SR, camber, pressure, decay=decay)

		return (Fy1 - Fy2)/2


	def _getFy(self, normalLoad, SA, SR, camber, pressure, giveByCy=0, decay=False):
		# Units: Newtons, Radians, n/a, Degrees, Degrees, PSI
		Fz = normalLoad # Newtons

		alpha = SA # radians
		alpha_star = tan(alpha) # E.E3
		gamma = np.radians(camber) # convert to radians
		gamma_star = sin(gamma) # 4.E4

		kappa = SR # 4.E5 longitudinal slip ratio

		# User scaling factors

		# Pure slip
		lambda_Fzo = 1 # nominal (rated) load
		lambda_muy = self.latKnockdown # peak friction coefficient
		lambda_mV = 1 if decay else 0 # with slip speed Vs decaying friction
		lambda_Kya = 1 # cornering stiffness
		lambda_Cy = 1 # shape factor
		lambda_Ey = 1 # curvature factor
		lambda_Hy = 0 # horizontal shift
		lambda_Vy = 0 # vertical shift
		lambda_Kyg = 1 # camber force stiffness
		lambda_Kzg = 1 # camber torque stiffness
		lambda_t = 1 # pneumatic trail (effecting aligning torque stiffness)
		lambda_Mr = 1 # residual torque

		# Combined slip
		lambda_yk = 1 # k influence on Fy(a)
		lambda_Vyk = 1 # k induced ‘ply-steer’ Fy
		lambda_s = 1 # Mz moment arm of Fx

		# Other
		lambda_Cz = 1 # radial tire stiffness
		lambda_Mx = 1 # overturning couple stiffness
		lambda_VMx = 1 # overturning couple vertical shift
		lambda_My = 1 # rolling resistance moment

		Vs = abs(alpha)*2
		Vo = 1
		A_mu = 10 # 4.E8
		lambda_muy_star = lambda_muy/(1 + lambda_mV*(Vs/Vo)) # 4.E7 exponent is custom
		lambda_muy_prime = A_mu*lambda_muy_star/(1 + (A_mu - 1)*lambda_muy) # 4.E8

		# Coefficients (Jianming)

		p_Cy1 = 1.418266537
		p_Dy1 = 2.434019893
		p_Dy2 = -0.695576039
		p_Dy3 = 12.30810792
		p_Ey1 = -0.84517486
		p_Ey2 = -4.54556333
		p_Ey3 = 0.447227266
		p_Ey4 = 8.764955681
		p_Ey5 = 1.24E+02
		p_Ky1 = 1.07E+02
		p_Ky2 = -0.427576554
		p_Ky3 = 0.463113678
		p_Ky4 = 0.243226338
		p_Ky5 = 37.59953533
		p_py1 = 0.599475805
		p_py2 = 1.586585962
		p_py3 = -0.256090058
		p_py4 = -0.468093311
		r_By1 = 8.408642105
		r_By2 = 13.58937167
		r_By3 = -0.067020443
		r_By4 = 10.40954157
		r_Cy1 = 1.374962365
		r_Ey1 = 0.450862075
		r_Ey2 = -0.079718502
		r_Hy1 = 0.017441348
		r_Hy2 = 0.031134507
		r_Vy1 = 0.002337109
		r_Vy2 = -0.00447019
		r_Vy3 = -0.178349769
		r_Vy4 = 11.54220691
		r_Vy5 = 1.357956528
		r_Vy6 =	12.46078307

		# undetermined
		p_Hy1 = 0
		p_Hy2 = 0
		p_Ky6 = 0
		p_Ky7 = 0
		p_py5 = 0
		p_Vy1 = 0
		p_Vy2 = 0
		p_Vy3 = 0
		p_Vy4 = 0

		# assume turn slip negligible (Page 178)
		zeta_o = 1
		zeta_2 = 1
		zeta_3 = 1
		zeta_4 = 1

		# avoid singularities (Page 177)
		eps_K = 0.001
		eps_y = 0.001

		# Page 177:
		Fzo = 250 * 4.448 # nominal rated wheel load (N)
		Fzo_prime = lambda_Fzo * Fzo # for tire with different nominal rated load
		dfz = (Fz - Fzo_prime)/Fzo_prime

		p_i = pressure * 6.89476 # tire pressure (kPa)
		p_io = 12 * 6.89476 # nominal inflation pressure (kPa)
		dpi = (p_i - p_io)/p_io # normalized inflation pressure


		# To find Dy...
		mu_y = (p_Dy1 + p_Dy2*dfz) * (1 + p_py3*dpi + p_py4*dpi**2) * (1 - p_Dy3 * gamma_star**2)*lambda_muy_star # 4.E23
		Dy = mu_y * Fz * zeta_2 # 4.E22

		# Cy
		Cy = np.clip(p_Cy1*lambda_Cy, 0, None) # 4.E21

		# To find S_Hy...
		K_ya = p_Ky1 * Fzo_prime * (1 + p_py1*dpi) * (1 - p_Ky3*abs(gamma_star)) * sin(p_Ky4*arctan((Fz/Fzo_prime)/((p_Ky2 + p_Ky5*gamma_star**2)*(1+p_py2*dpi))))*zeta_3*lambda_Kya # 4.E25
		S_Vyg = Fz*(p_Vy3 + p_Vy4*dfz)*gamma_star * lambda_Kyg * lambda_muy_prime * zeta_2 # 4.E28
		S_Vy = Fz*(p_Vy1 + p_Vy2*dfz) * lambda_Vy * lambda_muy_prime * zeta_2 + S_Vyg # 4.E29
		K_ygo = Fz*(p_Ky6 + p_Ky7*dfz)*(1 + p_py5*dpi)*lambda_Kyg # 4.E30
		S_Hy = (p_Hy1 + p_Hy2*dfz)*lambda_Hy + ((K_ygo*gamma_star - S_Vy)/(K_ya + eps_K))*zeta_o + zeta_4 - 1 # 4.E27
		alpha_y = alpha_star + S_Hy

		# By
		By = K_ya/(Cy*Dy + eps_y) # 4.E26

		# Ey
		Ey = (p_Ey1 + p_Ey2*dfz)*(1 + p_Ey5*gamma_star**2 - (p_Ey3 + p_Ey4*gamma_star)*sign(alpha_y))*lambda_Ey # 4.E24
		Ey = np.clip(Ey, None, 1.0)

		# Pure side slip
		Fy0 = Dy*sin(Cy*arctan(By*alpha_y - Ey*(By*alpha_y - arctan(By*alpha_y)))) + S_Vy # 4.E19


		D_Vyk = mu_y*Fz*(r_Vy1 + r_Vy2*dfz + r_Vy3*gamma_star) * cos(arctan(r_Vy4*alpha_star))*zeta_2

		# print('D_Vyk parts:')
		# print(mu_y*Fz*zeta_2)
		# print((r_Vy1 + r_Vy2*dfz + r_Vy3*gamma_star))
		# print(cos(arctan(r_Vy4*alpha_star)))
		# print('')
		# print('r_Vy4 = {}'.format(r_Vy4))
		# print('alpha_star = {}'.format(alpha_star))
		# print('')

		S_Vyk = D_Vyk*sin(r_Vy5*arctan(r_Vy6*kappa))*lambda_Vyk # 4.E66
		S_Hyk = r_Hy1 + r_Hy2*dfz # 4.E65
		E_yk = np.clip(r_Ey1 + r_Ey2*dfz, None, 1) # 4.E64
		C_yk = r_Cy1 # 4.E63
		B_yk = (r_By1 + r_By4*gamma_star**2)*cos(arctan(r_By2*(alpha_star-r_By3)))*lambda_yk # 4.E62
		B_yk = np.clip(B_yk, 0, None)

		# print('B_yk halves:')
		# print((r_By1 + r_By4*gamma_star**2))
		# print(cos(arctan(r_By2*(alpha_star-r_By3)))*lambda_yk)
		# print('')

		kappa_S = kappa + S_Hyk # 4.E61

		G_yko = cos(C_yk*arctan(B_yk*S_Hyk - E_yk*(B_yk*S_Hyk - arctan(B_yk*S_Hyk)))) # 4.E60
		G_yk = cos(C_yk*arctan(B_yk*kappa_S - E_yk*(B_yk*kappa_S - arctan(B_yk*kappa_S))))/G_yko # 4.E59
		G_yk = np.clip(G_yk, 0, None)

		Fy = G_yk * Fy0 + S_Vyk

		# print('By = {}'.format(By))
		# print('Cy = {}'.format(Cy))
		# print('Dy = {}'.format(Dy))
		# print('Ey = {}'.format(Ey))
		# print('Fy0 = {}'.format(Fy0))
		# print('Fy = {}'.format(Fy))
		# print('')
		# print('D_Vyk = {}'.format(D_Vyk))
		# print('S_Vyk = {}'.format(S_Vyk))
		# print('S_Hyk = {}'.format(S_Hyk))
		# print('E_yk = {}'.format(E_yk))
		# print('C_yk = {}'.format(C_yk))
		# print('B_yk = {}'.format(B_yk))
		# print('kappa_S = {}'.format(kappa_S))
		# print('G_yko = {}'.format(G_yko))
		# print('G_yk = {}'.format(G_yk))
		# print('')


		if giveByCy:
			return (By, Cy)

		return Fy

	def getMz(self, normalLoad, SA, SR, camber, pressure):
		Fz = normalLoad # Newtons

		alpha = SA # radians
		alpha_star = tan(alpha) # E.E3
		gamma = np.radians(camber) # convert to radians
		gamma_star = sin(gamma) # 4.E4
		R_0 = 9 * 0.0254 # nominal tire radius

		kappa = SR # 4.E5 longitudinal slip ratio

		lambda_muy = self.latKnockdown
		lambda_mV = 0
		Vs = -1
		Vo = -1
		A_mu = 10 # 4.E8
		lambda_muy_star = lambda_muy/(1 + lambda_mV*Vs/Vo) # 4.E7
		lambda_muy_prime = A_mu*lambda_muy_star/(1 + (A_mu - 1)*lambda_muy) # 4.E8
		lambda_Fzo = 1 # nominal (rated) load

		Fzo = 250 * 4.448 # nominal rated wheel load (N)
		Fzo_prime = lambda_Fzo * Fzo # for tire with different nominal rated load
		dfz = (Fz - Fzo_prime)/Fzo_prime

		p_i = pressure * 6.89476 # tire pressure (kPa)
		p_io = 12 * 6.89476 # nominal inflation pressure (kPa)
		dpi = (p_i - p_io)/p_io # normalized inflation pressure


		# Coefficients
		# p_pz1 = 0.89426748505866
		# p_pz2 = 0.525029025426923
		# q_Bz1 = 3.06952217907727
		# q_Bz10 = 0.638942639028383
		# q_Bz2 = 2.00669853797608
		# q_Bz3 = 3.67257592031595
		# q_Bz5 = 1.09460393465564
		# q_Bz6 = -17.2483279023548
		# q_Bz9 = 0
		# q_Cz1 = -0.131248660981021
		# q_Dz1 = 0.071941576666898
		# q_Dz10 = 0.735529945174363
		# q_Dz11 = -0.00488462182584
		# q_Dz2 = -0.009004124280201
		# q_Dz3 = -0.436337048773627
		# q_Dz4 = 0.414220153858278
		# q_Dz6 = 0.025357388049254
		# q_Dz7 = -0.000022712755858714
		# q_Dz8 = 2.21773542621264
		# q_Dz9 = -0.000844379994464579
		# q_Ez1 = 0.699641323057658
		# q_Ez2 = -2.00171190297911
		# q_Ez3 = 14.9706373211548
		# q_Ez4 = 3.19756032100024
		# q_Ez5 = -29.0416207125852

		p_pz1 = 0.98496
		p_pz2 = 0.38814
		q_Bz1 = 6.3863
		q_Bz10 = 0.539
		q_Bz2 = -2.984
		q_Bz3 = -6.4064
		q_Bz5 = 1.274
		q_Bz6 = -32.547
		q_Bz9 = 0
		q_Cz1 = 1.7268
		q_Dz1 = 0.17702
		q_Dz10 = -3.0497
		q_Dz11 = 14.455
		q_Dz2 = -0.084265
		q_Dz3 = 1.3761
		q_Dz4 = -53.558
		q_Dz6 = -0.0088627
		q_Dz7 = -0.03841
		q_Dz8 = 1.348
		q_Dz9 = -1.5976
		q_Ez1 = 0.57635
		q_Ez2 = -0.6198
		q_Ez3 = -2.8128
		q_Ez4 = 0.38244
		q_Ez5 = -9.6023





		F_y0 = self.getFy(normalLoad, SA, SR, camber, pressure)

		B_t = (q_Bz1 + q_Bz2*dfz + q_Bz3*dfz**2)*(1 + q_Bz5*abs(gamma_star) + q_Bz6*gamma_star**2)/lambda_muy # 4.E40
		B_t = np.clip(B_t, 0, None)

		C_t = np.clip(q_Cz1, 0, None) # 4.E41

		D_t0 = Fz*(R_0/Fzo_prime) * (q_Dz1 + q_Dz2*dfz) * (1 - p_pz1*dpi) # 4.E42
		D_t = D_t0 * (1 + q_Dz3*abs(gamma_star) + q_Dz4*gamma_star**2) # 4.E43

		E_t = (q_Ez1 + q_Ez2*dfz + q_Ez3*dfz**2) * (1 + (q_Ez4 + q_Ez5*gamma_star)*(2/np.pi)*arctan(B_t*C_t*alpha)) # 4.E44
		E_t = np.clip(E_t, None, 1)

		M_z0_prime = F_y0 * D_t * cos(C_t*arctan(B_t*alpha - E_t*(B_t*alpha - arctan(B_t*alpha)))) # 4.E32,33
		


		(B_y, C_y) = self.getFy(normalLoad, SA, SR, camber, pressure, giveByCy=1)

		B_r = (q_Bz9/lambda_muy_star) + q_Bz10*B_y*C_y

		C_r = 1 # 4.E46

		D_r = Fz * R_0 * ((q_Dz6 + q_Dz7*dfz) + ((q_Dz8 + q_Dz9*dfz)*(1 + p_pz2*dpi) + \
		      (q_Dz10 + q_Dz11*dfz)*abs(gamma_star))*gamma_star) * lambda_muy_star*cos(alpha) # 4.E47

		# print('D_r halves:')
		# print(Fz * R_0 * lambda_muy_star*cos(alpha))
		# print(((q_Dz6 + q_Dz7*dfz) + ((q_Dz8 + q_Dz9*dfz)*(1 + p_pz2*dpi) + (q_Dz10 + q_Dz11*dfz)*abs(gamma_star))*gamma_star))
		# print('')
		# print((q_Dz10 + q_Dz11*dfz)*abs(gamma_star)*gamma_star)
		# print((q_Dz6 + q_Dz7*dfz))
		# print(q_Dz6, q_Dz7, dfz)
		# print('')

		M_zr0 = D_r * cos(C_r * arctan(B_r*alpha)) * cos(alpha)

		M_z0 = M_z0_prime + M_zr0 # 4.E31

		# print('F_y0 = {}'.format(F_y0))
		# print('B_t = {}'.format(B_t))
		# print('C_t = {}'.format(C_t))
		# print('D_t = {}'.format(D_t))
		# print('E_t = {}'.format(E_t))
		# print('B_y = {}'.format(B_y))
		# print('C_y = {}'.format(C_y))
		# print('B_r = {}'.format(B_r))
		# print('C_r = {}'.format(C_r))
		# print('D_r = {}'.format(D_r))
		# print('M_z0 = {}'.format(M_z0))


		return M_z0

def test():
	tire = Tire(longKnockdown=0.6, latKnockdown=0.47)

	tire.getMz(normalLoad=1100, SA=np.radians(12), SR=0, camber=0, pressure=12)

	tire.getFy(normalLoad=220, SA=np.radians(5), SR=0, camber=0, pressure=12)

	tire.getFx(normalLoad=220, SA=0, SR=0.1, camber=0, pressure=12)

	print(tire.getFx(normalLoad=400, SA=0, SR=0.1, camber=0, pressure=12))

	normals = [220, 440, 670, 890, 1110, 1500, 2000, 2500]
	SRs = arange(-0.5, 0.5, 0.01)

	for normal in normals:
		Fxs = tire.getFx(normalLoad=normal, SA=0, SR=SRs, camber=0, pressure=12)
		plt.plot(SRs, Fxs)

	plt.legend([str(normal) + 'N' for normal in normals])
	plt.xlabel('Slip Ratio')
	plt.ylabel('Fx (N)')
	plt.show()

	print(tire.getFy(normalLoad=400, SA=5, SR=0, camber=0, pressure=12))

	SAs = arange(-0.22,0.22,0.01)
	colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink']

	for color, normal in zip(colors, normals):
		Fys = tire.getFy(normalLoad=normal, SA=SAs, SR=0, camber=0, pressure=12)
		#plt.plot(np.degrees(SAs)[:-1], Fys[1:] - Fys[:-1])
		plt.plot(np.degrees(SAs), Fys, color=color)

		# Fys = tire.getFy(normalLoad=normal, SA=SAs, SR=0, camber=0, pressure=12, decay=False)
		# plt.plot(np.degrees(SAs), (Fys-Fys[::-1])/2, color=color)

	plt.legend([str(normal) + 'N' for normal in normals])
	plt.xlabel('Slip Angle (deg)')
	plt.ylabel('Fy (N)')
	#plt.ylim(-4000, 4000)
	plt.show()

	print(tire.getMz(normalLoad=400, SA=5, SR=0, camber=0, pressure=12))

	for normal in normals:
		Mzs = tire.getMz(normalLoad=normal, SA=SAs, SR=0, camber=0, pressure=12)
		plt.plot(np.degrees(SAs), (Mzs-Mzs[::-1])/2)

	plt.legend([str(normal) + 'N' for normal in normals])
	plt.xlabel('Slip Angle (deg)')
	plt.ylabel('Mz (Nm)')
	plt.show()

#test()
