import numpy as np
from utils import *

def almostEqual(p1, p2, thresh = 0.01):
	return np.sqrt(((np.array(p1) - np.array(p2))**2).sum()) < thresh

class KineTest:
	def __init__(self):
		self.l1 = 0.062
		self.l2 = 0.209
		self.l3 = 0.195
		self.l4 = 0.004

		self.l14 = self.l1 + self.l4

		self.side = [-1, 1, -1, 1]

	def FK1(self, q, leg):
		s1 = np.sin(q[0])
		s2 = np.sin(q[1])
		s3 = np.sin(q[2])

		c1 = np.cos(q[0])
		c2 = np.cos(q[1])
		c3 = np.cos(q[2])

		c23 = c2*c3 - s2*s3
		s23 = s2*c3 + c2*s3

		p = [0]*3

		p[0] = self.l3*s23 + self.l2*s2
		p[1] = self.l14 * self.side[leg] * c1 + self.l3 * (s1*c23) + self.l2*c2*s1
		p[2] = self.l14 * self.side[leg] * s1 - self.l3 * (s1*c23) - self.l2*c2*c1

		return p

	def FK2(self, q, leg):
		# X: c1*(c2*c3*l3 + c2*l2 - 1.0*l3*s2*s3) - 1.0*l1*s1
		# Y: 1.0*c1*l1 + s1*(c2*c3*l3 + c2*l2 - 1.0*l3*s2*s3)
		# Z: -1.0*c2*l3*s3 - 1.0*c3*l3*s2 - 1.0*l2*s2


		# -90 in DH and -90 in Robot
		x = - self.l14*trig_solve('s', [q[0]]) + trig_solve('c', [q[0]])*(self.l3*trig_solve('cc', q[1:]) + self.l2*trig_solve('c', [q[1]]) - self.l3*trig_solve('ss', q[1:]))
		y =   self.l14*trig_solve('c', [q[0]]) + trig_solve('s', [q[0]])*(self.l3*trig_solve('cc', q[1:]) + self.l2*trig_solve('c', [q[1]]) - self.l3*trig_solve('ss', q[1:]))
		z = - self.l3*trig_solve('cs', q[1:]) - self.l3*trig_solve('sc', q[1:]) - self.l2*trig_solve('s', [q[1]]) 

		# pos = np.array([[x, y, z]]).T

		# pos = get_rot_mat(0, np.pi/2, 0)@pos

		# print(get_rot_mat(0, np.pi/2, 0))

		# p = [pos[0, 0], pos[1, 0], pos[2, 0]]

		p = [z, y, -x]

		return p

	def IK1(self, p, leg):
		D = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2] - self.l14 * self.l14 - self.l2 * self.l2 - self.l3 * self.l3) / (2 * self.l2 * self.l3)

		if D>1:
			D = 1

		if D<-1:
			D = -1

		q = [0]*3

		q[2] = np.arctan2(-np.sqrt(1 - D*D), D)
		q[0] = -np.arctan2(p[2], p[1]) - np.arctan2(np.sqrt(p[1] * p[1] + p[2] * p[2] - self.l14 * self.l14), self.side[leg] * self.l14);
		q[1] = np.arctan2(-p[0], np.sqrt(p[1] * p[1] + p[2] * p[2] - self.l14 * self.l14)) - np.arctan2(self.l3 * np.sin(q[2]), self.l2 + self.l3 * np.cos(q[2]));

		q[0] *= -1

		return q

	def IK2(self, p, leg):
		x, y, z = p[0], p[1], p[2]
		R = np.sqrt(z**2 + y**2)

		beta = np.arccos(y/R)
		alpha  = np.arccos(self.l14/R)

		theta1 = alpha - beta

		R_x = get_rot_mat(-theta1, 0, 0)
		R_yz = get_rot_mat(0, -np.pi/2, np.pi)

		print(R_x)
		print(R_yz)

		p = np.array([[x, y, z]]).T
		p = R_yz@(R_x@p)

		x_ = p[0, 0]
		y_ = p[1, 0] + self.l14
		z_ = p[2, 0]


		temp = (x_**2 + z_**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

		if temp>1:
			temp = 1
		if temp<-1:
			temp = -1

		theta3 =   np.arccos(temp)
		theta2 =   (np.arctan2(z_, x_) - np.arctan2(self.l3*np.sin(theta3),(self.l2 + self.l3*np.cos(theta3))))


		q = [theta1, theta2, theta3]

		return q


if __name__ == "__main__":

	kine = KineTest()

	init_q = [0, 0, 0]

	for i in range(2):
		print(f"Iter: {i}. Init Q: {init_q}")
		for leg in range(4):
			p = kine.FK2(init_q, leg)
			q = kine.IK2(p, leg)

			print(f"Leg: {leg}, Position: {p}, IK Angles: {q}")

		init_q = q


	x_vals = np.arange(-0.3, 0.3, 0.05)
	y_vals = np.arange(-0.05, 0.05, 0.05)
	z_vals = np.arange(-0.2, -0.1, 0.05)

	# y = 0.066
	# z = -0.2
	# for x in x_vals:
	# 	for y in y_vals:
	# 		for z in z_vals:
	# 			p_target = [x, y, z]
	# 			q = kine.IK2(p_target, 0)
	# 			p_FK = kine.FK2(q, 0)

	# 			almost_equal = almostEqual(p_target, p_FK)

	# 			print(almost_equal, p_target, p_FK)

	# 	print(almost_equal)