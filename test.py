import cv2
import numpy as np
from scipy.spatial.transform import Rotation as RTool
import math

def ToHomog(R, p):
	H = np.zeros((4, 4))
	H[:3, :3] = R
	H[:3, -1] = p.ravel()
	H[-1, -1] = 1

	return H

def ToSpatial(R, p):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	Sp = VecToSkewMat(p)
	X[3:, :3] = Sp@R

	return X


def GetRotationFromSpatial(X):
	return X[:3, :3]


def GetTranslationFromSpatial(X):
	R = GetRotationFromSpatial(X)
	skewR = X[3:, :3]

	return SketMatToVec(skewR@R.T)
	

def SketMatToVec(m):
	v = np.zeros((3, 1))

	v[0, 0] = m[2, 1] - m[1, 2]
	v[1, 0] = m[0, 2] - m[2, 0]
	v[2, 0] = m[1, 0] - m[0, 1]

	return v*0.5

def VecToSkewMat(v):
	x = v[0, 0]
	y = v[1, 0]
	z = v[2, 0]

	m = np.array([
				[ 0, -z,  y],
				[ z,  0, -x],
				[-y,  x,  0]
				])

	return m

def ToRotMat(r, p, y):
	sr = np.sin(r)
	cr = np.cos(r)

	sp = np.sin(p)
	cp = np.cos(p)

	sy = np.sin(y)
	cy = np.cos(y)

	RX = np.array([
					[1,  0,   0],
					[0, cr, -sr],
					[0, sr,  cr]
				   ])

	RY = np.array([
					[ cp, 0, sp],
					[  0, 1,  0],
					[-sp, 0, cp]
				   ])

	RZ = np.array([
					[cy, -sy, 0],
					[sy,  cy, 0],
					[ 0,   0, 1]
					])

	return RX@RY@RZ

def ToRad(deg):
	return deg / 180 * np.pi



R = ToRotMat(0, np.pi / 4, 0)

w = np.array([[0.1, 0.2, 0.33]]).T
p = np.array([[0.5, 0, 0.33]]).T

p_skew = VecToSkewMat(p)
w_skew = VecToSkewMat(w)

wp = np.cross(w.T, p.T)
wp_skew = p_skew@w
w_skewp = w_skew@p

print(R@wp.T)
print()
print(R@wp_skew)
print(p_skew@R@w)
print()

print(R@w_skewp)
print(R)


exit()

scale = 20

theta1 = np.random.random()
theta2 = np.random.random()
theta3 = np.random.random()
w_P_a = np.array([[2, 0, 0]]).T*scale
w_R_a = ToRotMat(0, ToRad(theta1), 0)

a_P_b = np.array([[2, 3, 0]]).T*scale
a_R_b = ToRotMat(ToRad(theta2), 0, 0)

b_P_c = np.array([[2, -2, 0]]).T*scale
b_R_c = ToRotMat(0, 0, ToRad(theta3))

c_P_d = np.array([[2, 0, 0]]).T*scale
c_R_d = ToRotMat(0, 0, 0)

w_X_a = ToSpatial(w_R_a, w_P_a)
a_X_b = ToSpatial(a_R_b, a_P_b)
b_X_c = ToSpatial(b_R_c, b_P_c)
c_X_d = ToSpatial(c_R_d, c_P_d)


w_T_a = ToHomog(w_R_a, w_P_a)
a_T_b = ToHomog(a_R_b, a_P_b)
b_T_c = ToHomog(b_R_c, b_P_c)
c_T_d = ToHomog(c_R_d, c_P_d)


w_X_d = w_X_a@a_X_b@b_X_c@c_X_d
w_T_d = w_T_a@a_T_b@b_T_c@c_T_d

print(GetTranslationFromSpatial(w_X_d).ravel())
print(w_T_d[:, -1].ravel())