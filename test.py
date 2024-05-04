import numpy as np


def ToSkewVec(v):
	m = np.zeros((3, 3))

	m[0, 1] = v[2, 0]
	m[0, 2] = v[1, 0]

	m[1, 0] = v[2, 0]
	m[1, 2] = v[0, 0]
	
	m[2, 0] = v[1, 0]
	m[2, 1] = v[0, 0]

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

v = np.array([[2, 3, -1]]).T

R = ToRotMat(ToRad(45), 0, 0)
Sv = ToSkewVec(v)

skewR = -R@Sv

print(Sv)
Sv_recons1 = -R.T@skewR
Sv_recons2 = skewR@R.T
print(Sv_recons1)
print(Sv_recons2)