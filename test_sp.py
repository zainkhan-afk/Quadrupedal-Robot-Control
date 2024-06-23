import numpy as np

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

class SpatialTransform:
	def __init__(self, R, p):
		self.R = R
		self.p = p

	def GetSpatialMatrix(self):
		X = np.zeros((6, 6))
		X[:3, :3] = self.R
		X[3:, 3:] = self.R
		X[3:, :3] = -self.R @ VecToSkewMat(self.p)

		return X

	def __mul__(self, other):
		return SpatialTransform(R@other.R, other.p + other.R.T@self.p)

w0 = np.array([[np.random.random(), np.random.random(), 
				np.random.random(), np.random.random(), 
				np.random.random(), np.random.random()]]).T

R1 = ToRotMat(np.random.random(), np.random.random(), np.random.random())
p1 = np.array([[np.random.random(), np.random.random(), np.random.random()]]).T
sp1 = SpatialTransform(R1, p1)

R2 = ToRotMat(np.random.random(), np.random.random(), np.random.random())
p2 = np.array([[np.random.random(), np.random.random(), np.random.random()]]).T
sp2 = SpatialTransform(R2, p2)

R3 = ToRotMat(np.random.random(), np.random.random(), np.random.random())
p3 = np.array([[np.random.random(), np.random.random(), np.random.random()]]).T
sp3 = SpatialTransform(R3, p3)


w3 = sp3.GetSpatialMatrix() @ sp2.GetSpatialMatrix() @ sp1.GetSpatialMatrix() @ w0
print(w3.T)

w3 = sp1.GetSpatialMatrix() @ sp2.GetSpatialMatrix() @ sp3.GetSpatialMatrix() @ w0
print(w3.T)

w1 = np.zeros((6, 1))
w1[3:, :] = R1 @ (w0[3:, :] + np.cross(w0[:3, :].T, p1.T).T)
w1[:3, :] = R1 @ w0[:3, :]

w2 = np.zeros((6, 1))
w2[3:, :] = R2 @ (w1[3:, :] + np.cross(w1[:3, :].T, p2.T).T)
w2[:3, :] = R2 @ w1[:3, :]

w3 = np.zeros((6, 1))
w3[3:, :] = R3 @ (w2[3:, :] + np.cross(w2[:3, :].T, p3.T).T)
w3[:3, :] = R3 @ w2[:3, :]


print(w3.T)
