import numpy as np
from spatial import SpatialTransformation


def GetRotMat(q, axis):
	if axis == 0:
		R = np.array([
						[1, 		0, 			0], 
						[0, np.cos(q), -np.sin(q)], 
						[0, np.sin(q), 	np.cos(q)]
					])

	elif axis == 1:
		R = np.array([
							[ np.cos(q), 0, np.sin(q)], 
							[ 		  0, 1, 		0], 
							[-np.sin(q), 0, np.cos(q)]
					])

	else:
		R = np.array([
							[np.cos(q), -np.sin(q), 0], 
							[np.sin(q),  np.cos(q), 0], 
							[		 0, 		 0, 1]
						])


	return R

def get_rot_mat( x, y, z):
	R_x = np.array([
		[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]
		])

	R_y = np.array([
		[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]
		])

	R_z = np.array([
		[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]
		])


	return R_x@R_y@R_z


def get_inverse_transformation(T):
	R = T[0:3, 0:3]
	trans = T[:-1, -1][:, np.newaxis]

	new_R = np.linalg.inv(R)
	new_trans = -new_R@trans

	T = np.append(new_R, new_trans, axis = 1)
	T = np.append(T, np.array([[0, 0, 0, 1]]), axis = 0)

	return T

def trig_solve(chain, angle_list):
	chain = chain.lower()
	i = 0
	product = 1
	for trig_func in chain:
		if trig_func == "s":
			product *= np.sin(angle_list[i])
		if trig_func == "c":
			product *= np.cos(angle_list[i])

		i += 1

	return product


def GetLegSignedVector(V, leg_i):
	if leg_i == 0:
		V[1] *= -1

	elif leg_i == 1:
		V = V
	
	elif leg_i == 2:
		V[0] *= -1
		V[1] *= -1
	
	elif leg_i == 3:
		V[0] *= -1

	return V


def Vector2SkewMat(V):
	M = np.array([
					[       0, -V[2, 0],  V[1, 0]],
					[ V[2, 0],        0, -V[0, 0]],
					[-V[1, 0],  V[0, 0],       0]
				 ])

	return M


def Mat2SkewVec(M):
	V = np.array([
					[M[2, 1] - M[1, 2]],
					[M[0, 2] - M[2, 0]],
					[M[1, 0] - M[0, 1]]
				]) * 0.5

	return V
