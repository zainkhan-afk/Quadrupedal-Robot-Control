import numpy as np


def GetRotMat(q, axis):
	if axis == 0:
		R = np.array([
							[1, 		0, 			0], 
							[0, np.cos(q), -np.sin(q)], 
							[0, np.sin(q), 	np.cos(q)],
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


def GetLegSignedVector(v, leg_i):
	V = v.copy()
	if leg_i == 0:
		V[1] *= -1

	elif leg_i == 1:
		V = v.copy()
	
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


def GetJointSubspace(axis):
	S = np.zeros((6, 1))

	S[axis] = 1

	return S


def QuatToEulerRot(q):
	e0 = q[0, 0]
	e1 = q[1, 0]
	e2 = q[2, 0]
	e3 = q[3, 0]

	R = np.array([
					[1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2)], 
					[2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1)],
					[2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2)]
				]).T

	return R

def IntegrateQuatImplicit(body_orientation, omega_body, dt):
	ang = np.linalg.norm(omega_body)
	if ang > 0:
		axis = omega_body / ang
	
	else:
		axis = np.array([[1, 0, 0]]).T

	ang *= dt
	ee = np.sin(ang / 2) * axis
	quat_d = np.array([[np.cos(ang / 2), ee[0, 0], ee[1, 0], ee[2, 0]]]).T

	quat_new = QuatProduct(body_orientation, quat_d)
	quat_new = quat_new / np.linalg.norm(quat_new)
	return quat_new;

def QuatProduct(q1, q2):
	r1 = q1[0, 0]
	r2 = q2[0, 0]
	v1 = np.array([[q1[1, 0], q1[2, 0], q1[3, 0]]]).T
	v2 = np.array([[q2[1, 0], q2[2, 0], q2[3, 0]]]).T

	r = r1 * r2 - v1.T.dot(v2)
	v = r1 * v2 + r2 * v1 + np.cross(v1.ravel(), v2.ravel()).reshape(3, 1)

	q = np.array([[r[0, 0], v[0, 0], v[1, 0], v[2, 0]]]).T
	return q