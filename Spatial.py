import numpy as np
from utils import Vector2SkewMat, Mat2SkewVec



def SpatialInertia(mass, I, COM):
	inertia = np.zeros((6, 6))
	COM_skew = Vector2SkewMat(COM)
	inertia[:3, :3] = I + (mass * (COM_skew @ COM_skew.T))
	inertia[:3, 3:] = mass*COM_skew
	inertia[3:, :3] = mass*COM_skew.T
	inertia[3:, 3:] = mass*np.eye(3)

	return inertia

def SpatialInertiaFromHomog(P_inertia):
	inertia = np.zeros((6, 6))

	mass = P_inertia[-1, -1]
	
	skew_vec = P_inertia[:3, -1]
	mat = P_inertia[:3, :3]

	I_b = np.trace(mat) * np.eye(3) - mat

	skew_mat = Vector2SkewMat(skew_vec)


	inertia[:3, :3] = I_b
	inertia[:3, 3:] = skew_mat
	inertia[3:, :3] = skew_mat.T
	inertia[3:, 3:] = mass * np.eye(3)

	return inertia



def GetPseudoInertia(inertia):
	skew_vec = Mat2SkewVec(inertia[:3, 3:])
	I_b = inertia[:3, :3]
	mass = inertia[-1, -1]

	P_inertia = np.zeros((4, 4))

	P_inertia[:3, :3] = 0.5 * np.trace(I_b) * np.eye(3) - I_b
	P_inertia[:3, -1] = skew_vec
	P_inertia[-1, :3] = skew_vec.T

	P_inertia[-1, -1] = mass

	return P_inertia

def FlipSpatialInertia(inertia, axis):
	P_inertia = GetPseudoInertia(inertia)
	T = np.zeros(4, 4)

	if axis == 0:
		T[0, 0] = -1

	elif axis == 1:
		T[1, 1] = -1

	elif axis == 2:
		T[2, 2] = -1

	P_inertia = X@P_inertia@X
	return SpatialInertiaFromHomog(P_inertia)



def SpatialTransformation(R, T):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	X[3:, :3] = -R @ Vector2SkewMat(T)

	print(X)

	return X

if __name__ == "__main__":
	SpatialTransformation(np.random.random((3, 3)), np.array([[1],[2],[3]]))