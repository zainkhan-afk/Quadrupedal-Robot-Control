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
	
	skew_vec = P_inertia[:3, -1].reshape(3, 1)
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
	P_inertia[:3, -1] = skew_vec.ravel()
	P_inertia[-1, :3] = skew_vec.T.ravel()

	P_inertia[-1, -1] = mass

	return P_inertia

def FlipSpatialInertia(inertia, axis):
	P_inertia = GetPseudoInertia(inertia)
	T = np.eye(4)

	if axis == 0:
		T[0, 0] = -1

	elif axis == 1:
		T[1, 1] = -1

	elif axis == 2:
		T[2, 2] = -1

	P_inertia = T@P_inertia@T
	return SpatialInertiaFromHomog(P_inertia)

def SpatialTransformation(R, T):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	X[3:, :3] = R @ Vector2SkewMat(T)

	return X

def SpatialToHomog(X):
	T = np.zeros((4, 4))
	R = X[:3, :3]
	R_skew = X[3:, :3]

	T[:3, :3] = R
	T[-1, -1] = 1

	T[:3, -1] = Mat2SkewVec(R_skew @ R.T).ravel()

	return T

# if __name__ == "__main__":
# 	np.random.seed(1)
# 	I = SpatialInertia(1, np.random.random((3, 3)), np.zeros((3, 1)))
# 	print(I)
# 	print()
# 	I_flipped = FlipSpatialInertia(I, 1)
# 	print(I_flipped)
# 	print()
# 	I_flipped = FlipSpatialInertia(I_flipped, 1)
# 	print(I_flipped)
# 	print()

