import numpy as np
from utils import Vector2SkewMat

def SpatialInertia(mass, I, COM):
	inertia = np.zeros((6, 6))
	COM_skew = Vector2SkewMat(COM)
	inertia[:3, :3] = I + (mass * (COM_skew @ COM_skew.T))
	inertia[:3, 3:] = mass*COM_skew
	inertia[3:, :3] = mass*COM_skew.T
	inertia[3:, 3:] = mass*np.eye(3)

	return inertia


def SpatialTransformation(R, T):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	X[3:, :3] = -R @ Vector2SkewMat(T)

	print(X)

	return X

if __name__ == "__main__":
	SpatialTransformation(np.random.random((3, 3)), np.array([[1],[2],[3]]))