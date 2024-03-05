import numpy as np
from utils import Vector2SkewMat

def SpatialInertia(mass, inertia, COM):
	I = np.zeros((6, 6))
	I

	return I


def SpatialTransformation(R, T):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	X[3:, :3] = -R @ Vector2SkewMat(T)

	print(X)

	return X

if __name__ == "__main__":
	SpatialTransformation(np.random.random((3, 3)), np.array([[1],[2],[3]]))