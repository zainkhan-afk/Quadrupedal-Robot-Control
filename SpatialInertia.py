import numpy as np
from utils import Vector2SkewMat

class SpatialInertia:
	def __init__(self, mass, I, COM):
		COM_skew = Vector2SkewMat(COM)

		self.inertia = np.zeros((6, 6))

		self.inertia[:3, :3] = I + (mass * (COM_skew @ COM_skew.T))
		self.inertia[:3, 3:] = mass*COM_skew
		self.inertia[3:, :3] = mass*COM_skew.T
		self.inertia[3:, 3:] = mass*np.eye(3)

		print(self.inertia)


	def __add__(self, other):
		self.inertia + other.inertia


if __name__ == "__main__":
	np.random.seed(1)
	SI = SpatialInertia(1, np.random.random((3, 3)), np.array([[1], [1], [1]]))