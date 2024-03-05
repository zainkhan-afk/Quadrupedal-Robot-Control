import numpy as np

class RobotModel:
	def __init__(self, base_transformation):
		self.bodies = []
		self.parents = []
		self.inertias = []
		self.local_transformations = []
		self.global_transformations = []

		self.base_transformation = base_transformation

	def AddBody(self, body_id, parent_id, inertia, T):
		self.bodies.append(body_id)
		self.parents.append(parent_id)
		self.inertias.append(inertia)
		self.transformations.append(T)

	def ForwardKinematics(self):
		for body_id in self.bodies:
			