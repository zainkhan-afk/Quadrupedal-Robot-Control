import numpy as np

class RobotModel:
	def __init__(self, base_transformation):
		self.parents = []
		self.inertias = []
		self.local_transformations = []
		self.after_rotation_transformations = []
		self.global_transformations = []
		self.joint_axis_with_parent = []

		self.base_transformation = base_transformation

	def AddBody(self, inertia, T, parent_id, joint_axis_with_parent):
		self.inertias.append(inertia)
		self.local_transformations.append(T)
		self.parents.append(parent_id)
		self.joint_axis_with_parent.append(joint_type)

	def ForwardKinematics(self, state):
		for i in range(1, len(self.parents)):
			pass