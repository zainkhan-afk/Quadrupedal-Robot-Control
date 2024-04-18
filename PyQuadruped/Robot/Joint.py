from PyQuadruped.Spatial import SpatialTransformation
from PyQuadruped.utils import GetRotMat, GetJointSubspace
import numpy as np

class Joint:
	def __init__(self, q, T, axis, parent, child, joint_reversed = False, name = ""):
		self.q = q
		self.q_dot = np.zeros((6, 1))
		self.local_T = T
		self.global_T = T
		self.J_T = T
		self.axis = axis
		self.parent = parent
		self.child = child
		self.child.parent_joint = self
		self.joint_reversed = joint_reversed
		self.name = name

		self.S = GetJointSubspace(self.axis)

	def GetTransformationMatrices(self, all_T):
		# all_T.append(self.global_T)
		self.child.GetTransformationMatrices(all_T)

	def SetAngle(self, q, q_dot = np.zeros((6, 1))):
		self.q = q
		self.q_dot = q_dot
		self.joint_vel_spatial = self.S*self.q_dot

	def Update(self, parent_T_joint):
		if self.joint_reversed:
			R_joint = GetRotMat(-self.q, self.axis)
		else:
			R_joint = GetRotMat(self.q, self.axis)
		self.J_T = SpatialTransformation(R_joint, np.zeros((3, 1)))

		self.global_T = parent_T_joint@self.local_T

		self.child.Update(self.global_T@self.J_T)

	def __repr__(self):
		return "Joint " + self.name

	def __str__(self):
		return self.__repr__()

	# def Update(self, q):
	# 	self.q = q
	# 	R_joint = GetRotMat(self.q, self.axis)
	# 	T_joint = SpatialTransformation(R_joint, np.zeros((3, 1)))
	# 	self.child.global_T = self.parent.global_T@T_joint@self.child.local_T
	# 	self.global_T = self.parent.global_T@self.local_T