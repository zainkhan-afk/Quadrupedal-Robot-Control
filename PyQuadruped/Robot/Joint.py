from PyQuadruped.Spatial import SpatialTransformation
from PyQuadruped.utils import GetRotMat
import numpy as np

class Joint:
	def __init__(self, q, T, axis, child, joint_reversed = False):
		self.q = q
		self.local_T = T
		self.global_T = T
		self.axis = axis
		self.child = child
		self.joint_reversed = joint_reversed

	def GetTransformationMatrices(self, all_T):
		# all_T.append(self.global_T)
		self.child.GetTransformationMatrices(all_T)

	def SetAngle(self, q):
		self.q = q

	def Update(self, parent_T_joint):
		if self.joint_reversed:
			R_joint = GetRotMat(-self.q, self.axis)
		else:
			R_joint = GetRotMat(self.q, self.axis)
		joint_T_rotated_joint = SpatialTransformation(R_joint, np.zeros((3, 1)))

		self.global_T = parent_T_joint@self.local_T

		self.child.Update(self.global_T@joint_T_rotated_joint)

	# def Update(self, q):
	# 	self.q = q
	# 	R_joint = GetRotMat(self.q, self.axis)
	# 	T_joint = SpatialTransformation(R_joint, np.zeros((3, 1)))
	# 	self.child.global_T = self.parent.global_T@T_joint@self.child.local_T
	# 	self.global_T = self.parent.global_T@self.local_T