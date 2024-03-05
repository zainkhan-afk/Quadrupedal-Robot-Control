from Spatial import SpatialTransformation
from utils import GetRotMat
import numpy as np

class Joint:
	def __init__(self, T, q, axis, parent, child):
		self.T = T
		self.q = q
		self.axis = axis
		self.parent = parent
		self.child = child

	def Update(self, q):
		R_joint = GetRotMat(q, self.axis)
			T_joint = SpatialTransformation(R_joint, np.zeros((3, 1)))
		self.child.joint_rotated_T = T_joint@self.child.local_T
		self.child.global_T = self.parent.global_T@self.child.joint_rotated_T