import numpy as np
import cv2
from PyQuadruped.Spatial import ForceCrossProduct, SpatialTransformation
from PyQuadruped.utils import GetRotMat, GetJointSubspace

class JointLink:
	def __init__(self, inertia, T, joint_axis, parent, child, name = ""):
		self.inertia = inertia
		self.articulated_inertia = inertia.copy()
		self.local_T = T
		self.joint_axis = joint_axis
		self.name = name
		
		self.parent = None 
		self.children = []
		self.global_T = np.eye(6)
		self.parent_T_child = None

		self.v = np.eye(6)
		self.pa = np.zeros((6, 1))
		self.U  = None

	def AddChild(self, child):
		self.children.append(child)

	def Update(self, T):
		pass



	def __repr__(self):
		return "JointLink " + self.name

	def __str__(self):
		return self.__repr__()