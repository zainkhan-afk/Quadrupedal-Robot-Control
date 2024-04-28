import numpy as np
import cv2
from PyQuadruped.Spatial import ForceCrossProduct, SpatialTransformation
from PyQuadruped.utils import GetRotMat, GetJointSubspace

class JointLink:
	def __init__(self, inertia, T, name = ""):
		self.inertia = inertia
		self.articulated_inertia = inertia.copy()
		self.local_T = T
		self.name = name
		self.parent_joint = None 
		self.global_T = np.eye(6)
		self.v = np.eye(6)
		self.pa = np.zeros((6, 1))
		self.parent_T_child = None

		self.childred_joints = []
		self.U  = None


	def __repr__(self):
		return "JointLink " + self.name

	def __str__(self):
		return self.__repr__()