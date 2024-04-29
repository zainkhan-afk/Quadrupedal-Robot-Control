import numpy as np
import cv2
from PyQuadruped.Spatial import ForceCrossProduct, SpatialTransformation
from PyQuadruped.utils import GetRotMat, GetJointSubspace

class JointLink:
	def __init__(self, inertia, T, q, joint_axis, joint_reversed, 
				parent = None, children = [], name = ""):
		self.inertia = inertia
		self.local_T = T
		self.q = q
		self.joint_axis = joint_axis
		self.joint_reversed = joint_reversed
		self.parent = parent 
		self.children = children
		self.name = name
		
		self.J_T = T
		self.articulated_inertia = inertia.copy()
		self.G_T_i = np.eye(6)
		self.parent_T_child = None

		self.q_dot = np.zeros((6, 1))

		self.v = np.eye(6)
		self.pa = np.zeros((6, 1))
		self.U  = None
		
		self.S = GetJointSubspace(self.joint_axis)
		self.joint_vel_spatial = self.S*self.q_dot

	def AddChild(self, child):
		self.children.append(child)

	def GetTransformationMatrices(self, all_T):
		all_T.append(self.G_T_i)
		for child in self.children:
			child.GetTransformationMatrices(all_T)

	def SetAngle(self, i, state):
		if self.parent is not None:
			self.q = state.q[i, 0]
			self.q_dot = np.zeros((6, 1))
			self.joint_vel_spatial = self.S*self.q_dot
			i += 1
			for child in self.children:
				child.SetAngle(i, state)
		else:
			leg = 0
			for child in self.children:
				child.SetAngle(leg*3 + i, state)
				leg += 1

	def Update(self, G_T):
		if self.parent is not None:
			if self.joint_reversed:
				R_joint = GetRotMat(-self.q, self.joint_axis)
			else:
				R_joint = GetRotMat(self.q, self.joint_axis)

			self.J_T = SpatialTransformation(R_joint, np.zeros((3, 1)))
			print(self.q)
		else:
			self.J_T = np.eye(6)

		self.G_T_i = G_T@self.local_T
		
		for child in self.children:
			child.Update(self.G_T_i@self.J_T)

	def __repr__(self):
		return "JointLink " + self.name

	def __str__(self):
		return self.__repr__()