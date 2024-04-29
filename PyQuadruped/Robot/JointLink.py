import numpy as np
import cv2
from PyQuadruped.Spatial import ForceCrossProduct, SpatialTransformation, MotionCrossProduct, SpatialToHomog
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
		self.i_T_0 = np.eye(6)
		self.i_T_parent = None

		self.q_dot = 0
		self.tau = 0

		self.c = np.zeros((6, 1))
		self.v = np.zeros((6, 1))
		self.a = np.zeros((6, 1))
		self.u = np.zeros((6, 1))
		self.pa = np.zeros((6, 1))
		self.U  = None
		self.D = None
		
		self.S = GetJointSubspace(self.joint_axis)
		self.joint_vel_spatial = self.S*self.q_dot

	def AddChild(self, child):
		self.children.append(child)

	def GetTransformationMatrices(self, all_T):
		all_T.append(self.i_T_0)
		for child in self.children:
			child.GetTransformationMatrices(all_T)

	def SetAngle(self, i, state, tau):
		if self.parent is not None:
			self.q = state.q[i, 0]
			self.q_dot = state.q_dot[i, 0]
			self.tau = tau[i, 0]
			
			i += 1
			for child in self.children:
				child.SetAngle(i, state, tau)
		else:
			self.v = state.body_velocity
			leg = 0
			for child in self.children:
				child.SetAngle(leg*3 + i, state, tau)
				leg += 1

	# def Update(self, P_T_0): # Transformations according to the book
	# 	if self.joint_reversed:
	# 		R_joint = GetRotMat(-self.q, self.joint_axis)
	# 	else:
	# 		R_joint = GetRotMat(self.q, self.joint_axis)

	# 	self.J_T = SpatialTransformation(R_joint, np.zeros((3, 1)))

	# 	self.i_T_parent = self.J_T@self.local_T
	# 	self.i_T_0 = self.i_T_parent@P_T_0
		
	# 	for child in self.children:
	# 		child.Update(self.i_T_0)

	def Update(self, P_T_0):
		if self.parent is None:
			self.J_T = np.eye(6)

		else:
			if self.joint_reversed:
				R_joint = GetRotMat(-self.q, self.joint_axis)
			else:
				R_joint = GetRotMat(self.q, self.joint_axis)

			self.J_T = SpatialTransformation(R_joint, np.zeros((3, 1)))

		self.i_T_parent = self.J_T@self.local_T # Consider changing positiong of J_T and local_T as shown in the document
		self.i_T_0 = P_T_0@self.i_T_parent

		if self.parent is not None:
			# Do this for all the links other than the floating base
			vj = self.S*self.q_dot
			self.v = self.i_T_parent@self.parent.v + vj
			self.c = MotionCrossProduct(self.v, vj)
		
		self.articulated_inertia = self.inertia.copy()


		inertia_vel_product = self.inertia @ self.v
		self.pa = ForceCrossProduct(self.v, inertia_vel_product)


		for child in self.children:
			child.Update(self.i_T_0)


	def UpdateBottomUp(self):
		self.U = self.articulated_inertia@self.S
		self.D = self.S.T @ self.U

		Ia = self.i_T_parent.T @ self.articulated_inertia @ self.i_T_parent - self.U @ self.U.T / self.D
		self.parent.articulated_inertia += Ia

		self.u = self.tau - self.S.T @ self.pa - self.U.T @ self.c
		_pa = self.i_T_parent.T @ (self.pa + self.articulated_inertia @ self.c) + self.U @ self.u / self.D
		self.parent.pa += _pa
		
		if self.parent.parent is not None:
			self.parent.UpdateBottomUp()

	def UpdateTopDown(self, i, state_dot, g = None):
		if self.parent is None:
			a0 = -g
			self.a = self.i_T_parent@a0
			a_FB = np.linalg.inv(self.articulated_inertia)@(- self.pa - self.articulated_inertia.T @ self.a)
			self.a += a_FB
			
			i_R_parent = SpatialToHomog(self.i_T_parent)[:3, :3]
			state_dot.body_position_dot = i_R_parent.T @ self.v[:3, :]
			state_dot.body_veloctiy_dot = a_FB

			leg = 0
			for child in self.children:
				child.UpdateTopDown(leg*3 + i, state_dot)
				leg += 1
		
		else:
			state_dot.q_ddot[i, 0] = (self.u - self.U.T @ self.parent.a) / self.D;
			self.a = self.i_T_parent @ self.parent.a + self.S * state_dot.q_ddot[i, 0] + self.c

			i += 1

			for child in self.children:
				child.UpdateTopDown(i, state_dot)



	def __repr__(self):
		return "JointLink " + self.name

	def __str__(self):
		return self.__repr__()