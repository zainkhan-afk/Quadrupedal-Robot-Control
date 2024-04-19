import numpy as np
import cv2

class Link:
	def __init__(self, inertia, T, name = ""):
		self.inertia = inertia
		self.articulated_inertia = inertia.copy()
		self.local_T = T
		self.name = name
		self.parent_joint = None 
		self.global_T = np.eye(6)
		self.v = np.eye(6)
		self.parent_T_child = None

		self.childred_joints = []

	def GetTransformationMatrices(self, all_T):
		all_T.append(self.global_T)
		for joint in self.childred_joints:
			joint.GetTransformationMatrices(all_T)

	def AddJoint(self, joint):
		self.childred_joints.append(joint)

	def Update(self, T):
		if self.parent_joint is not None:
			self.parent_T_child = self.parent_joint.J_T@self.local_T
			self.v = self.parent_T_child @ self.parent_joint.parent.v + self.parent_joint.joint_vel_spatial;
		
		self.articulated_inertia = inertia.copy()

		self.global_T = T@self.local_T
		for joint in self.childred_joints:
			joint.Update(self.global_T)

	def UpdateABA(self):
		pass


	def __repr__(self):
		return "Link " + self.name

	def __str__(self):
		return self.__repr__()