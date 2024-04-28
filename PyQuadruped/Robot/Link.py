import numpy as np
import cv2
from PyQuadruped.Spatial import ForceCrossProduct

class Link:
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
		
		
		self.global_T = T@self.local_T
		for joint in self.childred_joints:
			joint.Update(self.global_T)

		self.articulated_inertia = self.inertia.copy()

		inertia_vel_product = self.inertia @ self.v;
		self.pa = ForceCrossProduct(self.v, inertia_vel_product);

	def ABAPass1(self):
		pass

	def ABAPass2(self):
		pass

	def ABAPass3(self):
		pass

	def UpdateABA(self):
		if self.parent_joint is not None:
			self.U = self.articulated_inertia@self.parent_joint.S

			self.d = self.parent_joint.S.T@self.U


			Ia = self.parent_T_child.T @ self.articulated_inertia @ self.parent_T_child - self.U @ self.U.T / self.d

			self.parent_joint.parent.articulated_inertia += Ia

			# print("Updating ABA - ", "Current ", self, "| Parent ", self.parent_joint.parent)

			self.parent_joint.parent.UpdateABA()

			if str(self.parent_joint.parent) == "Link floating base":
				print("--------------------------------------------------------------------------------------")
				for i in range(6):
					for j in range(6):
						print(round(self.parent_joint.parent.articulated_inertia[i, j], 2), end = "		")
					print()
				print("--------------------------------------------------------------------------------------")


	def __repr__(self):
		return "Link " + self.name

	def __str__(self):
		return self.__repr__()