import numpy as np
import cv2

class Link:
	def __init__(self, inertia, T):
		self.inertia = inertia
		self.articulated_inertia = inertia.copy()
		self.local_T = T
		self.parent_joint = None 
		self.global_T = np.eye(6)

		self.childred_joints = []

	def GetTransformationMatrices(self, all_T):
		all_T.append(self.global_T)
		for joint in self.childred_joints:
			joint.GetTransformationMatrices(all_T)


	def AddJoint(self, joint):
		self.childred_joints.append(joint)

	def Update(self, T):
		self.global_T = T@self.local_T
		for joint in self.childred_joints:
			joint.Update(self.global_T)