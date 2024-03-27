import numpy as np
import cv2

class Link:
	def __init__(self, inertia, T):
		self.inertia = inertia
		self.articulated_inertia = inertia.copy()
		self.local_T = T
		self.global_T = np.eye(6)

		self.joints = []

	def GetTransformationMatrices(self, all_T):
		all_T.append(self.global_T)
		for joint in self.joints:
			joint.GetTransformationMatrices(all_T)


	def AddJoint(self, joint):
		self.joints.append(joint)

	def Update(self, T):
		self.global_T = T@self.local_T
		for joint in self.joints:
			joint.Update(self.global_T)