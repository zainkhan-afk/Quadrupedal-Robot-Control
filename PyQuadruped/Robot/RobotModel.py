import numpy as np
from PyQuadruped.utils import GetRotMat
from PyQuadruped.Spatial import SpatialTransformation, SpatialToHomog
from PyQuadruped.Robot import Joint, Link

class RobotModel:
	def __init__(self, base_transformation):
		self.joints = []
		self.kinematic_tree = None
		self.kinematic_tree_legs = None
		self.base_transformation = base_transformation

	def AssignKinematicTree(self, kinematic_tree):
		self.kinematic_tree = kinematic_tree

	def AssignKinematicTreeLegs(self, kinematic_tree_legs):
		self.kinematic_tree_legs = kinematic_tree_legs

	def AddJoint(self, joint):
		self.joints.append(joint)

	def AddBody(self, inertia, T, parent_id, joint_axis_with_parent):
		self.inertias.append(inertia)
		self.local_transformations.append(T)
		self.parents.append(parent_id)
		self.joint_axis_with_parent_list.append(joint_axis_with_parent)
		self.after_rotation_transformations.append(np.eye(6))
		self.global_transformations.append(np.eye(6))

	def RunABA(self):
		pass


	def ForwardKinematics(self, state):
		for i in range(len(self.joints)):
			self.joints[i].SetAngle(state.q[i, 0])
			# if (i + 1) % 3 == 0:
			# 	print(SpatialToHomog(self.joints[i].child.global_T)[:3, -1].ravel())
			# 	print()
		self.kinematic_tree.Update(np.eye(6))