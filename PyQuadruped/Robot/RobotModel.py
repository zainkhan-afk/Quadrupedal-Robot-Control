import numpy as np
from PyQuadruped.utils import GetRotMat, QuatToEulerRot
from PyQuadruped.Spatial import SpatialTransformation, SpatialToHomog
from PyQuadruped.Robot import Joint, Link
from PyQuadruped.State import StateDot

class RobotModel:
	def __init__(self, base_transformation):
		self.joints = []
		self.kinematic_tree = None
		self.kinematic_tree_feet = None
		self.base_transformation = base_transformation

		self.g = np.array([[0, 0, 0, 0, 0, 0]]).T

	def AssignKinematicTree(self, kinematic_tree):
		self.kinematic_tree = kinematic_tree

	def AssignKinematicTreeFeet(self, kinematic_tree_feet):
		self.kinematic_tree_feet = kinematic_tree_feet

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
		idx = 0
		for leg in self.kinematic_tree_feet:
			leg.UpdateBottomUp()
			idx += 1

		state_dot = StateDot() 
		self.kinematic_tree.UpdateTopDown(0, state_dot, g = self.g)

		return state_dot

	def ForwardKinematics(self, state, current_taus, external_forces):
		self.kinematic_tree.SetAngle(0, state, current_taus, external_forces)
		self.kinematic_tree.Update(SpatialTransformation(QuatToEulerRot(state.body_orientation), state.body_position))