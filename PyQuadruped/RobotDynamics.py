import numpy as np
from RobotModel import RobotModel
from RobotParams import floating_body_spatial_inertia as FB_I
from RobotParams import abd_spatial_inertia as A_I
from RobotParams import hip_spatial_inertia as H_I
from RobotParams import knee_spatial_inertia as K_I

from RobotParams import abd_location, knee_location, hip_location


from Spatial import SpatialTransformation, FlipSpatialInertia
from utils import GetRotMat, GetLegSignedVector

from State import State

from PyQuadruped.Robot import Link, RobotModel

class RobotDynamics:
	def __init__(self, base_transformation):
		self.model = RobotModel(base_transformation)
		floating_body_link = Link(FB_I, np.zeros(6, 6))
		# self.model.AddBody(inertia = FB_I, T = np.zeros((6, 6)), parent_id = 0, joint_axis_with_parent = None)

		# parent_id = 0
		side = -1
		R_ident = np.eye(3)
		for leg_i in range(4):
			abd_T = SpatialTransformation(R_ident, GetLegSignedVector(abd_location, leg_i))
			if side < 0:
				abd_link = Link(FlipSpatialInertia(A_I, axis = 1), abd_T)
				# self.model.AddBody(inertia = FlipSpatialInertia(A_I, axis = 1), 
				# 					T = abd_T, 
				# 					parent_id = 0, 
				# 					joint_axis_with_parent = 0)
			else:
				abd_link = Link(A_I, abd_T)
				# self.model.AddBody(inertia = A_I, T = abd_T, 
				# 					parent_id = 0,
				# 					joint_axis_with_parent = 0)

			# parent_id += 1

			hip_T = SpatialTransformation(GetRotMat(np.pi, 2), GetLegSignedVector(hip_location, leg_i))
			if side < 0:
				thigh_link = Link(FlipSpatialInertia(H_I, axis = 1), hip_T)
				# self.model.AddBody(inertia = FlipSpatialInertia(H_I, axis = 1), 
				# 					T = hip_T, 
				# 					parent_id = parent_id, 
				# 					joint_axis_with_parent = 1)
			else:
				thigh_link = Link(H_I, hip_T)
				# self.model.AddBody(inertia = H_I, T = hip_T, 
				# 					parent_id = parent_id,
				# 					joint_axis_with_parent = 1)	

			# parent_id += 1

			knee_T = SpatialTransformation(R_ident, GetLegSignedVector(knee_location, leg_i))
			if side < 0:
				shin_link = Link(FlipSpatialInertia(K_I, axis = 1), knee_T)
				# self.model.AddBody(inertia = FlipSpatialInertia(K_I, axis = 1), 
				# 					T = knee_T, 
				# 					parent_id = parent_id, 
				# 					joint_axis_with_parent = 1)
			else:
				shin_link = Link(K_I, knee_T)
				# self.model.AddBody(inertia = K_I, T = knee_T, 
				# 					parent_id = parent_id,
				# 					joint_axis_with_parent = 1)	




			side *= -1

	def Step(self, state):
		self.model.ForwardKinematics(state)

if __name__ == "__main__":
	np.random.seed(1)

	zero_state = State()
	zero_state.q[0, 0] = np.pi / 6

	RD = RobotDynamics(np.random.random((6, 6)))
	RD.Step(zero_state)