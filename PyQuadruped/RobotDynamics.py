import numpy as np
from PyQuadruped.Robot.RobotModel import RobotModel
from PyQuadruped.RobotParams import floating_body_spatial_inertia as FB_I
from PyQuadruped.RobotParams import abd_spatial_inertia as A_I
from PyQuadruped.RobotParams import hip_spatial_inertia as H_I
from PyQuadruped.RobotParams import knee_spatial_inertia as K_I

from PyQuadruped.RobotParams import abd_location, knee_location, hip_location


from PyQuadruped.Spatial import SpatialTransformation, FlipSpatialInertia, SpatialToHomog

from PyQuadruped.utils import GetRotMat, GetLegSignedVector

from PyQuadruped.State import State

from PyQuadruped.Robot.Link import Link
from PyQuadruped.Robot.Joint import Joint
from PyQuadruped.Robot.RobotModel import RobotModel

from PyQuadruped.cvVisualizer import CVVisualizer

class RobotDynamics:
	def __init__(self, base_transformation):
		self.model = RobotModel(base_transformation)
		
		self.xz_vis = CVVisualizer(700, 700, title = "xz", scaler = 1000)
		self.yz_vis = CVVisualizer(700, 700, title = "yz", scaler = 1000)
		
		floating_body_link = Link(FB_I, np.eye(6))
		floating_body_link.global_T = np.eye(6)

		self.model.AssignKinematicTree(floating_body_link)
		# self.model.AddBody(inertia = FB_I, T = np.zeros((6, 6)), parent_id = 0, joint_axis_with_parent = None)

		# parent_id = 0
		side = -1
		R_ident = np.eye(3)
		for leg_i in range(4):
			FB_T_abd = SpatialTransformation(R_ident, GetLegSignedVector(abd_location, leg_i))
			abd_T_hip = SpatialTransformation(R_ident, GetLegSignedVector(hip_location, leg_i))
			hip_T_knee = SpatialTransformation(R_ident, GetLegSignedVector(knee_location, leg_i))

			if side < 0:
				abd_link = Link(FlipSpatialInertia(A_I, axis = 1), np.eye(6))
			else:
				abd_link = Link(A_I, np.eye(6))


			if side < 0:
				thigh_link = Link(FlipSpatialInertia(H_I, axis = 1), np.eye(6))
			else:
				thigh_link = Link(H_I, np.eye(6))

			if side < 0:
				shin_link = Link(FlipSpatialInertia(K_I, axis = 1), np.eye(6))
			else:
				shin_link = Link(K_I, np.eye(6))


			abd_joint = Joint(q = 0, T = FB_T_abd, axis = 0, child = abd_link)
			floating_body_link.AddJoint(abd_joint)
			
			hip_joint = Joint(q = 0, T = abd_T_hip, axis = 1, child = thigh_link)
			abd_link.AddJoint(hip_joint)
			
			knee_joint = Joint(q = 0, T = hip_T_knee, axis = 1, child = shin_link)
			thigh_link.AddJoint(knee_joint)

			
			self.model.AddJoint(abd_joint)
			self.model.AddJoint(hip_joint)
			self.model.AddJoint(hip_joint)

			side *= -1

	def Step(self, state):
		self.xz_vis.Clear()
		self.yz_vis.Clear()
		self.model.ForwardKinematics(state)


		xz_lines = []
		yz_lines = []

		all_T = []
		self.model.kinematic_tree.GetTransformationMatrices(all_T)
		print()
		for i in range(7 - 1):
			x1 = SpatialToHomog(all_T[i])[:3, -1].ravel()[0]
			y1 = SpatialToHomog(all_T[i])[:3, -1].ravel()[1]
			z1 = SpatialToHomog(all_T[i])[:3, -1].ravel()[2]

			x2 = SpatialToHomog(all_T[i + 1])[:3, -1].ravel()[0]
			y2 = SpatialToHomog(all_T[i + 1])[:3, -1].ravel()[1]
			z2 = SpatialToHomog(all_T[i + 1])[:3, -1].ravel()[2]

			xz_lines.append([x1, z1, x2, z2])
			yz_lines.append([y1, z1, y2, z2])
			print([x1, z1, x2, z2])



		self.xz_vis.Draw(xz_lines)
		self.yz_vis.Draw(yz_lines)

		k = self.xz_vis.Render()
		if k == ord("q"):
			exit()

		k = self.yz_vis.Render()
		if k == ord("q"):
			exit()




if __name__ == "__main__":
	np.random.seed(1)

	zero_state = State()
	zero_state.q[0, 0] = np.pi / 6

	RD = RobotDynamics(np.random.random((6, 6)))
	RD.Step(zero_state)