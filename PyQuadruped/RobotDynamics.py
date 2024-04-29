import numpy as np
from PyQuadruped.Robot.RobotModel import RobotModel
from PyQuadruped.RobotParams import floating_body_spatial_inertia as FB_I
from PyQuadruped.RobotParams import abd_spatial_inertia as A_I
from PyQuadruped.RobotParams import hip_spatial_inertia as H_I
from PyQuadruped.RobotParams import knee_spatial_inertia as K_I

from PyQuadruped.RobotParams import abd_location, knee_location, hip_location, foot_location


from PyQuadruped.Spatial import SpatialTransformation, FlipSpatialInertia, SpatialToHomog

from PyQuadruped.utils import GetRotMat, GetLegSignedVector

from PyQuadruped.State import State

from PyQuadruped.Robot.JointLink import JointLink
from PyQuadruped.Robot.RobotModel import RobotModel

from PyQuadruped.cvVisualizer import CVVisualizer

class RobotDynamics:
	def __init__(self, base_transformation):
		self.model = RobotModel(base_transformation)
		
		self.xz_vis = CVVisualizer(700, 700, title = "xz", scaler = 500)
		self.yz_vis = CVVisualizer(700, 700, title = "yz", scaler = 500)
		
		floating_body_link = JointLink(FB_I, np.eye(6), 0, -1, False, parent = None, children = [], name = "Floating Base")
		# floating_body_link = Link(FB_I, np.eye(6), name = "floating base")
		# floating_body_link.global_T = np.eye(6)

		self.model.AssignKinematicTree(floating_body_link)
		# self.model.AddBody(inertia = FB_I, T = np.zeros((6, 6)), parent_id = 0, joint_axis_with_parent = None)

		# parent_id = 0
		feet = []
		side = -1
		R_ident = np.eye(3)
		for leg_i in range(4):
			FB_T_abd = SpatialTransformation(R_ident, GetLegSignedVector(abd_location, leg_i))
			abd_T_hip = SpatialTransformation(R_ident, GetLegSignedVector(hip_location, leg_i))
			hip_T_knee = SpatialTransformation(R_ident, GetLegSignedVector(knee_location, leg_i))

			if side < 0:
				abd_link = JointLink(FlipSpatialInertia(A_I, axis = 1), T = FB_T_abd, 
								q = 0, joint_axis = 0, joint_reversed = False, parent = floating_body_link, 
								children = [], name = "Abduction Link")

			else:
				abd_link = JointLink(A_I, T = FB_T_abd, q = 0, joint_axis = 0, 
								joint_reversed = False, parent = floating_body_link, 
								children = [], name = "Abduction Link")
				

			if side < 0:
				thigh_link = JointLink(FlipSpatialInertia(H_I, axis = 1), T = abd_T_hip, 
								q = 0, joint_axis = 1, joint_reversed = True, parent = abd_link, 
								children = [], name = "Thigh Link")
			else:
				thigh_link = JointLink(H_I, T = abd_T_hip, q = 0, 
								joint_axis = 1, joint_reversed = True, parent = abd_link, 
								children = [], name = "Thigh Link")
				
			if side < 0:
				shin_link = JointLink(FlipSpatialInertia(K_I, axis = 1), T = hip_T_knee, 
								q = 0, joint_axis = 1, joint_reversed = True, parent = thigh_link, 
								children = [], name = "Shin Link")
			else:
				shin_link = JointLink(K_I, T = hip_T_knee, q = 0, 
								joint_axis = 1, joint_reversed = True, parent = thigh_link, 
								children = [], name = "Shin Link")

			floating_body_link.AddChild(abd_link)
			abd_link.AddChild(thigh_link)
			thigh_link.AddChild(shin_link)

			# abd_joint = Joint(q = 0, T = FB_T_abd, axis = 0, parent = floating_body_link, child = abd_link, name = "abd")
			# floating_body_link.AddJoint(abd_joint)
			
			# hip_joint = Joint(q = 0, T = abd_T_hip, axis = 1, parent = abd_link, child = thigh_link, joint_reversed = True, name = "hip")
			# abd_link.AddJoint(hip_joint)
			
			# knee_joint = Joint(q = 0, T = hip_T_knee, axis = 1, parent = thigh_link, child = shin_link, joint_reversed = True, name = "knee")
			# thigh_link.AddJoint(knee_joint)

			
			# self.model.AddJoint(abd_joint)
			# self.model.AddJoint(hip_joint)
			# self.model.AddJoint(knee_joint)

			feet.append(shin_link)

			side *= -1

		self.model.AssignKinematicTreeFeet(feet)

	def DebugVisualization(self):
		xz_lines = []
		yz_lines = []

		all_T = []
		self.model.kinematic_tree.GetTransformationMatrices(all_T)

		body = SpatialToHomog(all_T[0])[:3, -1].ravel()

		x0 = body[0]
		y0 = body[1]
		z0 = body[2]
		
		for leg in [0, 1, 2, 3]:
			for i in range(1, 3):
				index = leg*3 + i
				# print(leg, i, index, len(all_T))
				if i == 1:
					x1 = x0
					y1 = y0
					z1 = z0

				else:
					x1 = SpatialToHomog(all_T[index])[:3, -1].ravel()[0]
					y1 = SpatialToHomog(all_T[index])[:3, -1].ravel()[1]
					z1 = SpatialToHomog(all_T[index])[:3, -1].ravel()[2]

				x2 = SpatialToHomog(all_T[index + 1])[:3, -1].ravel()[0]
				y2 = SpatialToHomog(all_T[index + 1])[:3, -1].ravel()[1]
				z2 = SpatialToHomog(all_T[index + 1])[:3, -1].ravel()[2]

				if i == 2:
					parent_T_ee = SpatialTransformation(np.eye(3), GetLegSignedVector(foot_location, leg))

					x_ee = SpatialToHomog(all_T[index + 1]@parent_T_ee)[:3, -1].ravel()[0]
					y_ee = SpatialToHomog(all_T[index + 1]@parent_T_ee)[:3, -1].ravel()[1]
					z_ee = SpatialToHomog(all_T[index + 1]@parent_T_ee)[:3, -1].ravel()[2]
				
				if leg in [0, 1]:
					yz_lines.append([y1, z1, y2, z2])
					if i == 2:
						yz_lines.append([y2, z2, y_ee, z_ee])


				if leg in [0, 2]:
					xz_lines.append([x1, z1, x2, z2])
					if i == 2:
						xz_lines.append([x2, z2, x_ee, z_ee])


		self.xz_vis.Draw(xz_lines)
		self.yz_vis.Draw(yz_lines)

		k = self.xz_vis.Render()
		if k == ord("q"):
			exit()

		k = self.yz_vis.Render()
		if k == ord("q"):
			exit()

	def Step(self, state):
		self.xz_vis.Clear()
		self.yz_vis.Clear()
		self.model.ForwardKinematics(state)
		self.model.RunABA()

		self.DebugVisualization()




if __name__ == "__main__":
	np.random.seed(1)

	zero_state = State()
	zero_state.q[0, 0] = np.pi / 6

	RD = RobotDynamics(np.random.random((6, 6)))
	RD.Step(zero_state)