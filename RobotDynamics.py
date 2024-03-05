import numpy as np
from RobotModel import RobotModel
from RobotParams import floating_body_spatial_inertia as FB_I
from RobotParams import abd_spatial_inertia as A_I
from RobotParams import hip_spatial_inertia as H_I
from RobotParams import knee_spatial_inertia as K_I

from RobotParams import abd_location, knee_location, hip_location


from spatial import SpatialTransformation
from utils import GetRotMat

class RobotDynamics:
	def __init__(self, base_transformation):
		self.model = RobotModel(base_transformation)
		self.model.AddBody(inertia = FB_I, T = np.zeros((6, 6)), parent_id = 0, joint_axis_with_parent = None)

		link_id = 0
		side = -1
		R_ident = np.eye(3)
		for leg_i in range(4):
			
			self.model.AddBody(inertia = A_I, T = abd_location, parent_id = 0, joint_axis_with_parent = 0)

			link_id += 1
			side *= -1