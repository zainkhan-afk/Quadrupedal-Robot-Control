import numpy as np
from RobotModel import RobotModel
from RobotParams import floating_body_spatial_inertia as FB_I
from RobotParams import abd_spatial_inertia as A_I
from RobotParams import hip_spatial_inertia as H_I
from RobotParams import knee_spatial_inertia as K_I

from RobotParams import abd_location, knee_location, hip_location

class RobotDynamics:
	def __init__(self, base_transformation):
		self.model = RobotModel(base_transformation)