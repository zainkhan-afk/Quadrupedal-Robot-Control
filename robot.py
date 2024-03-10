import pybullet as p
import numpy as np
# from utils import *

class Robot:
	def __init__(self, urdf_path, start_pos, start_orientation):
		self.start_pos = start_pos
		self.start_orientation = p.getQuaternionFromEuler(start_orientation)
		
		self.spinning_friction = 0.0065
		self.P = 1
		self.I = 0
		self.D = 0.01

		self.robot = p.loadURDF(urdf_path, self.start_pos, self.start_orientation, useFixedBase=False)
		p.changeDynamics(self.robot, 3, spinningFriction=self.spinning_friction)
		p.changeDynamics(self.robot, 7, spinningFriction=self.spinning_friction)
		p.changeDynamics(self.robot, 11, spinningFriction=self.spinning_friction)
		p.changeDynamics(self.robot, 15, spinningFriction=self.spinning_friction)

		self.joint_dict = {}

		num_joints = p.getNumJoints(self.robot)
		for i in range(num_joints):
			joint_name = p.getJointInfo(self.robot, i)[1].decode("UTF-8")
			
			if 'toe' not in joint_name:
				self.joint_dict[i] = joint_name

		print(self.joint_dict.keys())

		self.joint_indices = self.joint_dict.keys()

		self.joint_position_prev_errors = np.zeros(12)
		self.joint_position_errors_sum = np.zeros(12)

	def move_joints(self, desired):
		joint_states = p.getJointStates(self.robot, self.joint_indices)

		current = []
		for val in joint_states:
			current.append(val[0])

		desired = np.array(desired)
		current = np.array(current)

		error = desired - current

		diff_error = error - self.joint_position_prev_errors
		self.joint_position_errors_sum += error

		val = self.P*error + self.D*diff_error + self.I*self.joint_position_errors_sum

		p.setJointMotorControlArray(self.robot, self.joint_indices, p.POSITION_CONTROL, targetPositions = val)

		self.joint_position_prev_errors = error.copy()