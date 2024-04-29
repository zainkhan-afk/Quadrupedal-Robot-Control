import pybullet as p
import pybullet_data
import time
import numpy as np

from robot import Robot

from PyQuadruped.RobotDynamics import RobotDynamics
from PyQuadruped.State import State


class Simulation:
	def __init__(self, robot_filename):
		physicsClient = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
		planeId = p.loadURDF("plane.urdf")


		p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
		p.setTimeStep(1/500) #THe lower this is, more accurate the simulation 
		p.setRealTimeSimulation(0)  # we want to be faster than real time :

		self.robot = Robot("urdf/mini_cheetah/mini_cheetah.urdf.xacro", [0,0,0.6], [0,0,0])
		self.dynamics = RobotDynamics(np.eye(6))

	def Step(self):
		base_pose = p.getBasePositionAndOrientation(self.robot.robot) # returns base position [x, y, z] and angular pose in quat

		get_velocity = p.getBaseVelocity(self.robot.robot) # returns linear velocity [x, y, z] and angular velocity [wx, wy, wz]
		get_invert = p.invertTransform(base_pose[0], base_pose[1]) 
		get_matrix = p.getMatrixFromQuaternion(get_invert[1])

		joint_states = p.getJointStates(self.robot.robot, list(self.robot.joint_dict.keys()))

		body_pose = [0]*7
		motor_data = [0]*24

		body_pose[3] = base_pose[1][0]
		body_pose[4] = base_pose[1][1]
		body_pose[5] = base_pose[1][2]
		body_pose[6] = base_pose[1][3]

		current_state = State()

		current_state.body_position[0, 0] = base_pose[0][0]
		current_state.body_position[1, 0] = base_pose[0][1]
		current_state.body_position[2, 0] = base_pose[0][2]

		current_state.body_orientation[0, 0] = base_pose[1][0]
		current_state.body_orientation[1, 0] = base_pose[1][1]
		current_state.body_orientation[2, 0] = base_pose[1][2]
		current_state.body_orientation[3, 0] = base_pose[1][3]

		for i in range(12):
			current_state.q[i, 0] = joint_states[i][0]
			current_state.q_dot[i, 0] = joint_states[i][1]

		self.dynamics.Step(current_state)

		desired = []

		for i in range(4):
			if i == 0:
				desired += [0.75, 0.5, 1]
			else:
				desired += [0, 0, 0]

		self.robot.move_joints(desired)

		p.stepSimulation()

		time.sleep(1 / 10000)

	def Simulate(self):
		while True:
			self.Step()