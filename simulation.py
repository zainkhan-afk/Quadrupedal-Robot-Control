import pybullet as p
import pybullet_data
import time
import numpy as np

from robot import Robot
import ctypes
from cvVisualizer import CVVisualizer

class Simulation:
	def __init__(self, dll_filename, robot_filename):
		self.cpp_gait_ctrller = ctypes.cdll.LoadLibrary(dll_filename)
		self.cpp_gait_ctrller.Init()
		self.cpp_gait_ctrller.GetTorques.restype = ctypes.POINTER(ctypes.c_double * 12)
		self.cpp_gait_ctrller.GetAnglesForPosition.restype = ctypes.POINTER(ctypes.c_double * 12)

		self.visualizer = CVVisualizer(800, 800)

		physicsClient = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
		planeId = p.loadURDF("plane.urdf")


		p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
		p.setTimeStep(1/500) #THe lower this is, more accurate the simulation 
		p.setRealTimeSimulation(0)  # we want to be faster than real time :

		self.robot = Robot("urdf/mini_cheetah/mini_cheetah.urdf.xacro", [1,0,0.6], [0,0,0])

	def ConvertType(self, input):
		ctypes_map = {
			int: ctypes.c_int,
			float: ctypes.c_double,
			str: ctypes.c_char_p,
					 }
		input_type = type(input)
		if input_type is list:
			length = len(input)
			if length == 0:
				print("convert type failed...input is " + input)
				return 0
			else:
				arr = (ctypes_map[type(input[0])] * length)()
				for i in range(length):
					arr[i] = bytes(
						input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
				return arr
		else:
			if input_type in ctypes_map:
				return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
			else:
				print("convert type failed...input is "+input)
				return 0

	def Step(self):
		base_pose = p.getBasePositionAndOrientation(self.robot.robot) # returns base position [x, y, z] and angular pose in quat

		get_velocity = p.getBaseVelocity(self.robot.robot) # returns linear velocity [x, y, z] and angular velocity [wx, wy, wz]
		get_invert = p.invertTransform(base_pose[0], base_pose[1]) 
		get_matrix = p.getMatrixFromQuaternion(get_invert[1])

		joint_states = p.getJointStates(self.robot.robot, list(self.robot.joint_dict.keys()))

		body_pose = [0]*7
		motor_data = [0]*24

		body_pose[0] = base_pose[0][0]
		body_pose[1] = base_pose[0][1]
		body_pose[2] = base_pose[0][2]

		body_pose[3] = base_pose[1][0]
		body_pose[4] = base_pose[1][1]
		body_pose[5] = base_pose[1][2]
		body_pose[6] = base_pose[1][3]


		for i in range(12):
			motor_data[i] = joint_states[i][0]
			motor_data[12+i] = joint_states[i][1]

		# print(body_pose)
		# print(motor_data)
		# print(joint_states)

		# torques = self.cpp_gait_ctrller.GetTorques(self.ConvertType(body_pose), self.ConvertType(motor_data))
		q = self.cpp_gait_ctrller.GetAnglesForPosition(self.ConvertType(body_pose), self.ConvertType(motor_data))
		
		# p.setJointMotorControlArray(bodyUniqueId=self.robot.robot,
		# 							jointIndices=list(self.robot.joint_dict.keys()),
		# 							controlMode=p.TORQUE_CONTROL,
		# 							forces=list(torques.contents))

		p.setJointMotorControlArray(bodyUniqueId=self.robot.robot,
									jointIndices=list(self.robot.joint_dict.keys()),
									controlMode=p.POSITION_CONTROL,
									targetPositions=q.contents)
		
		# print(torques.contents)
		# print(torques.contents[0])
		# print(list(torques.contents))


		self.visualizer.Render()


		p.stepSimulation()

		time.sleep(1 / 10000)

	def Simulate(self):
		while True:
			self.Step()