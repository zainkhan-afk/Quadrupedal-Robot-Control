import pybullet as p
import pybullet_data
import time
import numpy as np

from robot import Robot
import ctypes

class Simulation:
	def __init__(self, dll_filename, robot_filename):
		self.cpp_gait_ctrller = ctypes.cdll.LoadLibrary(dll_filename)
		self.cpp_gait_ctrller.Init()

		physicsClient = p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
		planeId = p.loadURDF("plane.urdf")



		p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
		p.setTimeStep(1/500) #THe lower this is, more accurate the simulation 
		p.setRealTimeSimulation(0)  # we want to be faster than real time :

		self.robot = Robot("urdf/mini_cheetah/mini_cheetah.urdf.xacro", [1,0,0.5], [0,0,0])

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

		# print(base_pose[0])
		body_pose = [0]*7
		motor_data = [0]*24

		torques = self.cpp_gait_ctrller.GetTorques(self.ConvertType(body_pose), self.ConvertType(motor_data))



		p.stepSimulation()

		time.sleep(1 / 500)

	def Simulate(self):
		while True:
			self.Step()