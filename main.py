import pybullet as p
import pybullet_data
import time
import numpy as np

from robot import Robot


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #Loads the plane urdf file
planeId = p.loadURDF("plane.urdf")



p.setGravity(0,0,-9.81, physicsClientId = physicsClient)
p.setTimeStep(1/500) #THe lower this is, more accurate the simulation 
p.setRealTimeSimulation(0)  # we want to be faster than real time :

robot = Robot("urdf/mini_cheetah/mini_cheetah.urdf.xacro", [1,0,0.5], [0,0,0])

while True:
	base_pose = p.getBasePositionAndOrientation(robot.robot) # returns base position [x, y, z] and angular pose in quat [wx, wy, wz]

	get_velocity = p.getBaseVelocity(robot.robot) # returns linear velocity [x, y, z] and angular velocity [wx, wy, wz]
	get_invert = p.invertTransform(base_pose[0], base_pose[1]) 
	get_matrix = p.getMatrixFromQuaternion(get_invert[1])

	print(base_pose)
	print(get_invert)
	print(get_matrix)
	print()
	# print(get_velocity)


	p.stepSimulation()

	time.sleep(1 / 500)