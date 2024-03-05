import numpy as np

class State:
	def __init__(self):
		self.body_position = np.zeros((3, 1))
		self.body_orientation = np.zeros((4, 1))

		self.body_velocity = np.zeros((6, 1))


		self.q = np.zeros((12, 1))
		self.q_dot = np.zeros((12, 1))

	def __repr__(self):
		return f"Position - ({round(self.body_position[0, 0], 2)}, {round(self.body_position[1, 0], 2)}, {round(self.body_position[2, 0], 2)})"

	def __str__(self):
		return self.__repr__()


class StateDot:
	def __init__(self):
		self.body_position_dot = np.zeros((3, 1))
		self.body_veloctiy_dot = np.zeros((6, 1))
		self.q_ddot = np.zeros((12, 1))
