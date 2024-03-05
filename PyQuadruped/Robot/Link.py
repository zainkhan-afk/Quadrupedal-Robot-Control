class Link:
	def __init__(self, inertia, T):
		self.inertia = inertia
		self.local_T = T
		self.joint_rotated_T = None
		self.global_T = None