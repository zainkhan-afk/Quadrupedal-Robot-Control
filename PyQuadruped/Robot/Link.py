class Link:
	def __init__(self, mass, inertia, COM, T):
		self.mass = mass
		self.inertia = inertia
		self.COM = COM
		self.local_T = T
		self.joint_rotated_T = None
		self.global_T = None