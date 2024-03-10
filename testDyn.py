import numpy as np
from PyQuadruped.State import State
from PyQuadruped.RobotDynamics import RobotDynamics
import time


np.random.seed(1)

zero_state = State()

RD = RobotDynamics(np.random.random((6, 6)))



ang = 0.0
while True:
	zero_state.q[0, 0] = np.sin(ang)
	# zero_state.q[3, 0] = np.pi / 6
	# zero_state.q[6, 0] = np.pi / 6
	# zero_state.q[9, 0] = np.pi / 6
	RD.Step(zero_state)
	time.sleep(0.1)
	ang += 0.1