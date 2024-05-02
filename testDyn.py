import numpy as np
from PyQuadruped.State import State
from PyQuadruped.RobotDynamics import RobotDynamics
import time


TOTAL_TIMESTEPS = 1000
DT = 0.01

np.random.seed(1)

state = State()

state.body_position[0, 0] = 0
state.body_position[1, 0] = 0
state.body_position[2, 0] = 0

state.body_orientation[-1, 0] = 1

RD = RobotDynamics(np.eye(6))

current_taus = np.zeros((12, 1))
external_forces = np.zeros((6, 13))

external_forces[0, 0] = 1

for time_step in range(TOTAL_TIMESTEPS):
	print(time_step)
	state_dot = RD.Step(state, current_taus, external_forces)
	RD.Integrate(state, state_dot, DT)

	RD.

	time.sleep(0.05)