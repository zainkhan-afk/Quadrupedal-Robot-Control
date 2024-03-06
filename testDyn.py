import numpy as np
from PyQuadruped.State import State
from PyQuadruped.RobotDynamics import RobotDynamics


np.random.seed(1)

zero_state = State()
zero_state.q[0, 0] = np.pi / 6

RD = RobotDynamics(np.random.random((6, 6)))
RD.Step(zero_state)