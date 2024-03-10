# from simulation import Simulation

# sim = Simulation("QuadrupedControl/Build/QuadrupedControl.dll", "urdf/mini_cheetah/mini_cheetah.urdf.xacro")
# sim.Simulate()


from simulationPython import Simulation
sim = Simulation("urdf/mini_cheetah/mini_cheetah.urdf.xacro")
sim.Simulate()