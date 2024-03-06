import numpy as np
from PyQuadruped.Spatial import SpatialInertia

body_width = 0.098
body_length = 0.38
abd_link_len = 0.062
hip_link_len = 0.209
knee_link_length = 0.195 
knee_link_Y_offset = 0.004 
max_leg_length = 0.409

# Inertia Parameters
floating_body_inertia = np.array([
									[11253, 0, 0],
									[0, 36203, 0],
									[0, 0, 42673]
								 ]) * 1e-6

floating_body_COM = np.array([[0], [0], [0]])
floating_body_mass = 3.3
floating_body_spatial_inertia = SpatialInertia(floating_body_mass, floating_body_inertia, floating_body_COM)

abd_inertia = np.array([
							[381, 58, 0.45],
							[58, 560, 0.95],
							[0.45, 0.95, 444]
					   ]) * 1e-6
abd_COM = np.array([[0], [0.036], [0]])
abd_mass = 0.54
abd_spatial_inertia = SpatialInertia(abd_mass, abd_inertia, abd_COM)


hip_inertia = np.array([
							[1983, 245, 13],
							[245, 2103, 1.5],
							[13, 1.5, 408]
						]) * 1e-6
hip_COM = np.array([[0], [0.016], [-0.02]])
hip_mass = 0.634
hip_spatial_inertia = SpatialInertia(hip_mass, hip_inertia, hip_COM)


knee_inertia = np.array([
							[6, 0, 0],
							[0, 248, 0],
							[0, 0, 245]
						]) * 1e-6
knee_COM = np.array([[0], [0], [-0.061]])
knee_mass = 0.064
knee_spatial_inertia = SpatialInertia(knee_mass, knee_inertia, knee_COM)

# Location parameters
abd_location = np.array([[body_length, body_width, 0]]).T * 0.5
hip_location = np.array([[0, abd_link_len, 0]]).T
knee_location = np.array([[0, 0, -hip_link_len]]).T


abd_joint_location = np.array([[body_length, body_width, 0]]).T * 0.5
hip_joint_location = np.array([[0, abd_link_len, 0]]).T
knee_joint_location = np.array([[0, 0, -hip_link_len]]).T