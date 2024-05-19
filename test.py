import cv2
import numpy as np
from scipy.spatial.transform import Rotation as RTool
import math
# def RToQuat(RT):
# 	q = np.zeros((4, 1))
# 	R = RT.T.copy()

# 	RTrace = np.trace(R)

# 	if RTrace > 0.0:
# 		S = np.sqrt(RTrace + 1.0) * 2.0
		
# 		q[0, 0] = 0.25 * S;
# 		q[1, 0] = (R[2, 1] - R[1, 2]) / S
# 		q[2, 0] = (R[0, 2] - R[2, 0]) / S
# 		q[3, 0] = (R[1, 0] - R[0, 1]) / S
# 	elif((R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2])):
# 		S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
# 		q[0, 0] = (R[2, 1] - R[1, 2]) / S
# 		q[1, 0] = 0.25 * S
# 		q[2, 0] = (R[0, 1] + R[1, 0]) / S
# 		q[3, 0] = (R[0, 2] + R[2, 0]) / S
# 	elif (R[1, 1] > R[2, 2]): 
# 		S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
# 		q[0, 0] = (R[0, 2] - R[2, 0]) / S;
# 		q[1, 0] = (R[0, 1] + R[1, 0]) / S;
# 		q[2, 0] = 0.25 * S;
# 		q[3, 0] = (R[1, 2] + R[2, 1]) / S;
# 	else:
# 		S = np.sqrtf(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
# 		q[0, 0] = (R[1, 0] - R[0, 1]) / S
# 		q[1, 0] = (R[0, 2] + R[2, 0]) / S
# 		q[2, 0] = (R[1, 2] + R[2, 1]) / S
# 		q[3, 0] = 0.25 * S;

# 	return q

def RToQuat(m):
	#q0 = qw
	t = np.matrix.trace(m)
	q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
	q = np.zeros((4, 1))

	if(t > 0):
		t = np.sqrt(t + 1)
		q[3, 0] = 0.5 * t
		t = 0.5/t
		q[0, 0] = (m[2,1] - m[1,2]) * t
		q[1, 0] = (m[0,2] - m[2,0]) * t
		q[2, 0] = (m[1,0] - m[0,1]) * t

	else:
		i = 0
		if (m[1,1] > m[0,0]):
			i = 1
		if (m[2,2] > m[i,i]):
			i = 2
		j = (i+1)%3
		k = (j+1)%3

		t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
		q[i, 0] = 0.5 * t
		t = 0.5 / t
		q[3, 0] = (m[k,j] - m[j,k]) * t
		q[j, 0] = (m[j,i] + m[i,j]) * t
		q[k, 0] = (m[k,i] + m[i,k]) * t

	return q

def QuatToEuler(q):
	euler = np.zeros((3, 1))

	x = q[0, 0]
	y = q[1, 0]
	z = q[2, 0]
	w = q[3, 0]

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	euler[0, 0] = math.atan2(t0, t1)
 
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	euler[1, 0] = math.asin(t2)
 
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	euler[2, 0] = math.atan2(t3, t4)
 
	return euler

def ToHomog(R, p):
	H = np.zeros((4, 4))
	H[:3, :3] = R
	H[:3, -1] = p.ravel()
	H[-1, -1] = 1

	return H

def ToSpatial(R, p):
	X = np.zeros((6, 6))

	X[:3, :3] = R
	X[3:, 3:] = R

	Sp = VecToSkewMat(p)
	# X[3:, :3] = Sp@R
	X[3:, :3] = -Sp

	return X

# def ToSpatial2(R, p):
# 	X = np.zeros((6, 6))

# 	X[:3, :3] = R
# 	X[3:, 3:] = R

# 	Sp = VecToSkewMat(p)
# 	X[3:, :3] = Sp@R

# 	return X

def GetRotationFromSpatial(X):
	return X[:3, :3]


def GetTranslationFromSpatial(X):
	R = GetRotationFromSpatial(X)
	skewR = X[3:, :3]

	return SketMatToVec(-skewR)
	

# def GetTranslationFromSpatial2(X):
# 	R = GetRotationFromSpatial(X)
# 	skewR = X[3:, :3]

# 	return SketMatToVec(skewR@R.T)



def SketMatToVec(m):
	v = np.zeros((3, 1))

	v[0, 0] = m[2, 1] - m[1, 2]
	v[1, 0] = m[0, 2] - m[2, 0]
	v[2, 0] = m[1, 0] - m[0, 1]

	return v*0.5

def VecToSkewMat(v):
	x = v[0, 0]
	y = v[1, 0]
	z = v[2, 0]

	m = np.array([
				[ 0, -z,  y],
				[ z,  0, -x],
				[-y,  x,  0]
				])

	return m

def ToRotMat(r, p, y):
	sr = np.sin(r)
	cr = np.cos(r)

	sp = np.sin(p)
	cp = np.cos(p)

	sy = np.sin(y)
	cy = np.cos(y)

	RX = np.array([
					[1,  0,   0],
					[0, cr, -sr],
					[0, sr,  cr]
				   ])

	RY = np.array([
					[ cp, 0, sp],
					[  0, 1,  0],
					[-sp, 0, cp]
				   ])

	RZ = np.array([
					[cy, -sy, 0],
					[sy,  cy, 0],
					[ 0,   0, 1]
					])

	return RX@RY@RZ

def ToRad(deg):
	return deg / 180 * np.pi




to_spatial_func = ToSpatial
get_coords_func = GetTranslationFromSpatial

ang = 0

while True:
	img1 = np.zeros((400, 400, 3)).astype("uint8")
	img2 = np.zeros((400, 400, 3)).astype("uint8")
	img3 = np.zeros((400, 400, 3)).astype("uint8")

	theta1 = np.random.random() * 0
	theta2 = np.random.random() * 0
	theta3 = np.random.random() * 0

	theta3 = 25*np.sin(ang)

	X_b = 200
	Y_b = 200
	scale = 20
	w_P_a = np.array([[2, 0, 0]]).T*scale
	w_R_a = ToRotMat(0, ToRad(theta1), 0)

	a_P_b = np.array([[2, 3, 0]]).T*scale
	a_R_b = ToRotMat(ToRad(theta2), 0, 0)

	b_P_c = np.array([[2, -2, 0]]).T*scale
	b_R_c = ToRotMat(0, 0, ToRad(theta3))

	c_P_d = np.array([[2, 0, 0]]).T*scale
	c_R_d = ToRotMat(0, 0, 0)

	w_X_a = to_spatial_func(w_R_a, w_P_a)
	a_X_b = to_spatial_func(a_R_b, a_P_b)
	b_X_c = to_spatial_func(b_R_c, b_P_c)
	c_X_d = to_spatial_func(c_R_d, c_P_d)

	# Test 1
	w_X_b = w_X_a@a_X_b
	w_X_c = w_X_a@a_X_b@b_X_c
	w_X_d = w_X_a@a_X_b@b_X_c@c_X_d

	w_P_a_t = get_coords_func(w_X_a).ravel().astype("int")
	w_P_b_t = get_coords_func(w_X_b).ravel().astype("int")
	w_P_c_t = get_coords_func(w_X_c).ravel().astype("int")
	w_P_d_t = get_coords_func(w_X_d).ravel().astype("int")
	# print(w_P_a_t)
	# print(w_P_b_t)
	# print(w_P_c_t)
	print(w_P_d_t)
	# print()

	cv2.line(img1, (X_b, Y_b), (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (255, 0, 0), 1)
	cv2.line(img1, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (255, 0, 0), 1)
	cv2.line(img1, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (255, 0, 0), 1)
	cv2.line(img1, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (X_b+w_P_d_t[0], Y_b+w_P_d_t[1]), (255, 0, 0), 1)

	cv2.circle(img1, (X_b, Y_b), 10, (0, 255, 0))
	cv2.circle(img1, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), 10, (0, 255, 0))
	cv2.circle(img1, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), 10, (0, 255, 0))
	cv2.circle(img1, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), 10, (0, 255, 0))

	# Test 2
	w_X_b = a_X_b@w_X_a
	w_X_c = b_X_c@a_X_b@w_X_a
	w_X_d = c_X_d@b_X_c@a_X_b@w_X_a

	w_P_a_t = get_coords_func(w_X_a).ravel().astype("int")
	w_P_b_t = get_coords_func(w_X_b).ravel().astype("int")
	w_P_c_t = get_coords_func(w_X_c).ravel().astype("int")
	w_P_d_t = get_coords_func(w_X_d).ravel().astype("int")
	# print(w_P_a_t)
	# print(w_P_b_t)
	# print(w_P_c_t)
	# print(w_P_d_t)
	# print()

	cv2.line(img2, (X_b, Y_b), (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (255, 0, 0), 1)
	cv2.line(img2, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (255, 0, 0), 1)
	cv2.line(img2, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (255, 0, 0), 1)
	cv2.line(img2, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (X_b+w_P_d_t[0], Y_b+w_P_d_t[1]), (255, 0, 0), 1)

	cv2.circle(img2, (X_b, Y_b), 10, (0, 255, 0))
	cv2.circle(img2, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), 10, (0, 255, 0))
	cv2.circle(img2, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), 10, (0, 255, 0))
	cv2.circle(img2, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), 10, (0, 255, 0))


	# Homog
	w_T_a = ToHomog(w_R_a, w_P_a)
	a_T_b = ToHomog(a_R_b, a_P_b)
	b_T_c = ToHomog(b_R_c, b_P_c)
	c_T_d = ToHomog(c_R_d, c_P_d)

	w_T_b = w_T_a@a_T_b
	w_T_c = w_T_a@a_T_b@b_T_c
	w_T_d = w_T_a@a_T_b@b_T_c@c_T_d

	w_P_a_t = w_T_a[:3, -1].ravel().astype("int")
	w_P_b_t = w_T_b[:3, -1].ravel().astype("int")
	w_P_c_t = w_T_c[:3, -1].ravel().astype("int")
	w_P_d_t = w_T_d[:3, -1].ravel().astype("int")

	# print(w_P_a_t)
	# print(w_P_b_t)
	# print(w_P_c_t)
	print(w_P_d_t)
	print()


	cv2.line(img3, (X_b, Y_b), (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (255, 0, 0), 1)
	cv2.line(img3, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (255, 0, 0), 1)
	cv2.line(img3, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (255, 0, 0), 1)
	cv2.line(img3, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), (X_b+w_P_d_t[0], Y_b+w_P_d_t[1]), (255, 0, 0), 1)

	cv2.circle(img3, (X_b, Y_b), 10, (0, 255, 0))
	cv2.circle(img3, (X_b+w_P_a_t[0], Y_b+w_P_a_t[1]), 10, (0, 255, 0))
	cv2.circle(img3, (X_b+w_P_b_t[0], Y_b+w_P_b_t[1]), 10, (0, 255, 0))
	cv2.circle(img3, (X_b+w_P_c_t[0], Y_b+w_P_c_t[1]), 10, (0, 255, 0))

	cv2.imshow("test 1", img1)
	cv2.imshow("test 2", img2)
	cv2.imshow("test 3", img3)

	k = cv2.waitKey(100)

	ang += 0.1

	if k == ord("q"):
		break