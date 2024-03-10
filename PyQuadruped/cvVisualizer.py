import cv2
import numpy as np


class CVVisualizer:
	def __init__(self, width, height, title = "canvas", scaler = 1):
		self.width = width
		self.height = height
		self.origin_x = self.width // 2
		self.origin_y = self.height // 3
		self.scaler = scaler
		self.title = title
		
		self.Clear()

	def Clear(self):
		self.canvas = np.zeros((self.height, self.width, 3)).astype("uint8")

	def Draw(self, lines):
		for line in lines:
			x1 = self.origin_x + int(line[0]*self.scaler)
			y1 = self.origin_y - int(line[1]*self.scaler)
			x2 = self.origin_x + int(line[2]*self.scaler)
			y2 = self.origin_y - int(line[3]*self.scaler)
			# print(line)
			# print((x1, y1), (x2, y2))

			self.canvas = cv2.line(self.canvas, (x1, y1), (x2, y2), (255, 0, 0), 1)
			self.canvas = cv2.circle(self.canvas, (x1, y1), 10, (0, 255, 0), 1)
			self.canvas = cv2.circle(self.canvas, (x2, y2), 5, (0, 255, 255), 1)


	def Render(self):
		cv2.imshow(self.title, self.canvas)
		return cv2.waitKey(1)