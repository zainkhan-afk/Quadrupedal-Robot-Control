import cv2
import numpy as np


class CVVisualizer:
	def __init__(self, width, height, scaler = 1):
		self.width = width
		self.height = height
		self.centerX = self.width // 2
		self.centerY = self.height // 2
		self.scaler = scaler
		
		self.Clear()

	def Clear(self):
		self.canvas = cv2.zeros((self.height, self.width, 3)).astype("uint8")

	def Draw(self, lines):
		for line in lines:
			x1 = line[0]
			y1 = line[1]
			x2 = line[2]
			y2 = line[3]

			self.canvas = cv2.line(self.canvas, (x1, y1), (x2, y2), (255, 0, 0), 1)


	def Render(self):
		cv2.imshow("canvas", self.canvas)
		return cv2.waitKey(1)