import math as m
import numpy as np

def rotate(vector, degrees):
	pi_over_180 = m.pi / 180.0

	x = vector[0]
	y = vector[1]

	temp_x = ((x * m.cos(degrees * pi_over_180)) - (y * m.sin(degrees * pi_over_180)))
	temp_y = ((x * m.sin(degrees * pi_over_180)) + (y * m.cos(degrees * pi_over_180)))
	return (np.array([temp_x, temp_y]))