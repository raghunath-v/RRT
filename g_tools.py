from graphics import *
import numpy as np

def gen_point_list(points):
	# points: [[x_1, y_1],...,[x_N, y_N]]
	# have to make sure each 
	return [Point(x,y) for x,y in points]

def scale(x):
	scale_factor = 10
	offset = 100
	return scale_factor*x + offset

def scale_points(points):
	# points: [[x_1, y_1],...,[x_N, y_N]]
	return [[scale(x), scale(y)] for x,y in points]