from graphics import *
import numpy as np
import sys

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

def ray_intersects_segment(x, y, segment):
    eps = 0.00001
    huge = sys.float_info.max
    tiny = sys.float_info.min
    p_x = x
    p_y = y
    a_x = segment[0][0]
    a_y = segment[0][1]
    b_x = segment[1][0]
    b_y = segment[1][1]
    if a_y > b_y:
        a_y, b_y = b_y, a_y
        a_x, b_x = b_x, a_x
    if p_y == a_y or p_y == b_y:
        # increase by small amount to avoid overflow
        p_x+=eps
        p_y+=eps
 
    intersect = False

    # check if point in y range of segment or if 
    # point to right of segment
    if (p_y > b_y or p_y < a_y) or (
        p_x > max(a_x, b_x)):
        return False
    
    # if in range and to left, it intersects
    if p_x < min(a_x, b_x):
        intersect = True
    else:
        if abs(a_x - b_x) > tiny:
            m_red = (b_y - a_y) / float(b_x - a_x)
        else:
            m_red = huge
        if abs(a_x - p_x) > tiny:
            m_blue = (p_y - a_y) / float(p_x - a_x)
        else:
            m_blue = huge
        intersect = m_blue >= m_red
    return intersect