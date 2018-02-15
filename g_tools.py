from graphics import *
import numpy as np
import sys
import math

SCALE_FACTOR = 14

def get_sign(x):
    return x and (1, -1)[x < 0]

def gen_point_list(points):
    # points: [[x_1, y_1],...,[x_N, y_N]]
    # have to make sure each 
    return [Point(x,y) for x,y in points]

def scale(x):
    offset = 100
    return SCALE_FACTOR*x + offset

def scale_coordinate(x):
    offset = 100
    return SCALE_FACTOR*x + offset

def scale_vectors(x):
    return SCALE_FACTOR*x

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

def segments_intersect(ax, ay, bx, by, s):
    cx, cy = s[0][0], s[0][1]
    dx, dy = s[1][0], s[1][1]
    # calculate the four determinants
    a = np.linalg.det(np.array([
        [ax-cx, bx-cx],
        [ay-cy, by-cy]]))
    b = np.linalg.det(np.array([
        [cx-ax, dx-ax],
        [cy-ay, dy-ay]]))
    c = np.linalg.det(np.array([
        [ax-dx, bx-dx],
        [ay-dy, by-dy]]))
    d = np.linalg.det(np.array([
        [cx-bx, dx-bx],
        [cy-by, dy-by]]))
    return a*c < 0 and b*d < 0

def segment_intersects_circle(x_0, y_0, r, s):
    x_1,y_1 = s[0][0], s[0][1]
    x_2,y_2 = s[1][0], s[1][1]
    return abs( (x_2-x_1)*x_0 + (y_1-y_2)*y_0 + (x_1-x_2)*y_1 + x_1*(y_2-y_1) )/\
        math.sqrt((x_2 - x_1)**2 + (y_1 - y_2)**2) <= r

def emit_verbose(string, verbose, var=None):
    if verbose:
        if var:
            print(string+": "+str(var))
        else:
            print(string)