import random
from collections import defaultdict
from graphics import *
import numpy as np
import math as math
from DubinCircle import *
from Node import Node


def get_dist(p1, p2):
    distance = math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[1])**2)
    return distance

def get_radius(v,acc):
    return math.sqrt(v[0]**2+v[1]**2)/acc

def get_tangents(Circle1, Circle2):
    """The function returns only one tangent for directional circles"""
    C1 = Circle1.get_centre()
    R1 = Circle1.get_radius()
    dir1 = Circle1.get_dir()
    C2 = Circle2.get_centre()
    R2 = Circle2.get_radius()
    dir2 = Circle2.get_dir()

    check_swap = True
    swapped = False
    tangents = []

    if check_swap and (C1.x > C2.x):
        temp = R2
        R2 = R1
        R1 = temp
        temp = C2
        C2 = C1
        C1 = temp
        swapped = True
        print("Swapping")

    theta = math.atan(C1.slope_to(C2))
    d = C1.dist_to(C2)
    alpha = math.acos((R1-R2)/d)
    beta = math.acos((R1+R2)/d)

    #Outer tangents
    T1_1 = Node(math.ceil(C1.x + R1 * (math.cos(alpha + theta))),
                math.ceil(C1.y + R1 * (math.sin(alpha + theta))))
    T1_2 = Node(math.ceil(C2.x + R2 * (math.cos(math.pi - alpha + theta))),
                math.ceil(C2.y + R2 * (math.sin(math.pi - alpha + theta))))
    T2_1 = Node(math.ceil(C1.x + R1 * (math.cos(-alpha + theta))),
                math.ceil(C1.y + R1 * (math.sin(-alpha + theta))))
    T2_2 = Node(math.ceil(C2.x + R2 * (math.cos(-(math.pi - alpha) + theta))),
                math.ceil(C2.y + R2 * (math.sin(-(math.pi - alpha) + theta))))

    if check_swap and swapped:
        if (dir1 == 'L' and dir2 == 'L'):
            tangents.append((T1_2, T1_1))
        if (dir1 == 'R' and dir2 == 'R'):
            tangents.append((T2_2, T2_1))
    else:
        if (dir1 == 'L' and dir2 == 'L'):
            tangents.append((T1_1, T1_2))
        if (dir1 == 'R' and dir2 == 'R'):
            tangents.append((T2_1, T2_2))

    #Inner tangents
    T1_1 = Node(math.ceil(C1.x + R1 * (math.cos(beta + theta))),
                math.ceil(C1.y + R1 * (math.sin(beta + theta))))
    T1_2 = Node(math.ceil(C2.x + R2 * (math.cos(-(math.pi - beta) + theta))),
                math.ceil(C2.y + R2 * (math.sin(-(math.pi - beta) + theta))))
    T2_1 = Node(math.ceil(C1.x + R1 * (math.cos(-beta + theta))),
                math.ceil(C1.y + R1 * (math.sin(-beta + theta))))
    T2_2 = Node(math.ceil(C2.x + R2 * (math.cos(math.pi - beta + theta))),
                math.ceil(C2.y + R2 * (math.sin(math.pi - beta + theta))))

    if check_swap and swapped:
        if (dir1 == 'L' and dir2 == 'R'):
            tangents.append((T1_2, T1_1))
        if (dir1 == 'R' and dir2 == 'L'):
            tangents.append((T2_2, T2_1))
    else:
        if (dir1 == 'L' and dir2 == 'R'):
            tangents.append((T1_1, T1_2))
        if (dir1 == 'R' and dir2 == 'L'):
            tangents.append((T2_1, T2_2))

    return tangents


def getBestDubinPath(P1, V1, A1, P2, V2, A2):
    #paths=['RSR', 'LSL', 'RSL', 'LSR']

    C1_1, C2_1 = getDubinCircles(P1, V1, A1)
    C1_2, C2_2 = getDubinCircles(P2, V2, A2)

    bestPath = []
    bestDist = math.inf
    for circle1 in [C1_1, C2_1]:
        for circle2 in [C1_2, C2_2]:
            tang = get_tangents(circle1, circle2)
            pathLength = circle1.arclength(P1, tang[0][0])
            pathLength += tang[0][0].dist_to(tang[0][1])
            pathLength += circle2.arclength(P2, tang[0][1])

            if pathLength < bestDist:
                bestDist = pathLength
                bestPath = [[P1, circle1.dir], [tang[0][0], 0], [tang[0][1], circle2.dir]]

    return bestPath

def getDubinCircles(node, vel, acc):
    slope = -vel[0] / vel[1]
    radius = get_radius(vel, acc)
    k = radius / math.sqrt(1 + slope ** 2)

    # First Circle
    centre1 = Node(node.x + k, node.y + k * slope)
    if vel[1] > 0:
        direc = 'L'
    else:
        direc = 'R'
    circle1 = DubinCircle(centre1, radius, direc)

    # Second Circle
    centre2 = Node(node.x - k, node.y - k * slope)
    if vel[1] > 0:
        direc = 'L'
    else:
        direc = 'R'
    circle2 = DubinCircle(centre2, radius, direc)

    return circle1, circle2

def get_velocity_series(path, vel_start, vel_goal, vel_max):
    vel_series = [vel_start]
    for i in range(1,len(path)-1):
        vel_series.append(vel_max)
    vel_series.append(vel_goal)

def get_acceleration_series(path, acc_max):
    acc_series = [acc_max]
    for i in range(1,len(path)-1):
        acc_series.append(acc_max)
    acc_series.append(acc_max)

def find_acc(u, v, S):
    acceleration = (v ** 2 - u ** 2) / 2 * S
    return acceleration

def create_sling_path(path, vel_series, acc_series):
    #get_velocity_series(path, vel_start, vel_goal)
    #get_acceleration_series(path, acc_max)
    new_path = []
    new_vel_series = []
    new_acc_series = []
    for i in range(len(path)-1):
        best_dubin_path = getBestDubinPath(path[i], vel_series[i], acc_series[i],
                         path[i+1], vel_series[i+1], acc_series[i+1])
        new_path.append(best_dubin_path)
        new_vel_series.append(vel_series[i])
        new_vel_series.append(vel_series[i])
        new_vel_series.append(vel_series[i+1])
        new_acc_series.append(acc_series[i])
        #Find the acceleration needed to go from tangent points 1 to 2
        new_acc_series.append(find_acc(vel_series[i], vel_series[i+1],
                                       best_dubin_path[1][0].dist_to(best_dubin_path[2][0])))
        new_acc_series.append(acc_series[i+1])

    new_path.append([path[len(path) - 1], 0])
    new_vel_series.append(vel_series[len(path) - 1])
    new_acc_series.append(acc_series[len(path) - 1])
    return new_path, new_vel_series, new_acc_series


if __name__=='__main__': #Test for dynamic
    goal = Node(400, 300)
    init = Node(150, 200)
    v_in = np.array([-30, 30])
    v_fin = np.array([-20, -20])
    a_max = 0.5

    WIDTH = 600
    HEIGHT = 600

    C1_init, C2_init = getDubinCircles(init, v_in, a_max)
    C1_goal, C2_goal = getDubinCircles(goal, v_fin, a_max)

    dubin_path = getBestDubinPath(init, v_in, a_max, goal, v_fin, a_max)



    tangents = get_tangents(C1_init, C1_goal)





if __name__=='__test__':
    goal = Node(400, 300)
    init = Node(150, 200)
    v_in = np.array([-30, 30])
    v_fin = np.array([-20, -20])
    a_max = 0.5

    WIDTH = 600
    HEIGHT = 600

    C1_init, C2_init = getDubinCircles(init, v_in, a_max)
    C1_goal, C2_goal = getDubinCircles(goal, v_fin, a_max)

    dubin_path = getBestDubinPath(init, v_in, a_max, goal, v_fin, a_max)

    #print(C1_init.get_dir(), C1_goal.get_dir())

    tangents = get_tangents(C1_init, C1_goal)


    # Draw everything

    win = GraphWin("Graph", WIDTH, HEIGHT)
    g_init1 = Circle(init.get_point(), 5)
    g_init1.setOutline('Green')
    g_init1.setFill('Green')
    g_init1.draw(win)

    g_init1 = Circle(C1_init.get_point(), C1_init.get_radius())
    g_init1.setOutline('Green')
    #g_init1.setFill('Black')
    g_init1.draw(win)

    g_init1 = Circle(C2_init.get_point(), C2_init.get_radius())
    g_init1.setOutline('Green')
    #g_init1.setFill('Black')
    g_init1.draw(win)

    g_goal = Circle(goal.get_point(), 5)
    g_goal.setOutline('Red')
    g_goal.setFill('Red')
    g_goal.draw(win)

    g_goal = Circle(C1_goal.get_point(), C1_goal.get_radius())
    g_goal.setOutline('Black')
    #g_goal.setFill('Red')
    g_goal.draw(win)

    g_goal = Circle(C2_goal.get_point(), C2_goal.get_radius())
    g_goal.setOutline('Black')
    #g_goal.setFill('Red')
    g_goal.draw(win)


    #draw tangents
    for tan in tangents:
        g_goal = Circle(tan[0].get_point(), 5)
        g_goal.setOutline('Green')
        g_goal.setFill('Green')
        g_goal.draw(win)

        g_goal = Circle(tan[1].get_point(), 5)
        g_goal.setOutline('Blue')
        g_goal.setFill('Blue')
        g_goal.draw(win)

        Line(tan[0].get_point(),tan[1].get_point()).draw(win)

    #Draw velocity vectors
    new = Point(init.x + v_in[0], init.y + v_in[1])
    Line(init.get_point(), new).draw(win)
    new = Point(goal.x + v_fin[0], goal.y + v_fin[1])
    Line(goal.get_point(), new).draw(win)

    win.getMouse()
    win.close()

