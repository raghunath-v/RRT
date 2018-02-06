import random
from collections import defaultdict
from graphics import *
import numpy as np
import math as math
from DubinCircle import *
from Node import Node
from g_tools import get_sign

INCREASE_RADIUS = 6

def get_dist(p1, p2):
    distance = math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[1])**2)
    return distance

def get_radius(v,acc):
    return math.sqrt(v[0]**2+v[1]**2)/math.sqrt(np.dot(acc, acc))

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
        print("Im swapping")

    theta = math.atan(C1.slope_to(C2))
    d = C1.dist_to(C2)

    #print(R1, R2)
    if(R1 + R2 > d):
       print(R1, R2)
       print(d)
    alpha = math.acos((R1-R2)/d)
    beta = math.acos((R1+R2)/d)

    #Outer tangents
    T1_1 = Node(C1.x + R1 * (math.cos(alpha + theta)),
                C1.y + R1 * (math.sin(alpha + theta)))
    T1_2 = Node(C2.x + R2 * (math.cos(math.pi - alpha + theta)),
                C2.y + R2 * (math.sin(math.pi - alpha + theta)))
    T2_1 = Node(C1.x + R1 * (math.cos(-alpha + theta)),
                C1.y + R1 * (math.sin(-alpha + theta)))
    T2_2 = Node(C2.x + R2 * (math.cos(-(math.pi - alpha) + theta)),
                C2.y + R2 * (math.sin(-(math.pi - alpha) + theta)))

    if check_swap and swapped:
        if (dir1 < 0 and dir2 < 0):
            #print("Tan1")
            tangents.append((T1_2, T1_1))
        if (dir1 > 0 and dir2 > 0):
            #print("Tan2")
            tangents.append((T2_2, T2_1))
    else:
        if (dir1 > 0 and dir2 > 0):
            #print("Tan3")
            tangents.append((T1_1, T1_2))
        if (dir1 < 0 and dir2 < 0):
            #print("Tan4")
            tangents.append((T2_1, T2_2))

    #Inner tangents
    T1_1 = Node(C1.x + R1 * (math.cos(beta + theta)),
                C1.y + R1 * (math.sin(beta + theta)))
    T1_2 = Node(C2.x + R2 * (math.cos(-(math.pi - beta) + theta)),
                C2.y + R2 * (math.sin(-(math.pi - beta) + theta)))
    T2_1 = Node(C1.x + R1 * (math.cos(-beta + theta)),
                C1.y + R1 * (math.sin(-beta + theta)))
    T2_2 = Node(C2.x + R2 * (math.cos(math.pi - beta + theta)),
                C2.y + R2 * (math.sin(math.pi - beta + theta)))

    if check_swap and swapped:
        if (dir1 > 0 and dir2 < 0):
            #print("Tan5")
            tangents.append((T1_2, T1_1))
        if (dir1 < 0 and dir2 > 0):
            #print("Tan6")
            tangents.append((T2_2, T2_1))
    else:
        if (dir1 > 0 and dir2 < 0):
            #print("Tan7")
            tangents.append((T1_1, T1_2))
        if (dir1 < 0 and dir2 > 0):
            #print("Tan8")
            tangents.append((T2_1, T2_2))

    return tangents

def get_velocity_at_circle(start_pos, start_vel, steer, final_pos):
    radius = abs(1/steer)
    side = math.sqrt((final_pos.x - start_pos.x)**2 + (final_pos.y - start_pos.y)**2)
    angle = math.acos(1 - (side**2/(2*radius**2)))
    sign = get_sign(steer)
    rotation_matrix = np.array([[math.cos(angle), sign*math.sin(angle)],
                              [sign*math.sin(angle), math.cos(angle)]])
    new_vel = (rotation_matrix @ start_vel.T).T
    return new_vel


def getBestDubinPath(P1, V1, A1, P2, V2, A2, kinematic=False, obstacles=None, bounding_area=None):
    #paths=['RSR', 'LSL', 'RSL', 'LSR']
    C1_1, C2_1 = getDubinCircles(P1, V1, A1, kinematic=kinematic)
    C1_2, C2_2 = getDubinCircles(P2, V2, A2, kinematic=kinematic)
    '''
    For some reason, no circle will pass this collision test
    C_1 = []
    C_2 = []
    c11 = False
    c21 = False
    c12 = False
    c22 = False
    for obs in obstacles:
        c11 = c11 or obs.intersects_with_circle(C1_1.c.x, C1_1.c.y, C1_1.r)
        c21 = c21 or obs.intersects_with_circle(C2_1.c.x, C2_1.c.y, C2_1.r)
        c12 = c12 or obs.intersects_with_circle(C1_2.c.x, C1_2.c.y, C1_2.r)
        c22 = c22 or obs.intersects_with_circle(C2_2.c.x, C2_2.c.y, C2_2.r)
    '''
    # throw out bad circles
    # currently we disregard any circle that is intersecting with
    # bounding area or any obstacle
    bestPath = []
    bestDist = math.inf
    for circle1 in [C1_1, C2_1]:
        for circle2 in [C1_2, C2_2]:
            tang = get_tangents(circle1, circle2)
            pathLength = circle1.arclength(P1, tang[0][0])
            pathLength += tang[0][0].dist_to(tang[0][1])

            pathLength += circle2.arclength(tang[0][1], P2)
            # check first of tangents intersect with obstacles
            intersect = False
            if obstacles is not None:
                for obs in obstacles:
                    if obs.intersects_with_segment(tang[0][0].x, tang[0][0].y, tang[0][1].x, tang[0][1].y):
                        intersect=True
            # check here for circle intersection
            # check collision on circles and tangents right
            if pathLength < bestDist and not intersect:

                bestDist = pathLength
                bestPath = [[P1, circle1.dir], [tang[0][0], 0], [tang[0][1], circle2.dir]]
    return bestPath

def getDubinCircles(node, vel, acc, kinematic=False):
    slope = -vel[0] / vel[1]
    if kinematic:
        radius = acc
    else:
        radius = get_radius(vel, acc)
    k = radius / math.sqrt(1 + slope ** 2)

    #Direction
    if vel[1] < 0:
        direc = -1/radius
    else:
        direc = 1/radius

    # First Circle
    centre1 = Node(node.x + k, node.y + k * slope)
    circle1 = DubinCircle(centre1, radius, direc)

    # Second Circle
    centre2 = Node(node.x - k, node.y - k * slope)
    circle2 = DubinCircle(centre2, radius, -direc)

    return circle1, circle2

def get_velocity_series(path, vel_start, vel_goal, vel_max):
    vel_series = [vel_start]
    for i in range(1, len(path)-1):
        theta = math.atan(path[i].slope_to(path[i+1]))
        vel_x = vel_max * math.cos(theta)
        vel_y = vel_max * math.sin(theta)
        vel_series.append(np.array([vel_x, vel_y]))
    vel_series.append(vel_goal)
    return vel_series

def get_acceleration_series(path, acc_max):
    # TODO: We reduce acceleration ALOT when we are going through a circle
    # find a way to NOT do that. SMALL RADIUS
    scaleAcc = 0.6
    acc_series = [scaleAcc * np.array([acc_max/1.414, acc_max/1.414])]
    for i in range(1, len(path)-1):
        acc_series.append(scaleAcc * np.array([acc_max/1.414, acc_max/1.414]))
    acc_series.append(scaleAcc * np.array([acc_max/1.414, acc_max/1.414]))
    return acc_series

def find_acc(u, v, S):
    u_mag = math.sqrt(u[0] ** 2 + u[1] ** 2)
    v_mag = math.sqrt(v[0] ** 2 + v[1] ** 2)
    #TODO Throw an error if u and v are not aligned
    angle = math.atan(u[1]/u[0])
    acc_mag = (v_mag ** 2 - u_mag ** 2) / (2 * S)
    acceleration = np.array([acc_mag*math.cos(angle), acc_mag*math.sin(angle)])
    return acceleration


def create_kinematic_sling_path(path, vel_series, turning_radius):
    new_path = []
    new_vel_series = []
    for i in range(len(path)-1):
        best_dubin_path = getBestDubinPath(path[i], vel_series[i], turning_radius,
            path[i+1], vel_series[i+1], turning_radius, kinematic=True)
        # Insert new nodes from dubin path
        new_path.append(best_dubin_path[0])     # The starting node in dubin path
        new_path.append(best_dubin_path[1])     # Tangent point T1
        new_path.append(best_dubin_path[2])     # Tangent point T2
        # Insert new velocities for dubin path
        new_vel_series.append(vel_series[i])    # Velocity at the start point of dubin
        new_vel_series.append(vel_series[i])    # Velocity at the tangent T1
        new_vel_series.append(vel_series[i+1])  # Velocity at the tangent T2
    # Append goal node, velocity and and acceleration
    new_path.append([path[len(path) - 1], 0])
    new_vel_series.append(vel_series[len(path) - 1])
    return new_path, new_vel_series

def create_sling_path(path, vel_series, acc_series, obstacles=None):
    #get_velocity_series(path, vel_start, vel_goal)
    #get_acceleration_series(path, acc_max)
    new_path = []
    new_vel_series = []
    new_acc_series = []
    for i in range(len(path)-1):
        best_dubin_path = getBestDubinPath(path[i], vel_series[i], acc_series[i],
                         path[i+1], vel_series[i+1], acc_series[i+1], obstacles=obstacles)

        # Insert new nodes from dubin path
        new_path.append(best_dubin_path[0])     # The starting node in dubin path
        new_path.append(best_dubin_path[1])     # Tangent point T1
        new_path.append(best_dubin_path[2])     # Tangent point T2

        # Insert new velocities for dubin path
        new_vel_series.append(vel_series[i])            # Velocity at the start point of dubin

        new_vel = get_velocity_at_circle(best_dubin_path[0][0], vel_series[i],
                                        best_dubin_path[0][1], best_dubin_path[1][0])  # Velocity at the tangent T1
        new_vel_series.append(new_vel)

        new_vel = get_velocity_at_circle(path[i+1], vel_series[i+1],
                                         best_dubin_path[2][1], best_dubin_path[2][0])  # Velocity at the tangent T2
        new_vel_series.append(new_vel)

        # Insert new accelerations for dubin path
        # Find the acceleration needed to go from tangent points 1 to 2
        angle = math.atan(-vel_series[i][0] / vel_series[i][1])
        acc_mag = math.sqrt(np.dot(acc_series[i], acc_series[i]))
        new_acc = np.array([acc_mag * math.cos(angle), acc_mag * math.sin(angle)])

        new_acc_series.append(new_acc)
        new_acc_series.append(find_acc(vel_series[i], vel_series[i+1],
                                       best_dubin_path[1][0].dist_to(best_dubin_path[2][0])))

        angle = math.atan(-vel_series[i+1][0] / vel_series[i+1][1])
        acc_mag = math.sqrt(np.dot(acc_series[i+1], acc_series[i+1]))
        new_acc = np.array([acc_mag * math.cos(angle), acc_mag * math.sin(angle)])
        new_acc_series.append(new_acc)

    # Append goal node, velocity and and acceleration
    new_path.append([path[len(path) - 1], 0])
    new_vel_series.append(vel_series[len(path) - 1])
    new_acc_series.append(acc_series[len(path) - 1])

    return ignore_silly_nodes(new_path, new_vel_series, new_acc_series)


def ignore_silly_nodes(path_ser, vel_ser, acc_ser):
    precision = 0.25
    add_entry = True
    new_path_ser = []
    new_vel_ser = []
    new_acc_ser = []
    for i in range(len(path_ser) - 1):
        if path_ser[i][1] != 0:
            if path_ser[i][0].dist_to(path_ser[i+1][0]) < precision:
                add_entry = False
                #path_ser[i][1] = path_ser[i+1][1]
        if add_entry == True:
            new_path_ser.append(path_ser[i])
            new_vel_ser.append(vel_ser[i])
            new_acc_ser.append(acc_ser[i])
        add_entry = True
    new_path_ser.append(path_ser[len(path_ser) - 1])
    new_vel_ser.append(vel_ser[len(path_ser) - 1])
    new_acc_ser.append(acc_ser[len(path_ser) - 1])
    return new_path_ser, new_vel_ser, new_acc_ser


if __name__=='__main__': #Test for dynamic
    goal = Node(10, 15)
    init = Node(1, 2)
    v_in = np.array([0.9, -0.2])
    v_fin = np.array([0.5, 0.5])
    a_max = 0.3

    canvas_width = 800
    canvas_height = 800
    win = GraphWin("area", canvas_width, canvas_height)
    win.yUp()

    C1_init, C2_init = getDubinCircles(init, v_in, a_max)
    C1_goal, C2_goal = getDubinCircles(goal, v_fin, a_max)
    dubin_path = getBestDubinPath(init, v_in, a_max, goal, v_fin, a_max)

    tangents = []
    for circ1 in [C1_init, C2_init]:
        for circ2 in [C1_goal, C2_goal]:
            tan = get_tangents(circ1, circ2)
            tangents.append((tan[0][0], tan[0][1]))

    # Draw everything
    g = Circle(init.get_scaled_point(), 5)
    g.setOutline('Green')
    g.setFill('Green')
    #g.draw(win)

    g = Circle(goal.get_scaled_point(), 5)
    g.setOutline('Red')
    g.setFill('Red')
    g.draw(win)

    #DubinCircle.fromArc(self.current_action[0], self.sling_path[-1][0], self.current_action[1])
    #dubin_circ = DubinCircle.fromArc(dubin_path[0][0], dubin_path[1][0], dubin_path[0][1])  #TODO: This will not work
    dubin_circ = DubinCircle.fromVel(dubin_path[0][0], dubin_path[0][1], v_in)
    g = Circle(dubin_circ.get_scaled_centre(), scale_vectors(dubin_circ.r))
    g.setOutline('Green')
    #g.draw(win)
    #print("Goal steer: ", dubin_path[2][1])
    #dubin_circ = DubinCircle.fromArc(dubin_path[2][0], goal, dubin_path[2][1])      #TODO: This will not work

    print("Status: ", goal, dubin_path[2][1], v_fin)
    dubin_circ = DubinCircle.fromVel(goal, dubin_path[2][1], v_fin)
    g = Circle(dubin_circ.get_scaled_centre(), scale_vectors(dubin_circ.r))
    g.setOutline('Red')
    g.draw(win)

    g = Circle(dubin_path[1][0].get_scaled_point(), 5)
    g.setOutline('Green')
    g.setFill('Green')
    #g.draw(win)

    g = Circle( dubin_path[2][0].get_scaled_point(), 5)
    g.setOutline('Blue')
    g.setFill('Blue')
    #g.draw(win)

    #Line(dubin_path[1][0].get_scaled_point(), dubin_path[2][0].get_scaled_point()).draw(win)




    # Draw velocity vectors
    new = Node(init.x + v_in[0], init.y + v_in[1])
    Line(init.get_scaled_point(), new.get_scaled_point()).draw(win)
    new = Node(goal.x + v_fin[0], goal.y + v_fin[1])
    Line(goal.get_scaled_point(), new.get_scaled_point()).draw(win)

    win.getMouse()
    win.close()


if __name__=='__test__':  #This works
    goal = Node(10, 15)
    init = Node(1, 2)
    v_in = np.array([0.9, -0.2])
    v_fin = np.array([0.5, -0.5])
    a_max = 0.3

    canvas_width = 800
    canvas_height = 800
    win = GraphWin("area", canvas_width, canvas_height)
    win.yUp()

    C1_i, C2_i = getDubinCircles(init, v_in, a_max)
    C1_g, C2_g = getDubinCircles(goal, v_fin, a_max)

    #dubin_path = getBestDubinPath(init, v_in, a_max, goal, v_fin, a_max)

    #print(C1_init.get_dir(), C1_goal.get_dir())
    tangents = []
    for circ1 in [C1_i, C2_i]:
        for circ2 in [C1_g, C2_g]:
            tan = get_tangents(circ1, circ2)
            tangents.append((tan[0][0], tan[0][1]))

    steer1 = C1_i.dir
    steer2 = C1_g.dir
    print(steer1)
    print(steer2)

    #C1_init = DubinCircle.fromArc(init, tangents[0][0], steer1)
    #C2_init = DubinCircle.fromArc(init, tangents[2][0], -steer1)
    #C1_goal = DubinCircle.fromArc(goal, tangents[0][1], steer2)
    #C2_goal = DubinCircle.fromArc(goal, tangents[1][1], -steer2)

    C1_init = DubinCircle.fromVel(init, steer1, v_in)
    C2_init = DubinCircle.fromVel(init, -steer1, v_in)
    C1_goal = DubinCircle.fromVel(goal, steer2, v_fin)
    C2_goal = DubinCircle.fromVel(goal, -steer2, v_fin)

    # Draw everything
    g = Circle(init.get_scaled_point(), 5)
    g.setOutline('Green')
    g.setFill('Green')
    g.draw(win)

    g = Circle(C1_init.get_scaled_centre(), scale_vectors(C1_init.r))
    g.setOutline('Green')
    g.draw(win)

    g = Circle(C2_init.get_scaled_centre(), scale_vectors(C2_init.r))
    g.setOutline('Green')
    g.draw(win)

    g = Circle(goal.get_scaled_point(), 5)
    g.setOutline('Red')
    g.setFill('Red')
    g.draw(win)

    g = Circle(C1_goal.get_scaled_centre(), scale_vectors(C1_goal.r))
    g.setOutline('Black')
    g.draw(win)

    g = Circle(C2_goal.get_scaled_centre(), scale_vectors(C2_goal.r))
    g.setOutline('Black')
    g.draw(win)


    #draw tangents
    for tan in tangents:
        g = Circle(tan[0].get_scaled_point(), 5)
        g.setOutline('Green')
        g.setFill('Green')
        g.draw(win)

        g = Circle(tan[1].get_scaled_point(), 5)
        g.setOutline('Blue')
        g.setFill('Blue')
        g.draw(win)

        Line(tan[0].get_scaled_point(), tan[1].get_scaled_point()).draw(win)

    #Draw velocity vectors
    new = Node(init.x + (v_in[0]*3), init.y + (v_in[1]*3))
    Line(init.get_scaled_point(), new.get_scaled_point()).draw(win)
    new = Node(goal.x + (v_fin[0]*3), goal.y + (v_fin[1]*3))
    Line(goal.get_scaled_point(), new.get_scaled_point()).draw(win)

    win.getMouse()
    win.close()

