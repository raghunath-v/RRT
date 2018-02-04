from graphics import Circle, Point, Line
import numpy as np
from g_tools import scale, scale_vectors
from Path import *
from Goal import Goal
from math import sqrt, cos, sin, atan
from Node import Node
from DubinCircle import DubinCircle


class DynamicPoint:
    def __init__(self, vel_start, pos_start, dt, vel_max, acc_max, win):
        # dynamics related
        self.vel_x = 0.1
        self.vel_y = 0.1
        self.vel_start = np.array(vel_start)
        self.vel_curr = np.array(vel_start)
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.acc_x = 0
        self.acc_y = 0
        self.dt = 0.05
        self.vel_max = vel_max
        self.acc_max = acc_max
        self.dist_max = vel_max * dt
        # path planning related
        self.finished = False
        self.path = None
        self.sling_path = None
        self.sling_path_calculated = None
        self.sling_vel = None
        self.sling_acc = None
        self.node_count = 0
        self.node_count_sling = 0
        self.at_node = True
        self.total_time = 0
        # graphics related
        self.body = None
        self.body_radius = 10
        self.vel_arrow = None
        self.acc_arrow = None
        self.win = win
        # for other stuff
        self.current_action = None
        self.current_acc = None
        self.current_vel = None
        self.new_action = True
        self.arc_length = 0
        self.theta = 0
        self.T = 0
        self.n = 0
        self.beta = 0
        self.circle = None

    def set_velocity(self, goal):
        # check if we have reached the next node
        if self.pos_x == self.sling_path[-1][0].x and self.pos_y == self.sling_path[-1][0].y or self.n == 0:
            # we have reached the next one, but is it
            # the goal node?
            if (len(self.sling_path) == 1):
                self.finished = True
                return
            else:
                self.current_action = self.sling_path.pop()
                self.current_vel = self.sling_vel.pop()
                self.vel_x, self.vel_y = self.current_vel[0], self.current_vel[1]
                self.current_acc = self.sling_acc.pop()
                self.new_action = True

        # are we going into a circle?
        if (self.current_action[1] != 0):
            if self.new_action:
                self.set_circle_params()
                self.T = self.arc_length / math.sqrt(self.current_vel[0] ** 2 + self.current_vel[1] ** 2)
                self.n = math.floor(self.T / self.dt)
                self.beta = self.theta / self.n
                self.new_action = False
            self.move_circular()
        else:
            # we are moving straight
            self.move()

        self.total_time += self.dt

    def move_circular(self):
        angle = atan(-self.vel_x / self.vel_y)
        self.acc_x = self.current_acc * cos(angle)
        self.acc_y = self.current_acc * sin(angle)

        self.vel_x = self.vel_x * cos(self.beta) - self.vel_y * sin(self.beta)
        self.vel_y = self.vel_x * sin(self.beta) + self.vel_y * cos(self.beta)

        self.pos_x = (self.pos_x - self.circle.c.x) * cos(self.beta) - (self.pos_y - self.circle.c.y) * sin(
            self.beta) + self.circle.c.x
        self.pos_y = (self.pos_x - self.circle.c.x) * sin(self.beta) + (self.pos_y - self.circle.c.y) * cos(
            self.beta) + self.circle.c.y

        self.n -= 1
        self.set_graphicals()

    def move(self):
        angle = atan(self.vel_y / self.vel_x)
        self.acc_x = self.current_acc * cos(angle)
        self.acc_y = self.current_acc * sin(angle)

        self.vel_x = self.vel_x + self.acc_x * self.dt
        self.vel_y = self.vel_y + self.acc_y * self.dt

        self.pos_x += self.dt * self.vel_x
        self.pos_y += self.dt * self.vel_y

        self.set_graphicals()

    def add_path(self, path):
        # A path is a list of nodes
        self.path = path
        self.node_count = len(path)

    def add_sling_path(self, goal):
        # A path is a list of nodes
        vel_series = get_velocity_series(self.path, self.vel_start, goal.vel, self.vel_max)
        acc_series = get_acceleration_series(self.path, self.acc_max)
        self.sling_path, self.sling_vel, self.sling_acc = create_sling_path(self.path, vel_series, acc_series)
        self.sling_path_calculated = self.sling_path
        print("Path Generated : ", self.sling_path_calculated)
        self.node_count_sling = len(self.sling_path)
        self.sling_path = [el for el in reversed(self.sling_path)]
        self.sling_vel = [el for el in reversed(self.sling_vel)]
        self.sling_acc = [el for el in reversed(self.sling_acc)]

    def set_circle_params(self):
        p1, p2 = self.current_action[0], self.sling_path[-1][0]
        vel_1 = self.current_vel
        dir_1 = self.current_action[1]
        slope = -vel_1[0] / vel_1[1]
        radius = abs(1 / dir_1)
        k = radius / math.sqrt(1 + slope ** 2)
        centre = Node(p1.x + k, p1.y + k * slope)
        circle = DubinCircle(centre, radius, dir_1)
        theta = circle.arcangle(p1, p2)
        self.arc_length = circle.arclength(p1, p2)
        self.theta = theta
        self.circle = circle

    def set_graphicals(self):
        draw_x = scale(self.pos_x)
        draw_y = scale(self.pos_y)

        # Draw the new path
        if self.sling_path_calculated is not None:
            for action in self.sling_path_calculated:
                self.body = Circle(action[0].get_scaled_point(), self.body_radius)
                self.body.setFill('yellow')
                self.body.draw(self.win)

        if self.circle is not None:
            dubinc = Circle(self.circle.c.get_scaled_point(), scale_vectors(self.circle.r))
            dubinc.setOutline('Green')
            dubinc.draw(self.win)

        if self.body:
            self.body.undraw()
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)
        if self.vel_arrow:
            self.vel_arrow.undraw()
        self.vel_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x + self.vel_x * 5), scale(self.pos_y + self.vel_y * 5)))
        self.vel_arrow.setFill('black')
        self.vel_arrow.setArrow("last")
        self.vel_arrow.draw(self.win)
        if self.acc_arrow:
            self.acc_arrow.undraw()
        self.acc_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x + self.acc_x * 5), scale(self.pos_y + self.acc_y * 5)))
        self.acc_arrow.setFill('blue')
        self.acc_arrow.setArrow('last')
        self.acc_arrow.draw(self.win)
        '''
        if self.sling_path is not None:
            for s in self.sling_path:
                node = s[0]
                direction = s[1]
                node = Circle(node.get_scaled_point(), 4)
                node.setFill('red')
                node.draw(self.win)
        '''

