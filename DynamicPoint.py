from graphics import Circle, Point, Line
import numpy as np
from g_tools import scale
from Path import *
from Goal import Goal

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

    def set_velocity(self, goal):

        self.acc_x = (goal.vel_x**2  - self.vel_x**2) / 2*np.sqrt((self.pos_x - goal.pos_x)**2 + (self.pos_y - goal.pos_y)**2)
        self.acc_y = (goal.vel_y**2 - self.vel_y**2) / 2*np.sqrt((self.pos_x - goal.pos_x)**2 + (self.pos_y - goal.pos_y)**2)

        # goal is of type Goal
        dist = np.sqrt((self.pos_x - goal.pos_x)**2 + (self.pos_y - goal.pos_y)**2)
        # if the distance is within some reasonable limit
        if dist <= 0.1:
            self.vel_x = goal.vel_x
            self.vel_y = goal.vel_y
            self.finished = True
            return
        
        # Aim at goal
        dir_x = goal.pos_x - self.pos_x
        dir_y = goal.pos_y - self.pos_y
        dir_len =  np.sqrt(dir_x**2 + dir_y**2)
        dir_unit_x = dir_x/dir_len
        dir_unit_y = dir_y/dir_len
                #self.acc_x = self.acc_max * dir_unit_x
        #self.acc_y = self.acc_x * dir_unit_y
        # hit goal exactly in next time step if we can
        #if dist < self.dist_max:
        #    v_fin = dist / self.dt
        #    self.vel_x = v_fin * dir_unit_x
        #    self.vel_y = v_fin * dir_unit_y

        self.total_time+=self.dt

    def move(self):
        self.vel_x = self.vel_x + self.acc_x * self.dt
        self.vel_y = self.vel_y + self.acc_y * self.dt
        self.pos_x += self.dt*self.vel_x
        self.pos_y += self.dt*self.vel_y
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
        self.node_count_sling = len(self.sling_path)

    def set_graphicals(self):
        draw_x = scale(self.pos_x)
        draw_y = scale(self.pos_y)
        if self.body:
            self.body.undraw()
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)
        if self.vel_arrow:
            self.vel_arrow.undraw()
        self.vel_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x +self.vel_x), scale(self.pos_y + self.vel_y)))
        self.vel_arrow.setFill('black')
        self.vel_arrow.setArrow("last")
        self.vel_arrow.draw(self.win)
        if self.acc_arrow:
            self.acc_arrow.undraw()
        self.acc_arrow = Line(
            Point(draw_x, draw_y),
            Point(scale(self.pos_x +self.acc_x), scale(self.pos_y + self.acc_y)))
        self.acc_arrow.setFill('blue')
        self.acc_arrow.setArrow('last')
        self.acc_arrow.draw(self.win)
        if self.sling_path is not None:
            print(self.sling_path)
            for s in self.sling_path:
                '''
                [[node,dir], [node, dir], ...]
                    [
                        [Node, dir],
                        [Node, dir],
                        ...
                    ]
                '''
                print(s)
                node = Circle(s[0][0].get_point(), 4)
                node.setFill('red')
                node.draw(self.win)

        
