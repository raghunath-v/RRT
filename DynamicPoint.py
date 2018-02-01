from graphics import *
import numpy as np
import g_tools as g
from Goal import Goal

class DynamicPoint:
    def __init__(self, vel_start, pos_start, dt, vel_max, acc_max, win):
        self.win = win
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.acc_x = 0
        self.acc_y = 0
        self.dt = dt
        self.vel_max = vel_max
        self.acc_max = acc_max
        self.dist_max = vel_max * dt
        self.finished = False
        self.body = None
        self.vel_arrow = None
        self.acc_arrow = None
        self.body_radius = 10
        self.set_graphicals()
        self.counter = 0

    def set_velocity(self, goal):
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
        #self.vel_x = self.vel_max * dir_unit_x
        #self.vel_y = self.vel_max * dir_unit_y
        self.acc_x = self.acc_max * dir_unit_x
        self.acc_y = self.acc_x * dir_unit_y
        # hit goal exactly in next time step if we can
        if dist < self.dist_max:
            v_fin = dist / self.dt
            self.vel_x = v_fin * dir_unit_x
            self.vel_y = v_fin * dir_unit_y

    def move(self):
        self.vel_x = self.vel_x + self.acc_x * self.dt
        self.vel_y = self.vel_y + self.acc_y * self.dt
        self.pos_x += self.dt*self.vel_x
        self.pos_y += self.dt*self.vel_y
        self.set_graphicals()

    def draw(self):
        # draw player
        self.body.draw(self.win)
        self.vel_arrow.draw(self.win)
        self.acc_arrow.draw(self.win)
    def set_graphicals(self):
        draw_x = g.scale(self.pos_x)
        draw_y = g.scale(self.pos_y)
        if self.body:
            self.body.undraw()
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)
        if self.vel_arrow:
            self.vel_arrow.undraw()
        self.vel_arrow = Line(
            Point(draw_x, draw_y),
            Point(g.scale(self.pos_x +self.vel_x), g.scale(self.pos_y + self.vel_y)))
        self.vel_arrow.setFill('black')
        self.vel_arrow.setArrow("last")
        self.vel_arrow.draw(self.win)
        if self.acc_arrow:
            self.acc_arrow.undraw()
        self.acc_arrow = Line(
            Point(draw_x, draw_y),
            Point(g.scale(self.pos_x +self.acc_x), g.scale(self.pos_y + self.acc_y)))
        self.acc_arrow.setFill('blue')
        self.acc_arrow.setArrow('last')
        self.acc_arrow.draw(self.win)

        
