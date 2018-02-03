from graphics import *
import numpy as np
import g_tools as g
from Goal import Goal

class KinematicPoint:
    def __init__(self, vel_start, pos_start, dt, vel_max, win):
        # dynamics related
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # path planning related
        self.path = None
        self.node_count = 0
        self.path_idx = 0
        self.at_node = True
        self.next_node = False
        self.finished = False
        self.total_time = 0
        
        # graphics related
        self.body = None
        self.arrow = None
        self.body_radius = 10
        self.win = win

    def set_velocity(self, goal):
        if self.at_node:
            if self.path_idx == self.node_count - 1:
                self.vel_x = goal.vel_x
                self.vel_y = goal.vel_y
                self.finished = True
                return
             # Aim at next node
            self.path_idx+=1
            self.next_node = self.path[self.path_idx]
            dir_x = self.next_node.x - self.pos_x
            dir_y = self.next_node.y - self.pos_y
            dir_len =  np.sqrt(dir_x**2 + dir_y**2)
            dir_unit_x = dir_x/dir_len
            dir_unit_y = dir_y/dir_len
            self.vel_x = self.vel_max * dir_unit_x
            self.vel_y = self.vel_max * dir_unit_y
            self.at_node = False
    
        dist = np.sqrt((self.pos_x - self.next_node.x)**2 + (self.pos_y - self.next_node.y)**2)

        # hit next node exactly in next time step if we can
        if dist < self.dist_max:
            v_fin = dist / self.dt
            dir_x = self.next_node.x - self.pos_x
            dir_y = self.next_node.y - self.pos_y
            dir_len =  np.sqrt(dir_x**2 + dir_y**2)
            dir_unit_x = dir_x/dir_len
            dir_unit_y = dir_y/dir_len
            self.vel_x = v_fin * dir_unit_x
            self.vel_y = v_fin * dir_unit_y
            self.at_node = True
        
        self.total_time+=self.dt

    def set_auto_velocity(self, goal):
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
        self.vel_x = self.vel_max * dir_unit_x
        self.vel_y = self.vel_max * dir_unit_y

        # hit goal exactly in next time step if we can
        if dist < self.dist_max:
            v_fin = dist / self.dt
            self.vel_x = v_fin * dir_unit_x
            self.vel_y = v_fin * dir_unit_y

    def move(self):
        self.pos_x += self.dt*self.vel_x
        self.pos_y += self.dt*self.vel_y
        self.set_graphicals()

    def add_path(self, path):
        # Path is a list of Nodes
        self.path = path
        self.node_count = len(path)

    def set_graphicals(self):
        draw_x = g.scale(self.pos_x)
        draw_y = g.scale(self.pos_y)
        if self.body is not None:
            self.body.undraw()
        self.body = Circle(Point(draw_x, draw_y), self.body_radius)
        self.body.setFill('yellow')
        self.body.draw(self.win)
        if self.arrow is not None:
            self.arrow.undraw()
        self.arrow = Line(
            Point(draw_x, draw_y),
            Point(g.scale(self.pos_x +self.vel_x), g.scale(self.pos_y + self.vel_y)))
        self.arrow.setFill('black')
        self.arrow.setArrow("last")
        self.arrow.draw(self.win)
