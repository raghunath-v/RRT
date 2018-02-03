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
        # for other stuff
        self.current_action = None

    def set_velocity(self, goal):
        # are we going into a circle?
        if(self.current_action[1] !=0):

            # we are going into a circle
            # have we arrived at the next node?
            if self.pos_x == self.sling_path[-1].x and 
                self.pos_y == self.sling_path[-1].y:
                # we have arrived at the net node
                self
            else:
                self.move_circular()
        else:
            # we are moving straight
            self.move()



        self.total_time+=self.dt

    def move_circular(self):

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
        self.sling_path = reversed(self.sling_path)
        self.sling_vel = reversed(self.sling_vel)
        self.sling_acc = reversed(self.sling_acc)
        self.current_action = self.sling_path.pop()
        self.current_vel = self.sling_vel.pop()
        self.current_acc = self.sling_acc.pop()

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
        '''
        if self.sling_path is not None:
            for s in self.sling_path:
                node = s[0]
                direction = s[1]
                node = Circle(node.get_scaled_point(), 4)
                node.setFill('red')
                node.draw(self.win)
        '''
        
