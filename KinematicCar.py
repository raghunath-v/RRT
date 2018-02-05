import json
from graphics import * 
import g_tools as g
from Path import *
from Goal import Goal
from math import atan, acos, sin, cos, sqrt, tan

class KinematicCar:
    def __init__(self, vel_start, pos_start, vel_max, length, phi_max, dt, win):
        # gives radius:  L/tan(max_phi)

        # dynamics related
        vel_x = vel_start[0]
        vel_y = vel_start[1]
        self.vel_start = vel_start
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # Specific to differential drive
        self.phi_max = phi_max
        self.length = length
        self.theta = atan(vel_y/vel_x)
        self.max_turn_radius = self.length/tan(self.phi_max) 
        # makes sense to me to set turning to 0 in refernece
        # to itself initially instead of an arbitrary 0 
        self.phi = self.theta
        self.vel_magnitude = sqrt(vel_x**2 + vel_y**2)
        
        # path planning related
        self.path = None
        self.finished = False
        self.sling_path = None
        self.sling_path_calculated = None
        self.sling_vel = None
        self.sling_acc = None
        self.total_time = 0

        # graphics related
        self.body_back = None # the actual position
        self.body_front = None
        self.direction_arrow = None
        self.turning_arrow = None
        self.sling_path_drawables = None
        self.body_back_radius = 10
        self.body_front_radius = 5
        self.win = win
        
    def set_velocity(self, goal):
        '''
            Change stuff here. Like I understand it,
            the velocity should be considered a scalar
            while theta, which is determined by phi, should
            control the direction of the vehicle. So here we
            can change the velocity scalar and the current phi
            while taking into consderation the constraints of 
            max_phi and max_velocity
        '''

    def move(self):
        self.pos_x+= self.vel_magnitude*cos(self.theta)*self.dt
        self.pos_y+= self.vel_magnitude*sin(self.theta)*self.dt
        # according to the lecture, this is the order of things
        self.theta+=(self.vel_magnitude/self.length)*tan(self.phi)*self.dt
        self.set_graphicals()

    def add_path(self, path):
        self.path = path

    def add_sling_path(self, goal):
         # A path is a list of nodes
        vel_series = get_velocity_series(self.path, self.vel_start, goal.vel, self.vel_max)
        self.sling_path, self.sling_vel = create_kinematic_sling_path(self.path, vel_series, self.max_turn_radius)
        self.sling_path_calculated = self.sling_path
        self.node_count_sling = len(self.sling_path)
        self.sling_path = [el for el in reversed(self.sling_path)]
        self.sling_vel = [el for el in reversed(self.sling_vel)]

    def set_graphicals(self):
        draw_back_x = g.scale(self.pos_x)
        draw_back_y = g.scale(self.pos_y)
        draw_front_x = g.scale(self.pos_x + self.length*cos(self.theta))
        draw_front_y = g.scale(self.pos_y + self.length*sin(self.theta))
        
        if self.body_back is not None:
            self.body_back.undraw()
        self.body_back = Circle(Point(draw_back_x, draw_back_y), self.body_back_radius)
        self.body_back.setFill('yellow')
        self.body_back.draw(self.win)
        if self.body_front is not None:
            self.body_front.undraw()
        self.body_front = Circle(Point(draw_front_x, draw_front_y), self.body_front_radius)
        self.body_front.setFill('yellow')
        self.body_front.draw(self.win)
        if self.turning_arrow is not None:
            self.turning_arrow.undraw()
        self.turning_arrow = Line(
            Point(draw_front_x, draw_front_y),
            Point(draw_front_x+5*cos(self.theta + self.phi), draw_front_y+5*sin(self.theta + self.phi))
        )
        self.turning_arrow.setFill('green')
        self.turning_arrow.setArrow('last')
        self.turning_arrow.draw(self.win)
        
        # Sling path related plotting

        if self.sling_path_calculated is not None:
            self.sling_path_drawables = [Circle(action[0].get_scaled_point(), 3) 
                for action in self.sling_path_calculated]
            for el in self.sling_path_drawables:
                el.draw(self.win)