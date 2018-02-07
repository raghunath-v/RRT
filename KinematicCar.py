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
        self.vel_x = vel_start[0]
        self.vel_y = vel_start[1]
        self.vel_start = vel_start
        self.vel_max = vel_max
        self.pos_x = pos_start[0]
        self.pos_y = pos_start[1]
        self.dt = dt
        self.dist_max = vel_max * dt
        
        # Specific to differential drive
        self.phi_max = phi_max
        self.length = length
        if self.vel_x == 0:
            self.vel_x+=1e-10
        self.theta = atan(self.vel_y/self.vel_x)
        self.max_turn_radius = self.length/tan(self.phi_max) 
        # makes sense to me to set turning to 0 in refernece
        # to itself initially instead of an arbitrary 0 
        self.phi = 0
        self.vel_magnitude = sqrt(self.vel_x**2 + self.vel_y**2)
        
        # path planning related
        self.path = None
        self.finished = False
        self.sling_path = None
        self.sling_path_calculated = None
        self.sling_vel = None
        self.total_time = 0

        # for circle/arc parameters
        # set to equate initial velocity
        #self.theta = 0
        self.T = 0
        self.n = 0
        self.beta = 0
        self.circle = None

        # graphics related
        self.body_back = None # the actual position
        self.body_front = None
        self.direction_arrow = None
        self.turning_arrow = None
        self.sling_path_drawables = None
        self.body_back_radius = 10
        self.body_front_radius = 5
        self.win = win

        self.tolerance = self.vel_max*self.dt
        self.current_action = None
        self.next_action = None
        
    def set_velocity(self, goal):
        self.vel_magnitude = self.vel_max
        self.total_time+=self.dt
        if (abs(self.pos_x - self.next_action[0].x) < self.tolerance) and (abs(self.pos_y - self.next_action[0].y) < self.tolerance):
            self.pos_x = self.next_action[0].x
            self.pos_y = self.next_action[0].y
            self.current_action = self.sling_path.pop()
            if len(self.sling_path) == 0:
                # we are done
                self.finished = True
                # TODO : This is a little bit of a cheat, it set's 
                # theta and phi to match up with goal at the end
                # it would be better to correct phi or something at
                # the end
                if goal.vel_x == 0:
                    goal.vel_x = 1e-30
                self.theta = atan(goal.vel_y/goal.vel_x)
                if goal.vel_x == 0:
                    goal.vel_x = 0
                self.phi = 0
                self.set_graphicals()
                return
            self.next_action = self.sling_path[-1]
            if(self.current_action[1] !=0):
                #self.circle = DubinCircle.fromArc(self.current_action[0], self.sling_path[-1][0], self.current_action[1])
                vel_x = self.vel_magnitude*cos(self.theta)
                vel_y = self.vel_magnitude*sin(self.theta)

                self.circle = DubinCircle.fromVel(self.current_action[0], self.current_action[1], np.array([vel_x,vel_y]))
            else:
                if (self.next_action[0].x - self.pos_x) < 0:
                    self.theta = math.pi + atan((self.pos_y-self.next_action[0].y)/(self.pos_x-self.next_action[0].x))
                else:
                    self.theta = atan((self.pos_y-self.next_action[0].y)/(self.pos_x-self.next_action[0].x))

        else:
            # keep doing the current action
            if self.current_action[1] != 0:
                self.move(rot=self.current_action[1])
            else:
                self.move()

    def move(self, rot=False):
        if rot:
            if rot < 0:
                self.phi = self.phi_max
            else:
                self.phi = - self.phi_max
        else:
            self.phi = 0
        self.pos_x+= self.vel_magnitude*cos(self.theta)*self.dt
        self.pos_y+= self.vel_magnitude*sin(self.theta)*self.dt
        # according to the lecture, this is the order of things
        self.theta += (self.vel_magnitude/self.length)*tan(self.phi)*self.dt
        print(self.theta)
        self.set_graphicals()

    def add_path(self, path):
        self.path = path

    def add_sling_path(self, goal, obstacles):
         # A path is a list of nodes
        vel_series = get_velocity_series(self.path, self.vel_start, goal.vel, self.vel_max)
        self.sling_path,_ = create_kinematic_sling_path(self.path, vel_series, self.max_turn_radius, obstacles=None)
        if not self.sling_path:
            return False
        self.sling_path = [el for el in reversed(self.sling_path)]
        self.current_action = self.sling_path.pop()
        self.next_action = self.sling_path[-1]
        self.circle = DubinCircle.fromArc(self.current_action[0], self.sling_path[-1][0], self.current_action[1])
        # draw once
        for el in self.sling_path:
            cir = Circle(el[0].get_scaled_point(), 3)
            cir.draw(self.win)

    def set_graphicals(self):
        draw_back_x = g.scale(self.pos_x)
        draw_back_y = g.scale(self.pos_y)
        draw_front_x = g.scale(self.pos_x + self.length*cos(self.theta))
        draw_front_y = g.scale(self.pos_y + self.length*sin(self.theta))
        
        if self.body_back is not None:
            pass
            #self.body_back.undraw()
        self.body_back = Circle(Point(draw_back_x, draw_back_y), self.body_back_radius/10)
        self.body_back.setFill('yellow')
        self.body_back.draw(self.win)
        if self.body_front is not None:
            pass
            #self.body_front.undraw()
        self.body_front = Circle(Point(draw_front_x, draw_front_y), self.body_front_radius/10)
        self.body_front.setFill('yellow')
        self.body_front.draw(self.win)
        if self.turning_arrow is not None:
            self.turning_arrow.undraw()
        self.turning_arrow = Line(
            Point(draw_front_x, draw_front_y),
            Point(draw_front_x+20*cos(self.theta + self.phi), draw_front_y+20*sin(self.theta + self.phi))
        )
        self.turning_arrow.setFill('green')
        self.turning_arrow.setArrow('last')
        self.turning_arrow.draw(self.win)
        
        # Sling path related plotting
        if self.circle is not None:
            dubinc = Circle(self.circle.c.get_scaled_point(), scale_vectors(self.circle.r))
            dubinc.setOutline('Green')
            #dubinc.draw(self.win)